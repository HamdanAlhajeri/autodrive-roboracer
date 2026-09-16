"""Record actual simulator lap/collision and command telemetry; never controls the car."""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from .ros_utils import PREFIX, odom_state


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=600)
    parser.add_argument("--output", required=True)
    parser.add_argument("--stop-after-lap", action="store_true")
    args = parser.parse_args()
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("avlite_lap_recorder")
    state, initial = {}, {}
    counters = {"distance_m": 0.0, "resets": 0, "max_speed_mps": 0.0}
    previous = None

    def scalar(name, msg):
        # AutoDRIVE sends infinity until the first timed lap; keep valid JSON.
        value = msg.data if math.isfinite(msg.data) else None
        state[name] = value
        initial.setdefault(name, value)

    def odom(msg):
        nonlocal previous
        x, y, heading, speed = odom_state(msg)
        if previous is not None:
            delta = math.hypot(x - previous[0], y - previous[1])
            if delta > 1:
                counters["resets"] += 1
            else:
                counters["distance_m"] += delta
        previous = (x, y)
        state.update(x=x, y=y, heading=heading, speed=speed)
        counters["max_speed_mps"] = max(counters["max_speed_mps"], abs(speed))

    def command(msg):
        state.update(
            avlite_steer_rad=msg.drive.steering_angle, avlite_acceleration=msg.drive.acceleration
        )

    def reset(msg):
        if msg.data:
            counters["resets"] += 1

    for name in (
        "lap_count",
        "collision_count",
        "last_lap_time",
        "throttle_command",
        "steering_command",
        "throttle",
        "steering",
    ):
        kind = Int32 if name.endswith("count") else Float32
        node.create_subscription(kind, PREFIX + "/" + name, lambda m, n=name: scalar(n, m), 10)
    node.create_subscription(Odometry, PREFIX + "/odom", odom, 10)
    node.create_subscription(AckermannDriveStamped, "/avlite/control_command", command, 10)
    node.create_subscription(Bool, "/autodrive/reset_command", reset, 10)
    start, next_sample = time.monotonic(), 0
    completed = False
    lap_seen_at = None
    try:
        with path.open("w") as stream:
            while time.monotonic() - start < args.seconds:
                rclpy.spin_once(node, timeout_sec=0.05)
                elapsed = time.monotonic() - start
                if elapsed >= next_sample:
                    stream.write(json.dumps({"elapsed_s": elapsed, **state, **counters}) + "\n")
                    stream.flush()
                    next_sample = elapsed + 0.1
                completed = (
                    "lap_count" in initial and state.get("lap_count", 0) > initial["lap_count"]
                )
                if completed and lap_seen_at is None:
                    lap_seen_at = elapsed
                # Allow collision feedback from the finish frame to arrive too.
                if args.stop_after_lap and lap_seen_at is not None and elapsed > lap_seen_at + 1:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        summary = {
            "elapsed_s": time.monotonic() - start,
            "initial": initial,
            "final": state,
            **counters,
            "clean_lap": bool(
                completed
                and counters["resets"] == 0
                and "collision_count" in initial
                and state["collision_count"] == initial["collision_count"]
            ),
        }
        path.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
