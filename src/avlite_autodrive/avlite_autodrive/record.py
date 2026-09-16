"""Record actual simulator lap/collision and command telemetry; never controls the car."""

import argparse
import json
import math
import signal
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from .ros_utils import PREFIX, odom_state


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=600)
    parser.add_argument("--output", required=True)
    parser.add_argument("--stop-after-lap", action="store_true")
    parser.add_argument("--wait-for-odom", type=float, default=30,
                        help="Seconds to wait for valid odometry before starting the timer")
    parser.add_argument("--odom-timeout", type=float, default=10,
                        help="Fail if valid odometry stops for this many seconds")
    args = parser.parse_args()
    for name in ("seconds", "wait_for_odom", "odom_timeout"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("avlite_lap_recorder")
    state, initial = {}, {}
    counters = {"distance_m": 0.0, "resets": 0, "max_speed_mps": 0.0,
                "odometry_samples": 0}
    last_received = {}
    previous = None
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def scalar(name, msg):
        # AutoDRIVE sends infinity until the first timed lap; keep valid JSON.
        value = msg.data if math.isfinite(msg.data) else None
        state[name] = value
        initial.setdefault(name, value)
        last_received[name] = time.monotonic()

    def odom(msg):
        nonlocal previous
        try:
            x, y, heading, speed = odom_state(msg)
        except ValueError:
            counters["invalid_odom_samples"] = counters.get("invalid_odom_samples", 0) + 1
            return
        last_received["odom"] = time.monotonic()
        counters["odometry_samples"] += 1
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
        last_received["avlite_command"] = time.monotonic()
        state.update(
            avlite_steer_rad=(
                msg.drive.steering_angle if math.isfinite(msg.drive.steering_angle) else None
            ),
            avlite_acceleration=(
                msg.drive.acceleration if math.isfinite(msg.drive.acceleration) else None
            ),
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
    node.create_subscription(Odometry, PREFIX + "/odom", odom, qos_profile_sensor_data)
    node.create_subscription(AckermannDriveStamped, "/avlite/control_command", command, 10)
    node.create_subscription(Bool, "/autodrive/reset_command", reset, 10)
    waiting_since = time.monotonic()
    start, started_utc = None, None
    completed = False
    lap_seen_at = None
    error = None
    try:
        with path.open("w") as stream:
            print(f"Waiting up to {args.wait_for_odom:g}s for valid simulator odometry...",
                  flush=True)
            while not stopped and "odom" not in last_received:
                rclpy.spin_once(node, timeout_sec=0.05)
                if ("odom" not in last_received
                        and time.monotonic() - waiting_since >= args.wait_for_odom):
                    error = (
                        "No valid odometry received. Start/connect the simulator and finish "
                        "restarting the controllers before recording. If the bridge was "
                        "restarted, rerun the recording command to attach to its new connection."
                    )
                    break
            if not stopped and error is None:
                start = time.monotonic()
                started_utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                print(f"Odometry received. Recording {args.seconds:g}s of telemetry.", flush=True)
            next_sample = 0
            while (start is not None and not stopped
                   and time.monotonic() - start < args.seconds):
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                elapsed = now - start
                if now - last_received["odom"] >= args.odom_timeout:
                    error = (
                        f"Valid odometry stopped for {args.odom_timeout:g}s. Partial data saved. "
                        "Check the simulator connection, finish any controller/bridge restart, "
                        "then rerun the recording command."
                    )
                if elapsed >= next_sample:
                    sample = {
                        "timestamp_utc": datetime.now(timezone.utc).isoformat(
                            timespec="milliseconds"
                        ),
                        "elapsed_s": elapsed,
                        **state,
                        **counters,
                        **{key + "_age_s": now - received
                           for key, received in last_received.items()},
                    }
                    stream.write(json.dumps(sample, allow_nan=False) + "\n")
                    stream.flush()
                    next_sample = elapsed + 0.1
                if error is not None:
                    break
                completed = (
                    "lap_count" in initial and state.get("lap_count", 0) > initial["lap_count"]
                )
                if completed and lap_seen_at is None:
                    lap_seen_at = elapsed
                # Allow collision feedback from the finish frame to arrive too.
                if args.stop_after_lap and lap_seen_at is not None and elapsed > lap_seen_at + 1:
                    break
    except KeyboardInterrupt:
        stopped = True
    finally:
        summary = {
            "status": "failed" if error else "interrupted" if stopped else "completed",
            "error": error,
            "started_utc": started_utc,
            "ended_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "has_odometry": "speed" in state,
            "elapsed_s": time.monotonic() - start if start is not None else 0.0,
            "waiting_s": (start if start is not None else time.monotonic()) - waiting_since,
            "initial": initial,
            "final": state,
            **counters,
            "clean_lap": bool(
                completed
                and error is None
                and not stopped
                and counters["resets"] == 0
                and "collision_count" in initial
                and state["collision_count"] == initial["collision_count"]
            ),
        }
        path.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    if error:
        print(f"Recording failed: {error}", flush=True)
        return 2
    return 130 if stopped else 0


if __name__ == "__main__":
    raise SystemExit(main())
