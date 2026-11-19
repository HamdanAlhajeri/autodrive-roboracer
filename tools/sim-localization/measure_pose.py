"""Record what the localization sees and what the car does, for delay and error analysis.

Run inside the avlite container while the stack is driving (read-only apart from the optional
reset, which uses the simulator's own reset topic):

    python3 measure_pose.py OUT.jsonl --seconds 240 [--auto-reset]

One JSON line per message: ``k`` is the kind, ``tr`` the recorder's monotonic receive time and
``tn`` the ROS clock at receipt (to compare against message stamps). Kinds:

    truth   simulator ground truth (x, y, yaw, speed) and its header stamp
    wheel   wheel-odometry pose the localizer predicts from
    scan    LiDAR scan header stamp only
    diag    /avlite/controller_diagnostics (estimate, truth, alignment, path progress)
    cmd     /avlite/control_command (steering angle, speed, acceleration)
    lap, collision   the simulator's counters
    plan    the race plan (path and velocity), once
    reset   a reset this script issued, with the car's position and the reason

With --auto-reset it re-spawns the car when it has stood still for --stuck-s seconds, so a
crash does not end the measurement; each such reset is the crash site.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, String

PREFIX = "/autodrive/roboracer_1"


def odom_state(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw, msg.twist.twist.linear.x


def free_memory_mb():
    """Available memory of the container's VM in MB (the Docker VM, not Windows), or None."""
    try:
        with open("/proc/meminfo", encoding="ascii") as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) // 1024
    except OSError:
        pass
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--seconds", type=float, default=240.0)
    parser.add_argument("--auto-reset", action="store_true")
    parser.add_argument("--stuck-s", type=float, default=3.0)
    parser.add_argument("--require-motion-s", type=float, default=0.0,
                        help="abort (exit 3) if the car has not reached 0.5 m/s this many seconds "
                             "after the first truth message; catches a frozen simulator car")
    args = parser.parse_args()

    rclpy.init()
    node = Node("measure_pose")
    out = open(args.output, "w", encoding="utf-8")
    state = {"truth": None, "still_since": None, "last_reset": 0.0, "resets": 0, "release_at": None,
             "first_truth": None, "peak_v": 0.0}

    def now():
        return time.monotonic()

    def tn():
        t = node.get_clock().now().nanoseconds
        return t * 1e-9

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def write(kind, **fields):
        out.write(json.dumps({"k": kind, "tr": now(), "tn": tn(), **fields}) + "\n")

    def on_truth(msg):
        x, y, yaw, speed = odom_state(msg)
        write("truth", s=stamp(msg), x=x, y=y, yaw=yaw, v=speed, w=msg.twist.twist.angular.z)
        state["truth"] = (x, y, yaw, speed)
        state["first_truth"] = state["first_truth"] or now()
        state["peak_v"] = max(state["peak_v"], abs(speed or 0.0))
        if speed is not None and abs(speed) < 0.05:
            state["still_since"] = state["still_since"] or now()
        else:
            state["still_since"] = None

    def on_wheel(msg):
        x, y, yaw, speed = odom_state(msg)
        write("wheel", s=stamp(msg), x=x, y=y, yaw=yaw, v=speed, w=msg.twist.twist.angular.z)

    def on_diag(msg):
        try:
            write("diag", d=json.loads(msg.data))
        except ValueError:
            pass

    def on_cmd(msg):
        d = msg.drive
        write("cmd", s=stamp(msg), steer=d.steering_angle, v=d.speed, a=d.acceleration)

    sub = node.create_subscription
    sub(Odometry, PREFIX + "/odom", on_truth, qos_profile_sensor_data)
    sub(Odometry, PREFIX + "/wheel_odom", on_wheel, 10)
    sub(LaserScan, PREFIX + "/scan", lambda m: write("scan", s=stamp(m)), qos_profile_sensor_data)
    sub(String, "/avlite/controller_diagnostics", on_diag, 10)
    sub(AckermannDriveStamped, "/avlite/control_command", on_cmd, 10)
    sub(Int32, PREFIX + "/lap_count", lambda m: write("lap", n=m.data), 10)
    sub(Int32, PREFIX + "/collision_count", lambda m: write("collision", n=m.data), 10)
    sub(String, "/avlite/race_plan", lambda m: write("plan", d=json.loads(m.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    reset_pub = node.create_publisher(Bool, "/autodrive/reset_command", 1)

    def maybe_reset():
        t = now()
        if state["release_at"] is not None and t >= state["release_at"]:
            reset_pub.publish(Bool(data=False))  # the bridge re-spawns every frame while true
            state["release_at"] = None
        if (not args.auto_reset or state["release_at"] is not None or state["still_since"] is None
                or t - state["still_since"] < args.stuck_s or t - state["last_reset"] < 8.0):
            return
        x, y, yaw, _ = state["truth"]
        state["resets"] += 1
        write("reset", x=x, y=y, yaw=yaw, reason="stuck", count=state["resets"])
        reset_pub.publish(Bool(data=True))
        state["release_at"] = t + 1.0
        state["last_reset"] = t
        state["still_since"] = None

    end = now() + args.seconds
    write("meta", start_mem_mb=free_memory_mb(), argv=sys.argv[1:])
    frozen = False
    print(f"recording {args.seconds:.0f} s to {args.output}", flush=True)
    try:
        while now() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            maybe_reset()
            if (args.require_motion_s and state["first_truth"] and state["peak_v"] < 0.5
                    and now() - state["first_truth"] > args.require_motion_s):
                frozen = True
                break
    finally:
        write("meta", end_mem_mb=free_memory_mb(), peak_v=state["peak_v"], frozen=frozen)
        if state["release_at"] is not None:
            reset_pub.publish(Bool(data=False))
        out.close()
        node.destroy_node()
        rclpy.shutdown()
    if frozen:
        print(f"ABORT: car never passed 0.5 m/s in {args.require_motion_s:.0f} s (frozen simulator "
              "car or AVLite not driving); restart the simulator and the stack", flush=True)
        return 3
    print(f"done; {state['resets']} resets", flush=True)


if __name__ == "__main__":
    sys.exit(main())
