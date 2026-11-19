"""Respawn the car N times while AVLite keeps running; record the localization error afterwards.

Run in the avlite container (stack driving, Autonomous selected):

    python3 reset_test.py OUT.json --resets 10 --window 8

Each reset holds the simulator's reset flag for 1.5 s (the car is teleported to the spawn
point), releases it, then samples /avlite/controller_diagnostics for --window seconds. The
car is left driving between resets, so this is the real "crash, respawn, carry on" case.
Per reset it reports the error 1, 2, 3, 5 and 8 s after release, the worst error in the window,
and whether the plan stayed valid.
"""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

CHECKPOINTS = (1.0, 2.0, 3.0, 5.0, 8.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--resets", type=int, default=10)
    parser.add_argument("--window", type=float, default=8.0)
    parser.add_argument("--settle", type=float, default=6.0, help="seconds of driving between resets")
    args = parser.parse_args()

    rclpy.init()
    node = Node("reset_test")
    pub = node.create_publisher(Bool, "/autodrive/reset_command", 1)
    latest = {}
    node.create_subscription(String, "/avlite/controller_diagnostics",
                             lambda m: latest.update(d=json.loads(m.data), t=time.monotonic()), 10)

    def spin(seconds, hold=None, samples=None, t0=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if hold is not None:
                pub.publish(Bool(data=hold))
            rclpy.spin_once(node, timeout_sec=0.05)
            if samples is not None and "d" in latest and (not samples or latest["t"] != samples[-1][3]):
                d = latest["d"]
                samples.append((time.monotonic() - t0, d.get("localization_error_m"),
                                d.get("plan_valid"), latest["t"], d.get("localization_alignment")))

    spin(2.0)
    results = []
    for n in range(1, args.resets + 1):
        spin(args.settle)                       # drive for a while (or sit crashed)
        spin(1.5, hold=True)                    # teleport to the spawn point
        t0 = time.monotonic()
        samples = []
        pub.publish(Bool(data=False))
        spin(args.window, hold=False, samples=samples, t0=t0)
        row = {"reset": n, "samples": [(round(s[0], 2), s[1], s[2], s[4]) for s in samples]}
        at = {}
        for c in CHECKPOINTS:
            near = [s for s in samples if abs(s[0] - c) < 0.35 and s[1] is not None]
            at[c] = round(near[0][1], 2) if near else None
        row["error_at_s"] = at
        errs = [s[1] for s in samples if s[1] is not None]
        row["worst_m"] = round(max(errs), 2) if errs else None
        row["plan_invalid_s"] = round(sum(1 for s in samples if s[2] == 0) * 0.05, 1)
        results.append(row)
        print(f"reset {n}: error at {at}  worst {row['worst_m']} m  plan invalid {row['plan_invalid_s']} s",
              flush=True)
    json.dump(results, open(args.output, "w"), indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
