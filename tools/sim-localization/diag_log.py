"""Log /avlite/controller_diagnostics (error, alignment, plan validity) for N seconds as JSON lines.

Run in the avlite container: python3 diag_log.py OUT.jsonl SECONDS. ``t`` is the seconds since start.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

out, seconds = open(sys.argv[1], "w"), float(sys.argv[2])
rclpy.init()
node = Node("diag_log")
t0 = time.monotonic()


def on_diag(msg):
    d = json.loads(msg.data)
    out.write(json.dumps({"t": round(time.monotonic() - t0, 3), "err": d.get("localization_error_m"),
                          "align": d.get("localization_alignment"), "plan_valid": d.get("plan_valid"),
                          "v": d.get("target_velocity_mps"), "tx": d.get("truth_x_m"),
                          "ty": d.get("truth_y_m")}) + "\n")


node.create_subscription(String, "/avlite/controller_diagnostics", on_diag, 10)
while time.monotonic() - t0 < seconds:
    rclpy.spin_once(node, timeout_sec=0.05)
out.close()
