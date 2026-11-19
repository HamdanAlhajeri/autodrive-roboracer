"""Respawn the simulator car: hold the reset flag true for 1.5 s, then false (run in the avlite
container). Prints the car's position afterwards so a failed respawn is visible."""
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

rclpy.init()
node = Node("respawn")
pub = node.create_publisher(Bool, "/autodrive/reset_command", 1)
last = {}
node.create_subscription(Odometry, "/autodrive/roboracer_1/odom", lambda m: last.update(
    x=m.pose.pose.position.x, y=m.pose.pose.position.y), qos_profile_sensor_data)
for value, seconds in ((True, 1.5), (False, 1.0)):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pub.publish(Bool(data=value))
        rclpy.spin_once(node, timeout_sec=0.05)
print("respawned at x=%.2f y=%.2f" % (last.get("x", float("nan")), last.get("y", float("nan"))), flush=True)
node.destroy_node()
rclpy.shutdown()
