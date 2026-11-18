"""Log how old each transform is, 10 times a second: t_s,odom_to_base_age_s,map_to_base_age_s.

Negative ages mean a transform is stamped ahead of this node's clock.
    python tf_age.py OUTPUT.csv
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main(path):
    rclpy.init()
    node = Node("tf_age_logger")
    buffer = Buffer()
    TransformListener(buffer, node)
    out = open(path, "w", buffering=1)
    out.write("t_s,odom_to_base_age_s,map_to_base_age_s\n")

    def age(target):
        try:
            t = buffer.lookup_transform(target, "base_link", Time())
        except Exception:
            return ""
        return f"{(node.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9:.3f}"

    def tick():
        out.write(f"{node.get_clock().now().nanoseconds * 1e-9:.2f},{age('odom')},{age('map')}\n")

    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main(sys.argv[1])
