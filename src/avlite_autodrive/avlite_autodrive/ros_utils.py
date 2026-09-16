"""ROS message validation shared by the two independent processes."""

import math


PREFIX = "/autodrive/roboracer_1"


def valid_scan(msg):
    return (
        len(msg.ranges) >= 32
        and math.isfinite(msg.angle_min)
        and math.isfinite(msg.angle_increment)
        and msg.angle_increment > 0
        and math.isfinite(msg.range_max)
        and msg.range_max > msg.range_min >= 0
        and sum(math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges)
        >= len(msg.ranges) * 0.5
    )


def odom_state(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v = msg.twist.twist.linear
    values = (p.x, p.y, q.x, q.y, q.z, q.w, v.x, v.y)
    if not all(math.isfinite(x) for x in values):
        raise ValueError("nonfinite odometry")
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not 0.9 < norm < 1.1:
        raise ValueError("invalid orientation quaternion")
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    # AutoDRIVE's IMU sends body-frame velocity; do not rotate it a second time.
    return p.x, p.y, yaw, v.x
