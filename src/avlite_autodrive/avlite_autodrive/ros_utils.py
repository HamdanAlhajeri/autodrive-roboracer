"""ROS message validation shared by the two independent processes."""

import math


PREFIX = "/autodrive/roboracer_1"


def pose_discontinuous(previous, current, timeout=0.5):
    """Compare (x, y, speed, monotonic receive time) without a fixed speed cap.

    Keep the original one-metre tolerance at low speed. At high speed permit
    the distance travelled during a fresh update plus 0.25 m of pose error.
    A sensor interruption always breaks continuity, even at the same position.
    """
    if previous is None:
        return False
    dt = current[3] - previous[3]
    if not math.isfinite(dt) or dt < 0 or dt > timeout:
        return True
    reachable = max(abs(previous[2]), abs(current[2])) * dt
    return math.hypot(current[0] - previous[0], current[1] - previous[1]) > max(
        1.0, reachable + 0.25)


def valid_scan(msg):
    """Check scan dimensions, range limits and whether enough beams contain usable returns.

    At least half the beams must be finite and within range. This check decides whether the
    scan can refresh the sensor watchdog.
    """
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
    """Extract world x/y, heading in radians and body-frame forward speed in m/s.

    Validate and normalize the orientation quaternion before finding yaw. Raise ValueError
    for invalid data so each caller can invalidate its sensor history.
    """
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
