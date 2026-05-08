"""
Unit tests for racer_node.py — run inside the devkit container:

    cd /home/autodrive_devkit
    colcon build --packages-select my_team_racer
    source install/setup.bash
    pytest src/my_packages/my_team_racer/test/ -v
"""

import math
import sys
import types
import unittest
import numpy as np

# ---------------------------------------------------------------------------
# Minimal ROS 2 stubs so the module can be imported without a live ROS context
# ---------------------------------------------------------------------------
def _make_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.shutdown = lambda: None

    node_mod = types.ModuleType("rclpy.node")
    class Node:
        def __init__(self, name): pass
        def create_subscription(self, *a, **kw): return None
        def create_publisher(self, *a, **kw): return _Publisher()
        def get_logger(self): return _Logger()
        def destroy_node(self): pass
    node_mod.Node = Node
    rclpy.node = node_mod

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    class LaserScan: pass
    sensor_msgs_msg.LaserScan = LaserScan
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    class Float32:
        def __init__(self, data=0.0): self.data = data
    std_msgs_msg.Float32 = Float32
    std_msgs.msg = std_msgs_msg

    for name, mod in [
        ("rclpy", rclpy),
        ("rclpy.node", node_mod),
        ("sensor_msgs", sensor_msgs),
        ("sensor_msgs.msg", sensor_msgs_msg),
        ("std_msgs", std_msgs),
        ("std_msgs.msg", std_msgs_msg),
    ]:
        sys.modules.setdefault(name, mod)


class _Publisher:
    def __init__(self): self.last = None
    def publish(self, msg): self.last = msg

class _Logger:
    def info(self, *a): pass
    def warning(self, *a): pass
    def error(self, *a): pass


# Install stubs before importing the node
_make_ros_stubs()

from my_team_racer.racer_node import RacerNode, MAX_THROTTLE, MIN_THROTTLE  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _make_scan(n=540, fill=2.0, angle_min=-2.356, angle_inc=0.00872):
    """Build a fake LaserScan with uniform range `fill`."""
    from sensor_msgs.msg import LaserScan
    msg = LaserScan()
    msg.angle_min = angle_min
    msg.angle_increment = angle_inc
    msg.ranges = [fill] * n
    return msg


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
class TestRacerNodeSteering(unittest.TestCase):

    def setUp(self):
        self.node = RacerNode()
        # Patch publishers to capture output
        self.pub_steer = _Publisher()
        self.pub_throttle = _Publisher()
        self.node.pub_steering = self.pub_steer
        self.node.pub_throttle = self.pub_throttle

    # -- Straight track (symmetric walls) → near-zero steering ---------------
    def test_straight_track_zero_steer(self):
        msg = _make_scan(fill=1.5)
        self.node._lidar_cb(msg)
        self.assertAlmostEqual(self.pub_steer.last.data, 0.0, places=2)

    # -- Symmetric scan → max throttle ----------------------------------------
    def test_straight_track_max_throttle(self):
        msg = _make_scan(fill=1.5)
        self.node._lidar_cb(msg)
        self.assertAlmostEqual(self.pub_throttle.last.data, MAX_THROTTLE, places=2)

    # -- All-inf scan (open space) → still publishes, clipped ----------------
    def test_all_inf_scan(self):
        msg = _make_scan(fill=float("inf"))
        self.node._lidar_cb(msg)
        self.assertIsNotNone(self.pub_steer.last)
        self.assertIsNotNone(self.pub_throttle.last)
        self.assertTrue(-1.0 <= self.pub_steer.last.data <= 1.0)

    # -- Steering always in [-1, 1] -------------------------------------------
    def test_steering_clamped(self):
        for fill in [0.2, 1.0, 4.0, float("inf")]:
            msg = _make_scan(fill=fill)
            self.node._lidar_cb(msg)
            s = self.pub_steer.last.data
            self.assertGreaterEqual(s, -1.0)
            self.assertLessEqual(s, 1.0)

    # -- Throttle always in [MIN, MAX] ----------------------------------------
    def test_throttle_bounds(self):
        for fill in [0.2, 1.0, 4.0]:
            msg = _make_scan(fill=fill)
            self.node._lidar_cb(msg)
            t = self.pub_throttle.last.data
            self.assertGreaterEqual(t, MIN_THROTTLE)
            self.assertLessEqual(t, MAX_THROTTLE)

    # -- Hard steer → throttle drops below max --------------------------------
    def test_throttle_lower_in_corner(self):
        msg_straight = _make_scan(fill=1.5)
        self.node._lidar_cb(msg_straight)
        t_straight = self.pub_throttle.last.data

        # Skew the scan: left wall much closer than right
        n = 540
        ranges = [1.5] * n
        for i in range(n // 2, n):        # right half of scan → close wall
            ranges[i] = 0.3
        msg_corner = _make_scan()
        msg_corner.ranges = ranges
        self.node._lidar_cb(msg_corner)
        t_corner = self.pub_throttle.last.data

        self.assertLess(t_corner, t_straight)


class TestPurePursuitMath(unittest.TestCase):
    """White-box tests of internal steering math."""

    def setUp(self):
        self.node = RacerNode()

    def _steer(self, ranges, angle_min=-2.356, angle_inc=0.00872):
        return self.node._pure_pursuit_steer(
            np.array(ranges, dtype=np.float32), angle_min, angle_inc
        )

    def test_symmetric_gives_zero(self):
        s = self._steer([1.5] * 540)
        self.assertAlmostEqual(s, 0.0, places=2)

    def test_right_wall_closer_steers_left(self):
        n = 540
        ranges = [1.5] * n
        angle_min, angle_inc = -2.356, 0.00872
        # Set right-side beams (negative y) to 0.4 m
        angles = [angle_min + i * angle_inc for i in range(n)]
        for i, a in enumerate(angles):
            if math.cos(a) > 0 and math.sin(a) < 0:
                ranges[i] = 0.4
        s = self._steer(ranges)
        self.assertGreater(s, 0.0, "Should steer left (positive) when right wall is close")

    def test_left_wall_closer_steers_right(self):
        n = 540
        ranges = [1.5] * n
        angle_min, angle_inc = -2.356, 0.00872
        angles = [angle_min + i * angle_inc for i in range(n)]
        for i, a in enumerate(angles):
            if math.cos(a) > 0 and math.sin(a) > 0:
                ranges[i] = 0.4
        s = self._steer(ranges)
        self.assertLess(s, 0.0, "Should steer right (negative) when left wall is close")


if __name__ == "__main__":
    unittest.main()
