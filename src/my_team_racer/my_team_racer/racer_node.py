#!/usr/bin/env python3
"""
Racer node for AutoDRIVE RoboRacer.

Subscribes to LiDAR and publishes throttle + steering commands.
Topics:
  IN  /autodrive/roboracer_1/lidar          sensor_msgs/LaserScan
  OUT /autodrive/roboracer_1/throttle_command std_msgs/Float32  [-1, 1]
  OUT /autodrive/roboracer_1/steering_command std_msgs/Float32  [-1, 1]
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


# ── Tunable parameters ────────────────────────────────────────────────────────
LOOKAHEAD_DIST   = 0.8    # metres — lookahead distance L
WHEELBASE        = 0.32   # metres — RoboRacer wheelbase (approx)
MAX_STEER_RAD    = 0.4    # radians — max physical steering angle for normalisation
MAX_THROTTLE     = 0.4    # top speed on straights [0, 1]
MIN_THROTTLE     = 0.1    # minimum speed in corners
THROTTLE_DECAY   = 3.0    # corner braking aggressiveness
WALL_CLIP_DIST   = 4.0    # clip LiDAR beyond this distance (metres)
EMA_ALPHA        = 0.3    # smoothing factor for gy estimate (0=no update, 1=no filter)
# ─────────────────────────────────────────────────────────────────────────────


class RacerNode(Node):
    def __init__(self):
        super().__init__("racer_node")

        self.sub_lidar = self.create_subscription(
            LaserScan,
            "/autodrive/roboracer_1/lidar",
            self._lidar_cb,
            10,
        )

        self.pub_throttle = self.create_publisher(
            Float32, "/autodrive/roboracer_1/throttle_command", 10
        )
        self.pub_steering = self.create_publisher(
            Float32, "/autodrive/roboracer_1/steering_command", 10
        )

        self._gy_filtered = 0.0

        self.get_logger().info("RacerNode started — geometric pure pursuit")

    # ── LiDAR callback ────────────────────────────────────────────────────────

    def _lidar_cb(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges, dtype=np.float32)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        ranges = np.where(np.isfinite(ranges), ranges, WALL_CLIP_DIST)
        ranges = np.clip(ranges, 0.0, WALL_CLIP_DIST)

        angles = angle_min + np.arange(len(ranges)) * angle_inc

        gx = LOOKAHEAD_DIST
        gy = self._estimate_gy(ranges, angles)

        steering = self._pure_pursuit_steer(gx, gy)
        throttle = self._speed_from_steer(steering)

        self.pub_steering.publish(Float32(data=steering))
        self.pub_throttle.publish(Float32(data=throttle))

    # ── Centerline lateral offset estimate ───────────────────────────────────

    def _estimate_gy(self, ranges: np.ndarray, angles: np.ndarray) -> float:
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        fwd = xs > 0.05
        left_pts  = ys[fwd & (ys >= 0)]
        right_pts = ys[fwd & (ys <  0)]

        left_dist  = float(np.median(left_pts))   if len(left_pts)  > 0 else 1.0
        right_dist = float(np.median(-right_pts)) if len(right_pts) > 0 else 1.0

        # signed lateral offset to the centerline in metres (positive = left)
        gy_raw = (left_dist - right_dist) / 2.0

        self._gy_filtered = (1.0 - EMA_ALPHA) * self._gy_filtered + EMA_ALPHA * gy_raw
        return self._gy_filtered

    # ── Geometric pure pursuit steering ──────────────────────────────────────

    def _pure_pursuit_steer(self, gx: float, gy: float) -> float:
        L_sq = gx**2 + gy**2
        if abs(gy) < 1e-6:
            return 0.0
        curvature = (2.0 * abs(gy)) / L_sq
        delta = math.atan(curvature * WHEELBASE)
        steering = math.copysign(delta, gy)
        return float(np.clip(steering / MAX_STEER_RAD, -1.0, 1.0))

    # ── Throttle scheduling ───────────────────────────────────────────────────

    def _speed_from_steer(self, steering: float) -> float:
        throttle = MAX_THROTTLE * math.exp(-THROTTLE_DECAY * abs(steering))
        return float(np.clip(throttle, MIN_THROTTLE, MAX_THROTTLE))


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RacerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
