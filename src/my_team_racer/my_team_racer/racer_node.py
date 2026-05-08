#!/usr/bin/env python3
"""
Pure Pursuit racer node for AutoDRIVE RoboRacer.

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
LOOKAHEAD_DIST   = 0.8    # metres ahead on the centerline to target
MAX_THROTTLE     = 0.6    # top throttle on straights  [0, 1]
MIN_THROTTLE     = 0.15   # minimum throttle in tight corners
STEER_GAIN       = 1.2    # proportional gain on the heading error
THROTTLE_DECAY   = 2.5    # how aggressively throttle drops with |steer|
WALL_CLIP_DIST   = 4.0    # metres — clip LiDAR readings beyond this
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

        self.get_logger().info("RacerNode started — pure pursuit via LiDAR")

    # ── LiDAR callback ────────────────────────────────────────────────────────

    def _lidar_cb(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges, dtype=np.float32)
        angle_min = msg.angle_min        # rad, e.g. -135 deg for 270° scan
        angle_inc = msg.angle_increment  # rad per step

        # Replace inf / NaN with wall-clip distance
        ranges = np.where(np.isfinite(ranges), ranges, WALL_CLIP_DIST)
        ranges = np.clip(ranges, 0.0, WALL_CLIP_DIST)

        steering = self._pure_pursuit_steer(ranges, angle_min, angle_inc)
        throttle = self._speed_from_steer(steering)

        self.pub_steering.publish(Float32(data=float(steering)))
        self.pub_throttle.publish(Float32(data=float(throttle)))

    # ── Pure pursuit steering ─────────────────────────────────────────────────

    def _pure_pursuit_steer(
        self, ranges: np.ndarray, angle_min: float, angle_inc: float
    ) -> float:
        """
        1. Convert polar LiDAR scan to cartesian points.
        2. Separate points roughly left and right of the car.
        3. Average left / right distances in the forward half to find
           a rough centerline offset.
        4. Compute heading correction to a lookahead point on that centerline.
        """
        n = len(ranges)
        angles = angle_min + np.arange(n) * angle_inc  # rad

        # --- Cartesian scan (car frame: x forward, y left) ---
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        # Keep only points in the forward hemisphere
        fwd_mask = xs > 0.05

        # Left / right split on y-axis
        left_mask  = fwd_mask & (ys >= 0)
        right_mask = fwd_mask & (ys <  0)

        # Mean lateral distance to left / right walls
        left_dist  = float(np.mean(ys[left_mask]))   if left_mask.any()  else  1.0
        right_dist = float(np.mean(-ys[right_mask]))  if right_mask.any() else  1.0

        # Centerline error: positive → car is too close to right wall
        total = left_dist + right_dist
        center_error = (left_dist - right_dist) / total if total > 0 else 0.0

        # Lookahead point (LOOKAHEAD_DIST ahead, offset by center_error)
        lx = LOOKAHEAD_DIST
        ly = center_error * LOOKAHEAD_DIST

        # Pure-pursuit curvature → heading error
        heading_err = math.atan2(ly, lx)
        steering = float(np.clip(STEER_GAIN * heading_err, -1.0, 1.0))

        return steering

    # ── Throttle scheduling ───────────────────────────────────────────────────

    def _speed_from_steer(self, steering: float) -> float:
        """Scale speed inversely with |steering| — slow down in corners."""
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
