"""Wheel odometry for the simulator that does not use ground truth.

AutoDRIVE publishes the true pose (``/autodrive/roboracer_1/odom`` and TF
``world -> roboracer_1``). SLAM localization must not see it, so this node builds the same
inputs the real car provides:

- ``nav_msgs/Odometry`` on ``/autodrive/roboracer_1/wheel_odom`` from the rear wheel encoders
  (speed) and the IMU gyro (yaw rate), integrated from the start pose as ``odom -> base_link``;
- TF ``odom -> base_link`` and a static ``base_link -> laser`` at the simulator's LiDAR mount;
- the LiDAR scan republished on ``/autodrive/roboracer_1/scan`` in the ``laser`` frame.

These car-style frames form a tree separate from the simulator's ``world`` tree, so a frame
never has two parents. The node reads the true pose once, only to log where in the world its
odometry started: that is ``localization.map_origin_world`` when this run is used for mapping. Optional speed-scale and gyro-bias errors make the sim's ideal sensors
drift like real ones.

    python3 -m avlite_autodrive.sim_odometry --wheel-radius 0.059
"""

import argparse
import math

from avlite_autodrive.ros_utils import PREFIX, odom_state

ODOM_TOPIC = PREFIX + "/wheel_odom"
SCAN_TOPIC = PREFIX + "/scan"
LIDAR_MOUNT = (0.2733, 0.0, 0.096)  # Published by AutoDRIVE: rear axle -> LiDAR.


def angle_delta(current, previous):
    """Return the smallest signed change between two wheel angles in radians."""
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class WheelOdometry:
    """Integrate rear-wheel encoder angles and gyro yaw rate into a planar pose.

    The pose starts at (0, 0, 0) in the odom frame. Speed is the mean rear-wheel rotation
    rate times the wheel radius; heading comes from the gyro, not the wheel difference,
    because the rear wheels slip in corners.
    """

    def __init__(self, wheel_radius, speed_scale=1.0, yaw_rate_bias=0.0, wrapped=False):
        if not (math.isfinite(wheel_radius) and wheel_radius > 0):
            raise ValueError("wheel radius must be positive")
        self.wheel_radius = wheel_radius
        self.speed_scale = speed_scale
        self.yaw_rate_bias = yaw_rate_bias
        self.wrapped = wrapped  # True if encoders report angles in [0, 2*pi) or (-pi, pi]
        self.x = self.y = self.yaw = 0.0
        self.speed = self.yaw_rate = 0.0
        self.angles = None
        self.time = None

    def update(self, left, right, yaw_rate, t):
        """Advance to time t (s) with wheel angles (rad) and yaw rate (rad/s).

        Return True when the pose advanced. The first sample, a repeated or backwards
        time, or a nonfinite value only resets the reference without moving.
        """
        values = (left, right, yaw_rate, t)
        if not all(math.isfinite(v) for v in values):
            self.angles = self.time = None
            return False
        if self.angles is None or t <= self.time:
            self.angles, self.time = (left, right), t
            return False
        dt = t - self.time
        delta = angle_delta if self.wrapped else (lambda a, b: a - b)
        rotation = (delta(left, self.angles[0]) + delta(right, self.angles[1])) / 2
        distance = rotation * self.wheel_radius * self.speed_scale
        self.yaw_rate = yaw_rate + self.yaw_rate_bias
        # Midpoint heading keeps a constant turn on its arc better than Euler steps.
        mid = self.yaw + 0.5 * self.yaw_rate * dt
        self.x += distance * math.cos(mid)
        self.y += distance * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + self.yaw_rate * dt),
                              math.cos(self.yaw + self.yaw_rate * dt))
        self.speed = distance / dt
        self.angles, self.time = (left, right), t
        return True


def main(argv=None):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState, LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wheel-radius", type=float, default=0.059,
                        help="Rear wheel radius in metres; calibrate against ground truth")
    parser.add_argument("--speed-scale", type=float, default=1.0,
                        help="Multiply encoder distance, e.g. 1.02 for a 2%% error")
    parser.add_argument("--yaw-rate-bias", type=float, default=0.0,
                        help="Add to the gyro yaw rate in rad/s, e.g. 0.004")
    parser.add_argument("--wrapped-encoders", action="store_true",
                        help="Encoder angles wrap each turn (default: cumulative radians)")
    options, ros_args = parser.parse_known_args(argv)
    odometry = WheelOdometry(options.wheel_radius, options.speed_scale, options.yaw_rate_bias,
                             options.wrapped_encoders)
    rclpy.init(args=ros_args)
    node = Node("avlite_sim_odometry")
    odom_pub = node.create_publisher(Odometry, ODOM_TOPIC, 10)
    scan_pub = node.create_publisher(LaserScan, SCAN_TOPIC, qos_profile_sensor_data)
    tf_pub = TransformBroadcaster(node)
    static_pub = StaticTransformBroadcaster(node)
    latest = {"left": None, "right": None, "yaw_rate": None}
    origin = {"world": None, "logged": False}

    mount = TransformStamped()
    mount.header.stamp = node.get_clock().now().to_msg()
    mount.header.frame_id, mount.child_frame_id = "base_link", "laser"
    (mount.transform.translation.x, mount.transform.translation.y,
     mount.transform.translation.z) = LIDAR_MOUNT
    mount.transform.rotation.w = 1.0
    static_pub.sendTransform(mount)

    def on_encoder(side, msg):
        if msg.position:
            latest[side] = float(msg.position[0])
        if side == "right" and None not in latest.values():
            stamp = msg.header.stamp
            if odometry.update(latest["left"], latest["right"], latest["yaw_rate"],
                               stamp.sec + stamp.nanosec * 1e-9):
                if not origin["logged"] and origin["world"] is not None:
                    origin["logged"] = True
                    x, y, yaw = origin["world"]
                    node.get_logger().info(f"Odometry origin in world (map_origin_world for "
                                           f"a mapping run): [{x:.4f}, {y:.4f}, {yaw:.5f}]")
                publish(stamp)

    def on_truth(msg):
        if not origin["logged"] and odometry.time is None:
            try:
                origin["world"] = odom_state(msg)[:3]
            except ValueError:
                origin["world"] = None

    def on_imu(msg):
        latest["yaw_rate"] = float(msg.angular_velocity.z)

    def on_scan(msg):
        msg.header.frame_id = "laser"
        scan_pub.publish(msg)

    def publish(stamp):
        q = (0.0, 0.0, math.sin(odometry.yaw / 2), math.cos(odometry.yaw / 2))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = odometry.x, odometry.y
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        odom.twist.twist.linear.x = odometry.speed
        odom.twist.twist.angular.z = odometry.yaw_rate
        odom_pub.publish(odom)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x, t.transform.translation.y = odometry.x, odometry.y
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        tf_pub.sendTransform(t)

    node.create_subscription(JointState, PREFIX + "/left_encoder",
                             lambda m: on_encoder("left", m), qos_profile_sensor_data)
    node.create_subscription(JointState, PREFIX + "/right_encoder",
                             lambda m: on_encoder("right", m), qos_profile_sensor_data)
    node.create_subscription(Imu, PREFIX + "/imu", on_imu, qos_profile_sensor_data)
    node.create_subscription(Odometry, PREFIX + "/odom", on_truth, qos_profile_sensor_data)
    node.create_subscription(LaserScan, PREFIX + "/lidar", on_scan, qos_profile_sensor_data)
    node.get_logger().info(f"Wheel odometry on {ODOM_TOPIC}; scans relayed to {SCAN_TOPIC} "
                           f"(radius {options.wheel_radius} m, scale {options.speed_scale}, "
                           f"gyro bias {options.yaw_rate_bias} rad/s)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
