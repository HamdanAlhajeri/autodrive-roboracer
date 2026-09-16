"""Independent actuator publisher. AVLite never owns AutoDRIVE actuator topics."""

import argparse
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from .actuation import Actuation, Limits
from .configuration import load_config
from .ros_utils import PREFIX, odom_state, valid_scan


class ActuatorAdapter(Node):
    def __init__(self, parameters=None):
        super().__init__("avlite_actuator_adapter")
        defaults = Limits(**(parameters or {}))
        params = {k: self.declare_parameter(k, v).value for k, v in vars(defaults).items()}
        self.control = Actuation(Limits(**params))
        self.get_logger().info(
            f"Driving limits: max_speed={self.control.limits.max_speed:.3f} m/s, "
            f"max_throttle={self.control.limits.max_throttle:.3f}"
        )
        self.throttle = self.create_publisher(Float32, PREFIX + "/throttle_command", 1)
        self.steering = self.create_publisher(Float32, PREFIX + "/steering_command", 1)
        self.diagnostics = self.create_publisher(String, "/avlite/actuator_diagnostics", 1)
        self.create_subscription(
            AckermannDriveStamped, "/avlite/control_command", self.on_command, 1
        )
        self.create_subscription(Odometry, PREFIX + "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, PREFIX + "/lidar", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "/autodrive/reset_command", self.on_reset, 1)
        self.previous_pose = None
        self.previous_time = time.monotonic()
        self.last_reason = None
        self.create_timer(0.05, self.tick)

    def on_command(self, msg):
        # Reject delayed/replayed commands even when DDS delivers them just now.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age < -0.1 or age > self.control.limits.timeout:
            self.control.reset()
            return
        self.control.receive_command(
            msg.drive.steering_angle, msg.drive.acceleration, time.monotonic()
        )

    def on_odom(self, msg):
        try:
            x, y, _, speed = odom_state(msg)
        except ValueError:
            self.control.reset()
            return
        if (
            self.previous_pose
            and math.hypot(x - self.previous_pose[0], y - self.previous_pose[1]) > 1.0
        ):
            self.control.reset()
        self.previous_pose = (x, y)
        self.control.receive_speed(speed, time.monotonic())

    def on_scan(self, msg):
        if valid_scan(msg):
            self.control.scan_time = time.monotonic()
        else:
            self.control.reset()

    def on_reset(self, msg):
        if msg.data:
            self.control.reset()
            self.previous_pose = None

    def publish(self, throttle, steering):
        self.throttle.publish(Float32(data=float(throttle)))
        self.steering.publish(Float32(data=float(steering)))

    def tick(self):
        now = time.monotonic()
        dt, self.previous_time = now - self.previous_time, now
        # Refuse competing publishers instead of fighting the legacy controller.
        if (
            self.count_publishers(PREFIX + "/throttle_command") > 1
            or self.count_publishers(PREFIX + "/steering_command") > 1
        ):
            self.control.reset()
            self.control.reason = "competing actuator publisher; stop the other controller"
            result = (0.0, 0.0)
        else:
            result = self.control.tick(now, dt)
        self.publish(*result)
        diagnostics = dict(self.control.diagnostics)
        diagnostics["actuator_loop_dt_s"] = dt if math.isfinite(dt) else None
        self.diagnostics.publish(String(data=json.dumps(diagnostics, allow_nan=False)))
        if self.control.reason != self.last_reason:
            self.get_logger().info(self.control.reason)
            self.last_reason = self.control.reason


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", help="Actuator YAML profile with optional shared_settings")
    options, ros_args = parser.parse_known_args(args)
    parameters = None
    if options.config:
        parameters = load_config(options.config)["avlite_actuator_adapter"]["ros__parameters"]
    # Keep DDS alive long enough to publish zero when Compose sends SIGTERM.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = ActuatorAdapter(parameters)
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.publish(0.0, 0.0)
                time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
