"""Independent hardware actuator adapter (ROS 2).

Receives vehicle-convention commands from the supervisor on /roboracer/command, applies
its own expiry watchdog and the measured steering/speed conversion, and publishes
AckermannDriveStamped to the installed driver. It stays silent until the supervisor first
commands motion, so the driver's own teleop can use the topic during commissioning. After
an expiry or stop it publishes stop until the car is stationary and the hold time passes.

A fresh nonzero manual command on topics.manual_input overrides autonomy: it is forwarded
and reported, and the supervisor stops its session. SSH or process loss is not a physical
stop; the car's independent emergency stop/manual override must remain available.
"""

import argparse
import json
import math
import signal
import time

from .command import CommandSource, DriveCommand, ExpiryWatchdog, to_hardware
from .profile import HardwareProfile

COMMAND_TOPIC = "/roboracer/command"
STATUS_TOPIC = "/roboracer/actuator_status"


class ActuatorLogic:
    """ROS-free decision logic, exercised by tests and wrapped by the node."""

    def __init__(self, profile, stop_hold_s=2.0):
        self.profile = profile
        self.watchdog = ExpiryWatchdog(profile.get("timing.command_timeout_s"))
        self.stop_hold_s = stop_hold_s
        self.state = "idle"          # idle -> active -> stopping -> idle
        self.stop_until = -math.inf
        self.manual = None
        self.manual_time = -math.inf
        self.speed = 0.0
        self.fault = None
        self.commissioned = not profile.missing_for_autonomy()
        if not self.commissioned:
            self.fault = "hardware profile not commissioned; actuator stays silent"

    def receive_command(self, command, stamp_age_s, now):
        """Accept a supervisor command if its header stamp is fresh (not delayed DDS)."""
        if not self.commissioned:
            return
        if not -0.1 <= stamp_age_s <= self.watchdog.timeout_s:
            return
        self.watchdog.feed(command, now)
        if not command.is_stop:
            self.state = "active"

    def receive_manual(self, steering, speed, now):
        if all(math.isfinite(v) for v in (steering, speed)):
            self.manual = DriveCommand(steering, speed, CommandSource.MANUAL)
            self.manual_time = now

    @property
    def manual_override(self):
        return self.manual is not None and not self.manual.is_stop

    def tick(self, now, competing_publishers=0):
        """Return the Ackermann (steering, speed) to publish, or None to stay silent."""
        if not self.commissioned:
            return None  # Without measured conversion, never publish to the driver.
        if now - self.manual_time > self.watchdog.timeout_s:
            self.manual = None
        if self.manual_override:
            return to_hardware(self.manual, self.profile)
        if competing_publishers:
            self.fault = "another node publishes the driver command topic"
        command = self.watchdog.output(now)
        if self.state == "active" and (self.watchdog.expired or command.is_stop
                                       or self.fault):
            self.state = "stopping"
            self.stop_until = now + self.stop_hold_s
        if self.state == "stopping":
            if now >= self.stop_until and abs(self.speed) < 0.05:
                self.state = "idle"
                self.watchdog.reset()
            if not self.fault and not self.watchdog.expired and not command.is_stop:
                self.state = "active"
            else:
                return (self.profile.get("steering.center_offset_rad", 0.0),
                        float(self.profile.get("motor.stop_command_speed_mps", 0.0)))
        if self.state == "active":
            return to_hardware(command, self.profile)
        return None

    def status(self, now):
        return {"state": self.state, "fault": self.fault,
                "manual_override": self.manual_override,
                "command_expired": self.watchdog.expired,
                "measured_speed_mps": self.speed if math.isfinite(self.speed) else None}


def main(argv=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    options, ros_args = parser.parse_known_args(argv)
    profile = HardwareProfile.load(options.profile)
    logic = ActuatorLogic(profile)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("roboracer_actuator")
    output_topic = profile.topic("drive_output")
    publisher = node.create_publisher(AckermannDriveStamped, output_topic, 1)
    status = node.create_publisher(String, STATUS_TOPIC, 1)

    def on_command(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        try:
            source = CommandSource(msg.header.frame_id)
        except ValueError:
            return
        if not all(math.isfinite(v) for v in (msg.drive.steering_angle, msg.drive.speed)):
            return
        logic.receive_command(DriveCommand(msg.drive.steering_angle,
                                           max(msg.drive.speed, 0.0), source),
                              age, time.monotonic())

    def on_odom(msg):
        speed = msg.twist.twist.linear.x
        logic.speed = speed if math.isfinite(speed) else math.inf

    node.create_subscription(AckermannDriveStamped, COMMAND_TOPIC, on_command, 1)
    node.create_subscription(Odometry, profile.topic("odom"), on_odom, qos_profile_sensor_data)
    if profile.topic("manual_input"):
        node.create_subscription(
            AckermannDriveStamped, profile.topic("manual_input"),
            lambda m: logic.receive_manual(m.drive.steering_angle, m.drive.speed,
                                           time.monotonic()), 1)

    def publish(values):
        msg = AckermannDriveStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = profile.frame("base_link")
        msg.drive.steering_angle, msg.drive.speed = values
        publisher.publish(msg)

    def tick():
        now = time.monotonic()
        competing = 0
        if logic.state == "active":
            competing = (max(0, node.count_publishers(output_topic) - 1)
                         + max(0, node.count_publishers(COMMAND_TOPIC) - 1))
        values = logic.tick(now, competing)
        if values is not None:
            publish(values)
        status.publish(String(data=json.dumps(logic.status(now), allow_nan=False)))

    node.create_timer(0.02, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node.get_logger().info(f"Actuator ready; driver topic {output_topic}; "
                           f"fault: {logic.fault or 'none'}")
    try:
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok() and logic.state != "idle":
            for _ in range(5):
                publish((0.0, float(profile.get("motor.stop_command_speed_mps", 0.0))))
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
