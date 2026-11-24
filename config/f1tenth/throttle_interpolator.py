# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Modified for this car (Nov 2026): optional max_deceleration, so the car can speed up slowly but
# still stop quickly. Without max_deceleration the behaviour is the stock one (one limit both ways).

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

def ramp(last, desired, max_up, max_down):
    """One smoother step. Speeding up (further from zero) moves at most max_up, slowing down or
    reversing (towards zero) at most max_down. A reversal passes through zero before speeding up."""
    if desired == last:
        return last
    same_side = last == 0 or desired == 0 or (desired > 0) == (last > 0)
    if not same_side:
        desired = 0.0  # reversing: slow to a stop first, then speed up from zero
    speeding_up = same_side and abs(desired) > abs(last)
    limit = max_up if speeding_up else max_down
    return last + max(min(desired - last, limit), -limit)


class ThrottleInterpolator(Node):
    def __init__(self):
        super().__init__('throttle_interpolator')

        self.declare_parameter('rpm_input_topic', 'commands/motor/unsmoothed_speed')
        self.declare_parameter('rpm_output_topic', 'commands/motor/speed')
        self.declare_parameter('servo_input_topic', 'commands/servo/unsmoothed_position')
        self.declare_parameter('servo_output_topic', 'commands/servo/position')
        self.declare_parameter('max_acceleration')
        self.declare_parameter('max_deceleration', -1.0)  # <= 0: same as max_acceleration
        self.declare_parameter('speed_max')
        self.declare_parameter('speed_min')
        self.declare_parameter('throttle_smoother_rate')
        self.declare_parameter('speed_to_erpm_gain')
        self.declare_parameter('max_servo_speed')
        self.declare_parameter('steering_angle_to_servo_gain')
        self.declare_parameter('servo_smoother_rate')
        self.declare_parameter('servo_max')
        self.declare_parameter('servo_min')
        self.declare_parameter('steering_angle_to_servo_offset')

        self.rpm_input_topic = self.get_parameter('rpm_input_topic').value
        self.rpm_output_topic = self.get_parameter('rpm_output_topic').value
        self.servo_input_topic = self.get_parameter('servo_input_topic').value
        self.servo_output_topic = self.get_parameter('servo_output_topic').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_deceleration = self.get_parameter('max_deceleration').value
        if self.max_deceleration <= 0:
            self.max_deceleration = self.max_acceleration
        self.max_rpm = self.get_parameter('speed_max').value
        self.min_rpm = self.get_parameter('speed_min').value
        self.throttle_smoother_rate = self.get_parameter('throttle_smoother_rate').value
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value
        self.max_servo_speed = self.get_parameter('max_servo_speed').value
        self.steering_angle_to_servo_gain = self.get_parameter('steering_angle_to_servo_gain').value
        self.servo_smoother_rate = self.get_parameter('servo_smoother_rate').value
        self.max_servo = self.get_parameter('servo_max').value
        self.min_servo = self.get_parameter('servo_min').value
        self.last_servo = self.get_parameter('steering_angle_to_servo_offset').value

        self.last_rpm = 0
        self.desired_rpm = self.last_rpm
        self.desired_servo_position = self.last_servo

        self.rpm_output = self.create_publisher(Float64, self.rpm_output_topic, 1)
        self.servo_output = self.create_publisher(Float64, self.servo_output_topic, 1)

        self.rpm_sub = self.create_subscription(
            Float64,
            self.rpm_input_topic,
            self._process_throttle_command,
            1)
        self.servo_sub = self.create_subscription(
            Float64,
            self.servo_input_topic,
            self._process_servo_command,
            1)

        self.max_delta_servo = abs(self.steering_angle_to_servo_gain * self.max_servo_speed / self.servo_smoother_rate)
        self.servo_timer = self.create_timer(1.0/self.servo_smoother_rate, self._publish_servo_command)

        self.max_delta_rpm = abs(self.speed_to_erpm_gain * self.max_acceleration / self.throttle_smoother_rate)
        self.max_delta_rpm_brake = abs(self.speed_to_erpm_gain * self.max_deceleration / self.throttle_smoother_rate)
        self.rmp_timer = self.create_timer(1.0/self.throttle_smoother_rate, self._publish_throttle_command)

    def _publish_throttle_command(self):
        smoothed_rpm = ramp(self.last_rpm, self.desired_rpm, self.max_delta_rpm, self.max_delta_rpm_brake)
        self.last_rpm = smoothed_rpm
        rpm_msg = Float64()
        rpm_msg.data = float(smoothed_rpm)
        self.rpm_output.publish(rpm_msg)
            
    def _process_throttle_command(self,msg):
        input_rpm = msg.data
        # Do some sanity clipping
        input_rpm = min(max(input_rpm, self.min_rpm), self.max_rpm)
        self.desired_rpm = input_rpm

    def _publish_servo_command(self):
        desired_delta = self.desired_servo_position-self.last_servo
        clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
        smoothed_servo = self.last_servo + clipped_delta
        self.last_servo = smoothed_servo
        servo_msg = Float64()
        servo_msg.data = float(smoothed_servo)
        self.servo_output.publish(servo_msg)

    def _process_servo_command(self,msg):
        input_servo = msg.data
        # Do some sanity clipping
        input_servo = min(max(input_servo, self.min_servo), self.max_servo)
        # set the target servo position
        self.desired_servo_position = input_servo

def main(args=None):
    rclpy.init(args=args)
    p = ThrottleInterpolator()
    rclpy.spin(p)

if __name__ == '__main__':
    main()
