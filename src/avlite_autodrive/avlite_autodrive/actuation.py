"""Pure-Python command conversion, independently testable without ROS or AVLite."""

from dataclasses import dataclass
import math


def clamp(value, low, high):
    return min(high, max(low, value))


@dataclass
class Limits:
    max_speed: float = 0.5
    max_throttle: float = 0.02
    max_steer: float = math.radians(30)
    timeout: float = 0.5
    speed_kp: float = 0.01
    speed_ki: float = 0.002
    feedforward: float = 0.04

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError("Actuator limits/gains must be finite and positive")
        if self.max_throttle > 1 or self.max_steer >= math.pi / 2:
            raise ValueError("Invalid throttle or steering limit")


class Actuation:
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.reset()

    def reset(self):
        self.target_speed = 0.0
        self.integral = 0.0
        self.command = None
        self.command_time = self.odom_time = self.scan_time = -math.inf
        self.speed = 0.0
        self.reason = "waiting for sensors and command"

    def receive_command(self, steering, acceleration, now):
        if not all(math.isfinite(v) for v in (steering, acceleration)):
            self.reset()
            self.reason = "invalid command"
            return
        self.command = (steering, clamp(acceleration, -3.0, 2.0))
        self.command_time = now

    def receive_speed(self, speed, now):
        if not math.isfinite(speed):
            self.reset()
            self.reason = "invalid odometry"
            return
        self.speed = speed
        self.odom_time = now

    def tick(self, now, dt):
        ages = (now - self.command_time, now - self.odom_time, now - self.scan_time)
        if self.command is None or any(a < 0 or a > self.limits.timeout for a in ages):
            self.target_speed = self.integral = 0.0
            self.reason = "stale or missing sensors/command"
            return 0.0, 0.0
        if not math.isfinite(dt) or dt <= 0 or dt > self.limits.timeout:
            self.reset()
            self.reason = "control timing discontinuity"
            return 0.0, 0.0
        steer, acceleration = self.command
        # AVLite emits acceleration (m/s²), not throttle. Integrate a speed demand
        # and track that demand with measured speed, keeping the two units distinct.
        self.target_speed = clamp(self.target_speed + acceleration * dt, 0.0, self.limits.max_speed)
        error = self.target_speed - self.speed
        candidate = clamp(self.integral + error * dt, -1.0, 1.0)
        raw = (
            self.limits.feedforward * self.target_speed
            + self.limits.speed_kp * error
            + self.limits.speed_ki * candidate
        )
        # Conditional integration prevents windup at either actuator limit.
        if 0 <= raw <= self.limits.max_throttle:
            self.integral = candidate
        throttle = clamp(raw, 0.0, self.limits.max_throttle)
        if self.target_speed < 0.01 or self.speed > self.limits.max_speed * 1.1:
            throttle = 0.0
            self.integral = 0.0
        self.reason = "active"
        return throttle, clamp(steer / self.limits.max_steer, -1.0, 1.0)
