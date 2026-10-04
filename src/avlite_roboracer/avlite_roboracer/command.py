"""Command ownership, expiry and hardware conversion (no ROS).

The supervisor selects exactly one command source. Commands from any other source are
rejected, and the selected source's commands expire locally. The actuator process applies
its own, independent expiry before anything reaches the motor driver.
"""

from dataclasses import dataclass
from enum import Enum
import math


class CommandSource(str, Enum):
    NONE = "none"
    MANUAL = "manual"
    MAPPING = "mapping"
    RACING = "racing"


@dataclass(frozen=True)
class DriveCommand:
    """Steering angle in radians (positive = left) and forward speed in m/s."""

    steering_rad: float = 0.0
    speed_mps: float = 0.0
    source: CommandSource = CommandSource.NONE

    @property
    def is_stop(self):
        return self.speed_mps == 0.0


STOP = DriveCommand()


def _finite(*values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in values)


class CommandArbiter:
    """Pass commands from the single selected source and stop when they expire."""

    def __init__(self, timeout_s):
        if not _finite(timeout_s) or timeout_s <= 0:
            raise ValueError("command timeout must be positive")
        self.timeout_s = timeout_s
        self.owner = CommandSource.NONE
        self._command = None
        self._received_s = -math.inf
        self.rejected = 0

    def select(self, source):
        """Give command ownership to source and discard any earlier command."""
        self.owner = CommandSource(source)
        self._command = None
        self._received_s = -math.inf

    def release(self):
        """Remove every source's ownership; output becomes STOP immediately."""
        self.select(CommandSource.NONE)

    def submit(self, source, steering_rad, speed_mps, now):
        """Accept a command only from the owner and only with finite, forward values.

        Return whether the command was accepted. Rejected commands never refresh the
        expiry timer, so a competing source cannot keep the car moving.
        """
        source = CommandSource(source)
        if (source != self.owner or source == CommandSource.NONE
                or not _finite(steering_rad, speed_mps, now) or speed_mps < 0):
            self.rejected += 1
            return False
        self._command = DriveCommand(float(steering_rad), float(speed_mps), source)
        self._received_s = now
        return True

    def output(self, now):
        """Return the owner's latest command, or STOP if none is fresh."""
        age = now - self._received_s
        if self._command is None or not 0 <= age <= self.timeout_s:
            return STOP
        return self._command

    def age(self, now):
        """Age of the latest accepted command in seconds, or None."""
        return None if self._command is None else now - self._received_s


class ExpiryWatchdog:
    """Independent last-line timeout used by the actuator process."""

    def __init__(self, timeout_s):
        if not _finite(timeout_s) or timeout_s <= 0:
            raise ValueError("watchdog timeout must be positive")
        self.timeout_s = timeout_s
        self._command = None
        self._received_s = -math.inf
        self.expired = True

    def feed(self, command, now):
        self._command = command
        self._received_s = now

    def reset(self):
        self._command = None
        self._received_s = -math.inf

    def output(self, now):
        """Return the last command while fresh; otherwise STOP and mark the expiry."""
        age = now - self._received_s
        self.expired = self._command is None or not 0 <= age <= self.timeout_s
        return STOP if self.expired else self._command


class SpeedDemand:
    """Turn AVLite acceleration requests into a bounded forward-speed command.

    AVLite controllers output acceleration (m/s^2); the Ackermann driver expects speed.
    Integrate a demand, and when braking never let it exceed what the measured speed
    allows, so stale demand cannot launch the car when acceleration resumes.
    """

    def __init__(self, limit_mps):
        self.limit_mps = limit_mps
        self.demand_mps = 0.0

    def reset(self):
        self.demand_mps = 0.0

    def update(self, acceleration, measured_mps, dt):
        """Return the new speed command after one control period dt (seconds)."""
        if not _finite(acceleration, measured_mps, dt) or dt <= 0:
            self.reset()
            return 0.0
        if acceleration < 0:
            self.demand_mps = min(self.demand_mps, max(measured_mps, 0.0))
        self.demand_mps = min(max(self.demand_mps + acceleration * dt, 0.0), self.limit_mps)
        return self.demand_mps


def to_hardware(command, profile):
    """Convert a vehicle-convention command into the driver's Ackermann values.

    Clamp steering to the measured servo range, apply the measured sign and centre offset,
    and clamp speed to the motor ceiling. Return (steering_angle, speed) for the
    AckermannDriveStamped published to the driver. Stops always return speed 0.
    """
    profile.require_commissioned()
    limit = profile.get("steering.max_angle_rad")
    steering = min(max(command.steering_rad, -limit), limit)
    if not profile.get("steering.positive_left"):
        steering = -steering
    steering += profile.get("steering.center_offset_rad", 0.0)
    speed = min(max(command.speed_mps, 0.0), profile.get("motor.max_command_speed_mps"))
    if command.is_stop:
        speed = float(profile.get("motor.stop_command_speed_mps", 0.0))
    return float(steering), float(speed)
