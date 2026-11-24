"""Detects a car that is commanded to drive but is not moving.

On 24 Nov the first autonomous floor run commanded 0.8 m/s for about 27 s while the wheels
turned at 0.05 to 0.14 m/s, and later stalled twice in a row. Nothing noticed: the supervisor
kept pushing the speed command into a motor that was not following it. This stops the session
instead, with the numbers in the reason, so the cause is looked at before the next try.

No ROS here: the caller passes the commanded and the measured speed.
"""


class StallMonitor:
    def __init__(self, window_s=6.0, min_command_mps=0.3, min_ratio=0.25):
        self.window_s = window_s
        self.min_command_mps = min_command_mps
        self.min_ratio = min_ratio
        self.since = None

    def reset(self):
        self.since = None

    def update(self, now, command_mps, measured_mps):
        """Return a reason string once the car has been stalled for window_s, else None."""
        if self.window_s <= 0:
            return None
        command = abs(command_mps)
        if command < self.min_command_mps or abs(measured_mps) >= self.min_ratio * command:
            self.since = None
            return None
        if self.since is None:
            self.since = now
        if now - self.since >= self.window_s:
            return (f"car not following the speed command: commanded {command:.2f} m/s, "
                    f"measured {abs(measured_mps):.2f} m/s for {now - self.since:.1f} s")
        return None
