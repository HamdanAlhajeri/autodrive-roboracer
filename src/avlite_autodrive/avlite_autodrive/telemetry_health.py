"""ROS-independent checks for the publication feeding the simulator actuator."""

import math


class ActuatorPublicationHealth:
    FIELDS = ("throttle_command", "steering_command")

    def __init__(self, timeout=0.5):
        """Start tracking freshness and validity of outgoing throttle and steering messages.

        Keep the first detected error for the entire recording, even if publication later
        recovers.
        """
        self.timeout = timeout
        self.error = None
        self.max_age_s = dict.fromkeys(self.FIELDS, 0.0)
        self.complete = False

    def check(self, now, state, received, *, moving=False, startup_expired=False):
        """Check current command values and their monotonic receive times.

        Allow missing messages during stationary startup, but flag them once the car moves
        or the startup wait expires. Stale or invalid values fail immediately. Return the
        first recorded error, or None when no error has occurred.
        """
        missing, stale, invalid = [], [], []
        for field in self.FIELDS:
            if field not in received:
                missing.append(field)
                continue
            age = now - received[field]
            self.max_age_s[field] = max(self.max_age_s[field], age)
            if not 0 <= age <= self.timeout:
                stale.append(field)
            value = state.get(field)
            if (not isinstance(value, (int, float)) or not math.isfinite(value)
                    or not -1 <= value <= 1):
                invalid.append(field)
        self.complete = not (missing or stale or invalid)
        if self.error is None:
            if invalid:
                self.error = "Invalid actuator publication: " + ", ".join(invalid)
            elif stale:
                self.error = "Actuator publication expired: " + ", ".join(stale)
            elif missing and (moving or startup_expired):
                self.error = "Missing actuator publication: " + ", ".join(missing)
        return self.error

    @property
    def valid(self):
        """Require a complete current command pair and no earlier publication failure."""
        return self.complete and self.error is None
