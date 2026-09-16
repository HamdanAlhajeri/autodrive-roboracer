"""Count observed finish crossings and time complete intervals between them.

The first crossing may follow a stationary start or a mid-lap recording start.
It is deliberately excluded from rolling lap times. A lap-counter discontinuity
invalidates the run and breaks timing continuity; missed laps are never inferred.
"""

import math
from numbers import Real
from statistics import fmean, pstdev


class LapProgress:
    def __init__(self, requested_laps=None):
        if requested_laps is not None and (
            isinstance(requested_laps, bool)
            or not isinstance(requested_laps, int)
            or requested_laps <= 0
        ):
            raise ValueError("requested_laps must be a positive integer or None")
        self.requested_laps = requested_laps
        self.completed_laps = 0
        self.counter_discontinuity = False
        self.finish_times_s = []
        self.lap_times_s = []
        self._previous_count = None
        self._previous_finish_s = None
        self._previous_observation_s = None

    @property
    def target_reached(self):
        return (
            self.requested_laps is not None
            and self.completed_laps >= self.requested_laps
        )

    def observe(self, count, elapsed_s):
        """Accept one counter update; absent/invalid values are ignored.

        Only a +1 update is an observed finish crossing. A reset or skipped
        count is flagged, then establishes a new baseline for later updates.
        """
        if (
            isinstance(count, bool)
            or not isinstance(count, Real)
            or not math.isfinite(count)
            or count < 0
            or int(count) != count
            or isinstance(elapsed_s, bool)
            or not isinstance(elapsed_s, Real)
            or not math.isfinite(elapsed_s)
            or elapsed_s < 0
        ):
            return
        count = int(count)
        elapsed_s = float(elapsed_s)
        if (
            self._previous_observation_s is not None
            and elapsed_s < self._previous_observation_s
        ):
            self.counter_discontinuity = True
            self._previous_finish_s = None
            self._previous_count = count
            self._previous_observation_s = elapsed_s
            return
        self._previous_observation_s = elapsed_s
        if self._previous_count is None:
            self._previous_count = count
            return
        delta = count - self._previous_count
        self._previous_count = count
        if delta == 0:
            return
        if delta != 1:
            self.counter_discontinuity = True
            self._previous_finish_s = None
            return
        self.completed_laps += 1
        self.finish_times_s.append(elapsed_s)
        if self._previous_finish_s is not None:
            duration = elapsed_s - self._previous_finish_s
            if duration > 0:
                self.lap_times_s.append(duration)
            else:
                # Two crossings at one timestamp cannot provide a lap time.
                self.counter_discontinuity = True
        self._previous_finish_s = elapsed_s

    def summary(self):
        return {
            "requested_laps": self.requested_laps,
            "completed_laps": self.completed_laps,
            "counter_discontinuity": self.counter_discontinuity,
            "first_lap_elapsed_s": (
                self.finish_times_s[0] if self.finish_times_s else None
            ),
            "lap_times_s": list(self.lap_times_s),
            "lap_time_mean_s": fmean(self.lap_times_s) if self.lap_times_s else None,
            "lap_time_std_s": pstdev(self.lap_times_s) if self.lap_times_s else None,
        }
