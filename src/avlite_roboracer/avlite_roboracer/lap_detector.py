"""Detect completion of the autonomous mapping laps without simulator counters (no ROS).

The starting pose defines a finish line across the track. Mapping drives
``LapSettings.laps`` laps; each lap counts only after:

1. the car departs the start area,
2. wheel odometry reports enough travel,
3. the car crosses the finish line forwards, by its own motion, with matching heading,
4. the current scan resembles the scan recorded at the start.

Departure and travel limits apply to each lap. Travel comes from odometry, not map poses, so a localization correction cannot add
distance; a crossing during an update whose map motion disagrees with odometry (a
correction) is ignored.
"""

from dataclasses import dataclass
import math

import numpy as np

from .geometry import relative, wrap_angle
from .localization import motion_mismatch


@dataclass(frozen=True)
class LapSettings:
    departure_radius_m: float = 2.0
    min_travel_m: float = 10.0
    max_travel_m: float = 200.0
    finish_half_width_m: float = 1.0
    heading_tolerance_rad: float = math.radians(30)
    correction_tolerance_m: float = 0.15
    correction_tolerance_rad: float = math.radians(5)
    scan_match_tolerance_m: float = 0.15
    min_scan_match: float = 0.6
    laps: int = 1

    def __post_init__(self):
        if isinstance(self.laps, bool) or not isinstance(self.laps, int):
            raise ValueError("lap setting laps must be a whole number")
        for name, value in vars(self).items():
            if not (math.isfinite(value) and value > 0):
                raise ValueError(f"lap setting {name} must be positive")
        if self.min_travel_m >= self.max_travel_m or self.min_scan_match > 1:
            raise ValueError("invalid lap travel or scan-match settings")


def scan_similarity(a, b, tolerance_m):
    """Fraction of beams whose ranges agree within tolerance (both returns valid).

    Scans must have the same beam count. Beams without a return in both scans are ignored.
    A return in only one scan counts as half a disagreement: dark or shiny surfaces drop beams
    at random, and with 25 % dropout in each scan a full penalty capped the score near 0.6 and
    the finish line was never confirmed. A different place still scores low, because its
    beams that do return disagree.
    """
    a = np.array([np.nan if r is None else r for r in a], dtype=float)
    b = np.array([np.nan if r is None else r for r in b], dtype=float)
    if a.shape != b.shape or not len(a):
        return 0.0
    va, vb = np.isfinite(a), np.isfinite(b)
    both = va & vb
    one_sided = (va ^ vb).sum()
    if not both.any() and not one_sided:
        return 0.0
    agree = both & (np.abs(np.where(both, a - b, 0.0)) <= tolerance_m)
    return float(agree.sum() / (both.sum() + 0.5 * one_sided))


class LapDetector:
    def __init__(self, settings=None):
        self.settings = settings or LapSettings()
        self.start_pose = None
        self.start_scan = None
        self.state = "idle"
        self.travel_m = 0.0
        self.lap_travel_m = 0.0
        self.laps_completed = 0
        self.departed = False
        self.ignored_corrections = 0
        self.rejected_crossings = []
        self._previous = None

    def start(self, map_pose, odom_pose, scan_ranges):
        """Record the start pose (defining finish line and direction) and reference scan."""
        self.start_pose = tuple(map_pose)
        self.start_scan = list(scan_ranges)
        self.state = "mapping"
        self.travel_m = 0.0
        self.lap_travel_m = 0.0
        self.laps_completed = 0
        self.departed = False
        self.ignored_corrections = 0
        self.rejected_crossings = []
        self._previous = (tuple(map_pose), tuple(odom_pose))

    def _along(self, pose):
        """Signed distance along the start heading and lateral offset from the start."""
        local = relative(self.start_pose, pose)
        return local[0], local[1]

    def update(self, map_pose, odom_pose, scan_ranges):
        """Process one localized update; return 'complete', 'mapping' or 'failed'.

        'complete' means the last of the configured laps has been confirmed.

        map_pose and odom_pose are base_link in the map and odom frames at the same instant.
        """
        if self.state != "mapping":
            return self.state
        s = self.settings
        previous_map, previous_odom = self._previous
        self._previous = (tuple(map_pose), tuple(odom_pose))
        odom_step = relative(previous_odom, odom_pose)
        step = math.hypot(odom_step[0], odom_step[1])
        self.travel_m += step
        self.lap_travel_m += step
        if self.lap_travel_m > s.max_travel_m:
            self.state = "failed"
            return self.state
        distance = math.hypot(map_pose[0] - self.start_pose[0], map_pose[1] - self.start_pose[1])
        if distance > s.departure_radius_m:
            self.departed = True
        before, _ = self._along(previous_map)
        after, lateral = self._along(map_pose)
        if not (self.departed and before < 0 <= after and abs(lateral) <= s.finish_half_width_m):
            return self.state
        mismatch = motion_mismatch(previous_map, map_pose, previous_odom, odom_pose)
        if mismatch[0] > s.correction_tolerance_m or mismatch[1] > s.correction_tolerance_rad:
            self.ignored_corrections += 1
            self.rejected_crossings.append("localization correction")
            return self.state
        if odom_step[0] <= 0:
            self.rejected_crossings.append("not moving forwards")
            return self.state
        if self.lap_travel_m < s.min_travel_m:
            self.rejected_crossings.append(f"only {self.lap_travel_m:.1f} m travelled")
            return self.state
        if abs(wrap_angle(map_pose[2] - self.start_pose[2])) > s.heading_tolerance_rad:
            self.rejected_crossings.append("heading does not match the start")
            return self.state
        match = scan_similarity(scan_ranges, self.start_scan, s.scan_match_tolerance_m)
        if match < s.min_scan_match:
            self.rejected_crossings.append(f"scan match {match:.2f} below {s.min_scan_match}")
            return self.state
        self.laps_completed += 1
        if self.laps_completed >= s.laps:
            self.state = "complete"
        else:  # the next lap must leave the start area and travel far enough again
            self.departed = False
            self.lap_travel_m = 0.0
        return self.state

    def summary(self):
        return {"state": self.state, "laps_completed": self.laps_completed,
                "laps": self.settings.laps, "travel_m": self.travel_m,
                "lap_travel_m": self.lap_travel_m, "departed": self.departed,
                "ignored_corrections": self.ignored_corrections,
                "rejected_crossings": list(self.rejected_crossings[-10:]),
                "start_pose": list(self.start_pose) if self.start_pose else None}
