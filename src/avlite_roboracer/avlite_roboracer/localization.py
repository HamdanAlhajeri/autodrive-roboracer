"""Localization health shared by the live READY gate and the offline report (no ROS).

Health combines pose freshness, reported uncertainty, scan alignment against the
finalized map, and agreement between map-frame motion and wheel odometry.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from .geometry import compose, relative, wrap_angle


def scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Return sensor-frame x/y of beams with a real return (no-return beams excluded)."""
    ranges = np.array([np.nan if r is None else r for r in ranges], dtype=float)
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max)
    return np.column_stack((ranges[valid] * np.cos(angles[valid]),
                            ranges[valid] * np.sin(angles[valid])))


def transform_points(points, pose):
    """Apply a planar pose to an N-by-2 array."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.column_stack((pose[0] + c * points[:, 0] - s * points[:, 1],
                            pose[1] + s * points[:, 0] + c * points[:, 1]))


def scan_alignment(grid, sensor_points, base_pose, laser_mount, tolerance_m):
    """Fraction of scan endpoints within tolerance of an occupied map cell, and median distance.

    base_pose is base_link in the map frame. Returns (fraction, median_m); (0.0, inf) when
    the scan has no returns.
    """
    if not len(sensor_points):
        return 0.0, math.inf
    world = transform_points(sensor_points, compose(base_pose, laser_mount))
    distance = grid.distance_to_occupied(world)
    return float(np.mean(distance <= tolerance_m)), float(np.median(distance))


def motion_mismatch(previous_map, current_map, previous_odom, current_odom):
    """Compare the motion implied by two map poses with the odometry motion between them.

    Return (translation_m, rotation_rad). A localization correction appears as a large
    mismatch even when the car moved smoothly.
    """
    map_delta = relative(previous_map, current_map)
    odom_delta = relative(previous_odom, current_odom)
    return (math.hypot(map_delta[0] - odom_delta[0], map_delta[1] - odom_delta[1]),
            abs(wrap_angle(map_delta[2] - odom_delta[2])))


@dataclass
class LocalizationThresholds:
    pose_timeout_s: float
    max_position_stddev_m: float
    max_yaw_stddev_rad: float
    min_scan_alignment: float
    scan_alignment_tolerance_m: float
    max_motion_mismatch_m: float
    max_motion_mismatch_rad: float
    stable_duration_s: float

    @classmethod
    def from_profile(cls, profile):
        """Read thresholds; raise ValueError if any acceptance value is still unmeasured."""
        keys = {"pose_timeout_s": "timing.pose_timeout_s"}
        for name in cls.__dataclass_fields__:
            keys.setdefault(name, "localization." + name)
        values = {name: profile.get(path) for name, path in keys.items()}
        missing = [keys[n] for n, v in values.items() if v is None]
        if missing:
            raise ValueError("Localization thresholds not established: " + ", ".join(missing))
        return cls(**values)


@dataclass
class LocalizationMonitor:
    """Track live localization quality and how long it has stayed acceptable."""

    thresholds: LocalizationThresholds
    reasons: list = field(default_factory=lambda: ["no localization estimate"])
    stable_since: float = None
    last_pose_time: float = None
    last: dict = field(default_factory=dict)
    _previous: tuple = None

    def reset(self):
        self.reasons = ["no localization estimate"]
        self.stable_since = None
        self.last_pose_time = None
        self.last = {}
        self._previous = None

    def update(self, now, map_pose, odom_pose, covariance, alignment):
        """Evaluate one pose estimate.

        map_pose and odom_pose are base_link in the map and odom frames at the same time;
        covariance holds the x, y, yaw variances (None when unreported); alignment is the
        scan_alignment() fraction for the scan used by that estimate.
        """
        t = self.thresholds
        reasons = []
        if (self.last_pose_time is not None
                and not 0 < now - self.last_pose_time <= t.pose_timeout_s):
            self.stable_since = None
            self._previous = None
        if not all(math.isfinite(v) for v in (*map_pose, *odom_pose)):
            self.reset()
            self.last_pose_time = now
            self.reasons = ["localization pose is not finite"]
            return
        cov = [c if c is not None and math.isfinite(c) and c >= 0 else None
               for c in (covariance or [])]
        if len(cov) != 3 or any(c is None for c in cov):
            reasons.append("pose covariance unavailable")
            stddev = (None, None)
        else:
            stddev = (math.sqrt(max(cov[0], 0) + max(cov[1], 0)), math.sqrt(max(cov[2], 0)))
            if stddev[0] > t.max_position_stddev_m:
                reasons.append(f"position uncertainty {stddev[0]:.3f} m")
            if stddev[1] > t.max_yaw_stddev_rad:
                reasons.append(f"heading uncertainty {stddev[1]:.3f} rad")
        if alignment is not None and (not math.isfinite(alignment) or not 0 <= alignment <= 1):
            alignment = None
        if alignment is None or alignment < t.min_scan_alignment:
            reasons.append("scan does not align with the map"
                           if alignment is not None else "scan alignment unavailable")
        mismatch = (0.0, 0.0)
        if self._previous is not None:
            mismatch = motion_mismatch(self._previous[0], map_pose, self._previous[1], odom_pose)
            if mismatch[0] > t.max_motion_mismatch_m or mismatch[1] > t.max_motion_mismatch_rad:
                reasons.append("map motion disagrees with odometry")
        self._previous = (map_pose, odom_pose)
        self.last_pose_time = now
        self.last = {"pose": list(map_pose), "position_stddev_m": stddev[0],
                     "yaw_stddev_rad": stddev[1], "scan_alignment": alignment,
                     "motion_mismatch_m": mismatch[0], "motion_mismatch_rad": mismatch[1]}
        if reasons:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now
        self.reasons = reasons

    def failures(self, now):
        """Return reasons the current localization is not acceptable (empty when healthy)."""
        if self.last_pose_time is None:
            return ["no localization estimate"]
        age = now - self.last_pose_time
        if not 0 <= age <= self.thresholds.pose_timeout_s:
            return [f"localization estimate is {age:.2f}s old"]
        reasons = list(self.reasons)
        if not reasons and (now - self.stable_since) < self.thresholds.stable_duration_s:
            reasons.append(f"localization stable for only {now - self.stable_since:.1f}s")
        return reasons

    def snapshot(self, now):
        stable = (now - self.stable_since) if self.stable_since is not None else 0.0
        age = None if self.last_pose_time is None else now - self.last_pose_time
        return {**self.last, "pose_age_s": age, "stable_for_s": stable,
                "failures": self.failures(now)}
