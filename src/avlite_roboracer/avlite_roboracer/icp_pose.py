"""AVLite ICP localization for the real car: scan-to-map matching with odometry prediction.

After the mapping lap, the finalized occupancy grid's occupied cells are the reference map.
Each LiDAR scan is aligned to it with AVLite's own ICP (``LidarLocalization``), starting
from the previous pose moved by the wheel-odometry motion since the last scan. The
estimate is seeded at the pose where the mapping lap ended, so no global search is needed.

SLAM Toolbox localization slid about 1 m along the corridor in the simulator and held the
wrong pose while its own checks stayed healthy. With the odometry prediction, ICP's
along-track error is pulled back after each short burst, and it finished clean laps.

No ROS here: the runtime passes in a sensor frame and an odometry pose.
"""

from dataclasses import dataclass
import math

import numpy as np

SOURCES = ("icp", "slam")


@dataclass(frozen=True)
class IcpSettings:
    scan_subsample: int = 4         # keep every Nth real LiDAR hit
    max_iterations: int = 10        # ICP iterations per scan
    max_correspondence_m: float = 1.0
    alignment_tolerance_m: float = 0.10
    # Recovery: when alignment stays below lost_alignment for lost_scans scans in a row, search
    # a grid around the estimate and adopt the best fit. recovery_radius_m 0 turns it off.
    lost_alignment: float = 0.5
    lost_scans: int = 5
    recovery_radius_m: float = 1.0
    recovery_accept: float = 0.6


def localization_source(race_config):
    """Return 'icp' or 'slam' from the race config's localization block (default icp)."""
    block = (race_config or {}).get("localization") or {}
    source = block.get("source", "icp")
    if source not in SOURCES:
        raise ValueError(f"localization.source must be one of {SOURCES}")
    return source


def icp_settings(race_config):
    """Read and validate the ICP tuning values; raise ValueError naming a bad one."""
    block = (race_config or {}).get("localization") or {}
    defaults = IcpSettings()
    values = {}
    for name in ("scan_subsample", "max_iterations"):
        value = block.get(name, getattr(defaults, name))
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError(f"localization.{name} must be a positive integer")
        values[name] = value
    for name in ("max_correspondence_m", "alignment_tolerance_m"):
        value = block.get(name, getattr(defaults, name))
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0):
            raise ValueError(f"localization.{name} must be a positive number")
        values[name] = float(value)
    value = block.get("lost_scans", defaults.lost_scans)
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError("localization.lost_scans must be a positive integer")
    values["lost_scans"] = value
    for name in ("lost_alignment", "recovery_accept"):
        value = block.get(name, getattr(defaults, name))
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not 0 < value <= 1):
            raise ValueError(f"localization.{name} must be a number in (0, 1]")
        values[name] = float(value)
    value = block.get("recovery_radius_m", defaults.recovery_radius_m)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0):
        raise ValueError("localization.recovery_radius_m must be zero or a positive number")
    values["recovery_radius_m"] = float(value)
    return IcpSettings(**values)


class IcpPose:
    """Running map-frame pose of base_link from ICP against a fixed reference map."""

    def __init__(self, reference_points, start_pose, settings=None):
        from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
        from avlite_autodrive.avlite_localization import (
            AutoDRIVELidarLocalization, AvliteLocalizationConfig)
        reference = np.asarray(reference_points, dtype=float).reshape(-1, 2)
        if len(reference) < 50:
            raise ValueError(f"reference map has only {len(reference)} occupied points")
        if len(start_pose) != 3 or not all(math.isfinite(v) for v in start_pose):
            raise ValueError("start pose must be three finite numbers")
        self.settings = settings or IcpSettings()
        self._odom = None
        self._speed = None
        config = AvliteLocalizationConfig(
            start_pose_world=tuple(float(v) for v in start_pose),
            scan_subsample=self.settings.scan_subsample,
            max_iterations=self.settings.max_iterations,
            max_correspondence_m=self.settings.max_correspondence_m,
            alignment_tolerance_m=self.settings.alignment_tolerance_m,
            reference="map_hits")
        self._pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
        self._localizer = AutoDRIVELidarLocalization(
            self._pm, config, lambda: self._speed, odometry_pose=lambda: self._odom,
            reference_points=reference)
        self.time = None  # monotonic time of the last update
        self.low_scans = 0    # consecutive scans below settings.lost_alignment
        self.recoveries = 0   # times a search moved the estimate
        self.recovery_tries = 0
        self.last_recovery = None  # (monotonic time, metres moved) of the latest adoption

    def update(self, sensors, odom_pose, speed, now):
        """Align one scan. odom_pose is base_link in the odom frame at the scan time."""
        self._odom = tuple(odom_pose) if odom_pose is not None else None
        self._speed = speed
        self._localizer.localize(sensors=sensors)
        self.time = now
        self._check_lost(sensors, now)

    def _check_lost(self, sensors, now):
        """Count scans that fit the map badly; after lost_scans in a row, search for a pose."""
        alignment = self.alignment
        if alignment is None or alignment >= self.settings.lost_alignment:
            self.low_scans = 0
            return
        self.low_scans += 1
        if self.low_scans < self.settings.lost_scans or self.settings.recovery_radius_m <= 0:
            return
        before = self.pose
        self.recovery_tries += 1
        self.low_scans = 0
        if self._localizer.recover(sensors, radius_m=self.settings.recovery_radius_m,
                                   accept=self.settings.recovery_accept):
            after = self.pose
            self.recoveries += 1
            self.last_recovery = (now, math.hypot(after[0] - before[0], after[1] - before[1]))

    @property
    def pose(self):
        """(x, y, yaw) of base_link in the map frame, or None before the first update."""
        if self.time is None:
            return None
        ego = self._pm.ego_vehicle
        pose = (float(ego.x), float(ego.y), float(ego.theta))
        return pose if all(math.isfinite(v) for v in pose) else None

    @property
    def alignment(self):
        """Share of the last scan's hits within tolerance of the map (None before a scan)."""
        return self._localizer.alignment

    def age(self, now):
        return math.inf if self.time is None else now - self.time
