"""AVLite's own LiDAR localization (ICP) as the planned algorithm's pose source.

With ``localization.source: avlite`` in ``config/avlite.yaml``, the SyncExecuter runs
AVLite's ``LidarLocalization`` each tick instead of copying the simulator's true pose.
It aligns every LiDAR scan to a reference scan taken at the start (ICP) and writes the
pose into AVLite's ego state; speed comes from non-ground-truth wheel odometry
(``avlite_autodrive.sim_odometry``). Ground truth is read only for the
``localization_error_m`` diagnostic.

The estimate is seeded from ``start_pose_world``: where the car spawns, in the simulator's
world frame, so the world-frame race map applies. That is one fixed constant, like
surveying the start line, re-applied after every simulator reset.

AVLite's ICP is unchanged. This subclass only feeds it real LiDAR hits (our cloud puts
no-return beams at maximum range for Follow the Gap) and thins scan and reference map so
one step fits the 20 Hz control loop.
"""

from dataclasses import dataclass
import math

import numpy as np

from avlite.c10_perception.c16_localization_algs import LidarLocalization
from avlite.c10_perception.c19_settings import PerceptionSettings


@dataclass(frozen=True)
class AvliteLocalizationConfig:
    start_pose_world: tuple      # (x, y, yaw) where the car spawns, world frame
    scan_subsample: int = 4      # keep every Nth real LiDAR hit
    map_subsample: int = 4       # keep every Nth reference-map point
    max_iterations: int = 10     # ICP iterations per step
    max_correspondence_m: float = 1.0
    tolerance_m: float = 1e-4


def _positive_int(block, name, default):
    value = block.get(name, default)
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"localization.{name} must be a positive integer")
    return value


def avlite_localization_config(config):
    """Return an AvliteLocalizationConfig for ``source: avlite``, or None otherwise."""
    block = config.get("localization") or {}
    if block.get("source", "ground_truth") != "avlite":
        return None
    pose = block.get("start_pose_world")
    if (not isinstance(pose, (list, tuple)) or len(pose) != 3
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       and math.isfinite(v) for v in pose)):
        raise ValueError("localization.start_pose_world must be [x, y, yaw] for source: avlite")
    distance = block.get("max_correspondence_m", 1.0)
    if (isinstance(distance, bool) or not isinstance(distance, (int, float))
            or not math.isfinite(distance) or distance <= 0):
        raise ValueError("localization.max_correspondence_m must be a positive number")
    return AvliteLocalizationConfig(
        start_pose_world=tuple(float(v) for v in pose),
        scan_subsample=_positive_int(block, "scan_subsample", 4),
        map_subsample=_positive_int(block, "map_subsample", 4),
        max_iterations=_positive_int(block, "max_iterations", 10),
        max_correspondence_m=float(distance),
    )


def apply_perception_settings(settings):
    """Set AVLite's class-level ICP settings before constructing the localizer."""
    PerceptionSettings.c16_localization_icp_max_iterations = settings.max_iterations
    PerceptionSettings.c16_localization_icp_max_correspondence_dist = settings.max_correspondence_m
    PerceptionSettings.c16_localization_icp_tolerance = settings.tolerance_m
    PerceptionSettings.c16_localization_map_subsample = settings.map_subsample


class _HitsOnly:
    """A sensor frame view with only real LiDAR returns, thinned."""

    def __init__(self, frame, step):
        self._frame = frame
        mask = getattr(frame, "lidar_hit_mask", None)
        points = frame.lidar
        if points is not None and mask is not None and len(mask) == len(points):
            points = points[np.asarray(mask, dtype=bool)]
        self.lidar = None if points is None else points[::step]

    def __getattr__(self, name):
        return getattr(self._frame, name)


class AutoDRIVELidarLocalization(LidarLocalization):
    """AVLite's ICP localization fed real hits, seeded at the spawn pose, with odometry speed."""

    def __init__(self, perception_model, settings, speed_source):
        apply_perception_settings(settings)
        super().__init__(perception_model, PerceptionSettings)
        self.settings = settings
        self.speed_source = speed_source  # callable -> wheel-odometry speed (m/s)
        self.seed()

    def seed(self):
        """Put the ego state at the spawn pose; the next scan becomes the reference map."""
        ego = self.perception_model.ego_vehicle
        ego.x, ego.y, ego.theta = self.settings.start_pose_world

    def localize(self, perception_model=None, sensors=None):
        if sensors is not None:
            sensors = _HitsOnly(sensors, self.settings.scan_subsample)
        super().localize(perception_model, sensors)
        speed = self.speed_source()
        if speed is not None and math.isfinite(speed):
            self.perception_model.ego_vehicle.velocity = speed

    def reset(self):
        """After a simulator reset the car is back at the spawn pose: start over there."""
        super().reset()
        self.seed()
