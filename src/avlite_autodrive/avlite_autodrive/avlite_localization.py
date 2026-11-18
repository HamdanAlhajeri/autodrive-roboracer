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
from scipy.spatial import cKDTree

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
    reference: str = "first_scan"  # or "map_hits": the planned map's recorded LiDAR returns
    alignment_tolerance_m: float = 0.10  # a scan point this close to the map counts as aligned


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
    reference = block.get("reference", "first_scan")
    if reference not in ("first_scan", "map_hits"):
        raise ValueError("localization.reference must be first_scan or map_hits")
    return AvliteLocalizationConfig(
        reference=reference,
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


def _relative(a, b):
    """Pose b expressed in pose a's frame (planar)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy,
            math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2])))


class AutoDRIVELidarLocalization(LidarLocalization):
    """AVLite's ICP localization fed real hits, seeded at the spawn pose, with odometry speed.

    With an odometry_pose source, each step's ICP starts from the previous estimate moved
    by the wheel-odometry motion since the last step. ICP alone starts from the previous
    estimate, and between two parallel walls it cannot see motion along them, so it falls
    behind on straights; the prediction gives it the motion and ICP corrects the drift.
    """

    def __init__(self, perception_model, settings, speed_source, odometry_pose=None,
                 reference_points=None):
        apply_perception_settings(settings)
        super().__init__(perception_model, PerceptionSettings)
        self.settings = settings
        self.speed_source = speed_source  # callable -> wheel-odometry speed (m/s)
        self.odometry_pose = odometry_pose  # callable -> (x, y, yaw) in odom frame, or None
        self._last_odom = None
        # A prebuilt world-frame map. Without it, AVLite's reference is the first scan, which
        # drifts away from the start. A KD-tree makes each nearest-neighbour step fast.
        self._reference = None
        self._tree = None
        if reference_points is not None and len(reference_points):
            self._reference = np.asarray(reference_points, dtype=float)[:, :2]
            self._tree = cKDTree(self._reference)
        self.alignment = None  # fraction of this scan's hits within the tolerance of the map
        self.hits = 0
        self.seed()

    def _nearest_neighbours(self, points, reference):
        """Same matches as AVLite's brute-force search, from a KD-tree."""
        tree = self._tree if reference is self._map and self._tree is not None else cKDTree(reference)
        dists, idx = tree.query(points)
        return idx, dists

    def seed(self):
        """Put the ego state at the spawn pose; the next scan becomes the reference map.

        With a prebuilt map the estimate and reference are set now, so the first scan is
        already aligned to the real track.
        """
        ego = self.perception_model.ego_vehicle
        ego.x, ego.y, ego.theta = self.settings.start_pose_world
        self._last_odom = None
        if self._reference is not None:
            self._x, self._y, self._theta = (float(v) for v in self.settings.start_pose_world)
            self._map = self._reference

    def _predict(self):
        """Move the running estimate by the odometry motion since the previous step."""
        odom = self.odometry_pose() if self.odometry_pose else None
        if odom is None:
            return
        if self._last_odom is not None and self._x is not None:
            dx, dy, dyaw = _relative(self._last_odom, odom)
            c, s = math.cos(self._theta), math.sin(self._theta)
            self._x, self._y = self._x + c * dx - s * dy, self._y + s * dx + c * dy
            self._theta += dyaw
        self._last_odom = odom

    def localize(self, perception_model=None, sensors=None):
        if sensors is not None:
            sensors = _HitsOnly(sensors, self.settings.scan_subsample)
            self._predict()
            predicted = self._pose_estimate()
        else:
            predicted = None
        super().localize(perception_model, sensors)
        if predicted is not None and self._tree is not None:
            self._keep_better_than_prediction(sensors, predicted)
        if sensors is not None and self._tree is not None:
            self._score(sensors)
        speed = self.speed_source()
        if speed is not None and math.isfinite(speed):
            self.perception_model.ego_vehicle.velocity = speed

    def _pose_estimate(self):
        if self._x is None or self._map is None:
            return None
        return (self._x, self._y, self._theta)

    def _keep_better_than_prediction(self, sensors, predicted):
        """Undo an ICP step that fits the map worse than the odometry-predicted pose.

        A bad scan (a person walking past, a glare spike) can drag ICP away from where the
        wheels say the car is; the prediction alone is then the better estimate.
        """
        try:
            scan = self._squash(sensors.lidar_sensor.to_base(sensors.lidar))
            if scan is None or len(scan) < 3 or self._x is None:
                return
            if (self._alignment_at(scan, predicted)
                    > self._alignment_at(scan, (self._x, self._y, self._theta))):
                self._x, self._y, self._theta = predicted
                ego = self.perception_model.ego_vehicle
                ego.x, ego.y, ego.theta = predicted
        except Exception:  # A guard failure must never stop localization.
            return

    def recover(self, sensors, radius_m=1.0, step_m=0.5, yaw_span=0.2, accept=0.6):
        """Search around the estimate for a pose that fits the map better; True if adopted.

        Tries a grid of start poses (position within radius_m, yaw within yaw_span), runs ICP
        from each and keeps the best-aligned result. Ties go to the pose closest to the
        current estimate, so a featureless corridor does not move the car for nothing.
        Only a result aligned at least ``accept`` is adopted.
        """
        if self._tree is None or self._x is None:
            return False
        sensors = _HitsOnly(sensors, self.settings.scan_subsample)
        scan = self._squash(sensors.lidar_sensor.to_base(sensors.lidar))
        if scan is None or len(scan) < 3:
            return False
        offsets = np.arange(-radius_m, radius_m + 1e-9, step_m)
        best, best_key = None, None
        for dyaw in (-yaw_span, 0.0, yaw_span):
            for dx in offsets:
                for dy in offsets:
                    x, y, theta = self._icp(scan, self._map, self._x + dx, self._y + dy,
                                            self._theta + dyaw)
                    score = self._alignment_at(scan, (x, y, theta))
                    key = (round(score, 2), -math.hypot(x - self._x, y - self._y))
                    if best_key is None or key > best_key:
                        best, best_key = (x, y, theta, score), key
        if best is None or best[3] < accept:
            return False
        self._x, self._y, self._theta = best[:3]
        ego = self.perception_model.ego_vehicle
        ego.x, ego.y, ego.theta = best[:3]
        self.alignment = best[3]
        return True

    def _alignment_at(self, scan, pose):
        """Share of body-frame scan points within tolerance of the map at a pose."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = scan @ np.array([[c, -s], [s, c]]).T + np.array(pose[:2])
        dists, _ = self._tree.query(world)
        return float(np.mean(dists <= self.settings.alignment_tolerance_m))

    def _score(self, sensors):
        """Share of this scan's real hits that land on the map at the estimated pose."""
        try:
            world = sensors.lidar_sensor.to_map(sensors.lidar, self.perception_model.ego_vehicle)
            dists, _ = self._tree.query(np.asarray(world, dtype=float)[:, :2])
        except Exception:  # A scoring failure must never stop localization.
            return
        self.hits = len(dists)
        self.alignment = (float(np.mean(dists <= self.settings.alignment_tolerance_m))
                          if len(dists) else None)

    def reset(self):
        """After a simulator reset the car is back at the spawn pose: start over there."""
        super().reset()
        self.seed()
