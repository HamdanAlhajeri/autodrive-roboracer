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
import logging
import math
import time

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
    # Lost-pose recovery (0 radius = off): after lost_scans scans in a row aligned below
    # lost_alignment, search recovery_radius_m around the estimate; adopt a fit of recovery_accept+.
    recovery_radius_m: float = 0.0
    recovery_accept: float = 0.6
    lost_alignment: float = 0.5
    lost_scans: int = 5
    recovery_interval_s: float = 3.0  # at most one search this often (it takes a moment)
    recovery_budget_s: float = 0.0    # 0 = search the whole radius; else stop after this long (nearest first)
    # Wheel slip while launching makes the wheels turn faster than the car moves; wheel odometry then
    # over-reports distance and ICP cannot correct it in a straight corridor. The odometry prediction
    # may not move the estimate faster than the car can accelerate (0 = no limit).
    odometry_max_accel_mps2: float = 0.0
    odometry_limit_below_mps: float = 0.0  # apply that limit only below this speed (launch); 0 = at every speed
    # The pose handed to the controller is moved forward by speed x pose_lead_s, to cover the delay between
    # the scan being taken and the command it leads to (0 = off). The ICP's own estimate is not touched.
    pose_lead_s: float = 0.0


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
    def number(name, default, low, high):
        value = block.get(name, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not low <= value <= high):
            raise ValueError(f"localization.{name} must be a number from {low} to {high}")
        return float(value)

    return AvliteLocalizationConfig(
        reference=reference,
        recovery_radius_m=number("recovery_radius_m", 0.0, 0.0, 5.0),
        recovery_accept=number("recovery_accept", 0.6, 0.0, 1.0),
        lost_alignment=number("lost_alignment", 0.5, 0.0, 1.0),
        lost_scans=_positive_int(block, "lost_scans", 5),
        recovery_interval_s=number("recovery_interval_s", 3.0, 0.0, 60.0),
        recovery_budget_s=number("recovery_budget_s", 0.0, 0.0, 10.0),
        odometry_max_accel_mps2=number("odometry_max_accel_mps2", 0.0, 0.0, 20.0),
        odometry_limit_below_mps=number("odometry_limit_below_mps", 0.0, 0.0, 30.0),
        pose_lead_s=number("pose_lead_s", 0.0, 0.0, 0.5),
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


def after_interruption(localizer, teleported):
    """The runner saw sensors drop out or the car respawn: start over only if it was teleported.

    A teleport (simulator reset) puts the car back at the spawn pose. A sensor gap does not move
    the car, so re-seeding then would leave the estimate at the spawn point while the car is
    somewhere else on the track.
    """
    if teleported:
        localizer.reset()
    else:
        localizer.resume()


class AutoDRIVELidarLocalization(LidarLocalization):
    """AVLite's ICP localization fed real hits, seeded at the spawn pose, with odometry speed.

    With an odometry_pose source, each step's ICP starts from the previous estimate moved
    by the wheel-odometry motion since the last step. ICP alone starts from the previous
    estimate, and between two parallel walls it cannot see motion along them, so it falls
    behind on straights; the prediction gives it the motion and ICP corrects the drift.
    """

    def __init__(self, perception_model, settings, speed_source, odometry_pose=None,
                 reference_points=None, clock=time.monotonic):
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
        self.low_scans = 0       # scans in a row aligned below settings.lost_alignment
        self.recoveries = 0      # searches that moved the estimate
        self.recovery_tries = 0
        self._last_search = -math.inf
        self._clock = clock
        self._odom_time = None   # when the last odometry prediction was applied
        self._odom_speed = 0.0   # speed implied by the motion applied at the last prediction
        self.odometry_clipped = 0  # predictions shortened by the acceleration limit
        self._gap_search = False  # the next search follows a sensor gap: no time limit
        self._after_gap = False  # the next update follows a sensor gap: do not trust odometry blindly
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
        self._odom_time, self._odom_speed = None, 0.0  # a respawn starts from rest
        if self._reference is not None:
            self._x, self._y, self._theta = (float(v) for v in self.settings.start_pose_world)
            self._map = self._reference

    def _predict(self):
        """Move the running estimate by the odometry motion since the previous step."""
        odom = self.odometry_pose() if self.odometry_pose else None
        if odom is None:
            return
        now = self._clock()
        if self._last_odom is not None and self._x is not None:
            dx, dy, dyaw = _relative(self._last_odom, odom)
            dx, dy = self._limit_acceleration(dx, dy, now)
            c, s = math.cos(self._theta), math.sin(self._theta)
            self._x, self._y = self._x + c * dx - s * dy, self._y + s * dx + c * dy
            self._theta += dyaw
        self._last_odom = odom
        self._odom_time = now

    def _limit_acceleration(self, dx, dy, now):
        """Shorten an odometry step that implies the car sped up faster than it physically can."""
        limit = self.settings.odometry_max_accel_mps2
        if limit <= 0 or self._odom_time is None:
            return dx, dy
        dt = min(max(now - self._odom_time, 1e-3), 0.25)
        distance = math.hypot(dx, dy)
        allowed = (self._odom_speed + limit * dt) * dt
        below = self.settings.odometry_limit_below_mps
        if below > 0 and self._odom_speed >= below:
            allowed = math.inf  # cruising: odometry is honest here, and clipping its noise only makes the estimate lag
        if distance > allowed:
            dx, dy, distance = dx * allowed / distance, dy * allowed / distance, allowed
            self.odometry_clipped += 1
        self._odom_speed = distance / dt
        return dx, dy

    def localize(self, perception_model=None, sensors=None):
        original_sensors = sensors
        if sensors is not None:
            sensors = _HitsOnly(sensors, self.settings.scan_subsample)
            before = self._pose_estimate()
            self._predict()
            predicted = self._pose_estimate()
            if self._after_gap:
                predicted = self._first_update_after_gap(sensors, before, predicted)
        else:
            predicted = None
        super().localize(perception_model, sensors)
        if predicted is not None and self._tree is not None:
            self._keep_better_than_prediction(sensors, predicted)
        if sensors is not None and self._tree is not None:
            self._score(sensors)
            self._check_lost(original_sensors)
        speed = self.speed_source()
        if speed is not None and math.isfinite(speed):
            ego = self.perception_model.ego_vehicle
            ego.velocity = speed
            lead = self.settings.pose_lead_s
            if lead > 0 and self._x is not None:
                # Published pose = the ICP estimate moved forward; the next scan match starts from
                # self._x, so this does not accumulate.
                ego.x = self._x + speed * lead * math.cos(self._theta)
                ego.y = self._y + speed * lead * math.sin(self._theta)

    def _check_lost(self, sensors):
        """After lost_scans badly aligned scans in a row, search around the estimate for a better pose."""
        cfg = self.settings
        if cfg.recovery_radius_m <= 0 or self.alignment is None:
            return
        if self.alignment >= cfg.lost_alignment:
            self.low_scans = 0
            self._gap_search = False
            return
        self.low_scans += 1
        now = time.monotonic()
        if self.low_scans < cfg.lost_scans or now - self._last_search < cfg.recovery_interval_s:
            return
        self._last_search = now
        self.recovery_tries += 1
        started, before, low = time.monotonic(), self._pose_estimate(), self.alignment
        budget = None if self._gap_search or cfg.recovery_budget_s <= 0 else cfg.recovery_budget_s
        self._gap_search = False
        found = self.recover(sensors, radius_m=cfg.recovery_radius_m, accept=cfg.recovery_accept,
                             budget_s=budget)
        logging.info("Localization lost (alignment %.2f): searched %.1f m around the estimate in %.2f s, %s",
                     low, cfg.recovery_radius_m, time.monotonic() - started,
                     "found a fit %.1f m away" % math.hypot(self._x - before[0], self._y - before[1])
                     if found else "no fit good enough")
        if found:
            self.recoveries += 1
            self.low_scans = 0
            self._score(_HitsOnly(sensors, self.settings.scan_subsample))

    def resume(self):
        """Sensors came back after a gap: keep the estimate (the car did not teleport).

        The wheel odometry covers the gap too and its first update afterwards can be a large,
        wrong step, so the next update checks the odometry-moved pose against the old one, and a
        poor fit starts the recovery search at once (the car may have moved while blind).
        """
        self._after_gap = self._gap_search = True
        self._odom_time = None  # the time across the gap must not widen the acceleration limit
        self.low_scans = max(0, self.settings.lost_scans - 1)

    def _first_update_after_gap(self, sensors, before, predicted):
        """Keep whichever of the pre-gap pose and the odometry-moved pose fits the scan better."""
        self._after_gap = False
        if before is None or predicted is None or self._tree is None:
            return predicted
        try:
            scan = self._squash(sensors.lidar_sensor.to_base(sensors.lidar))
            if scan is None or len(scan) < 3:
                return predicted
            if self._alignment_at(scan, before) > self._alignment_at(scan, predicted):
                self._x, self._y, self._theta = before
                logging.info("Localization after a sensor gap: kept the pre-gap pose "
                             "(odometry moved it %.1f m and fit worse)",
                             math.hypot(predicted[0] - before[0], predicted[1] - before[1]))
                return before
        except Exception:  # Never stop localization over this check.
            pass
        return predicted

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

    def recover(self, sensors, radius_m=1.0, step_m=0.5, yaw_span=0.2, accept=0.6, budget_s=None):
        """Search around the estimate for a pose that fits the map better; True if adopted.

        Tries a grid of start poses (position within radius_m, yaw within yaw_span), runs ICP
        from each and keeps the best-aligned result. Ties go to the pose closest to the
        current estimate, so a featureless corridor does not move the car for nothing.
        Only a result aligned at least ``accept`` is adopted. Candidates are tried nearest first,
        and with ``budget_s`` the search stops after that many seconds (the control loop waits for it).
        """
        if self._tree is None or self._x is None:
            return False
        sensors = _HitsOnly(sensors, self.settings.scan_subsample)
        scan = self._squash(sensors.lidar_sensor.to_base(sensors.lidar))
        if scan is None or len(scan) < 3:
            return False
        offsets = np.arange(-radius_m, radius_m + 1e-9, step_m)
        candidates = sorted(((dx, dy, dyaw) for dyaw in (0.0, -yaw_span, yaw_span)
                             for dx in offsets for dy in offsets),
                            key=lambda c: (math.hypot(c[0], c[1]), abs(c[2])))
        best, best_key = None, None
        deadline = None if budget_s is None else time.monotonic() + budget_s
        for dx, dy, dyaw in candidates:
            if deadline is not None and best is not None and time.monotonic() > deadline:
                break
            x, y, theta = self._icp(scan, self._map, self._x + dx, self._y + dy, self._theta + dyaw)
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
