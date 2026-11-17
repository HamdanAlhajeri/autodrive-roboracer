"""SLAM Toolbox localization as the planned algorithm's pose source.

With ``localization.source: slam`` in ``config/avlite.yaml``, the planned controller no longer
reads the simulator's true pose. SLAM Toolbox localizes against a saved pose graph, using the
LiDAR and non-ground-truth wheel odometry (``avlite_autodrive.sim_odometry``). The car stack's
health monitor (``avlite_roboracer.localization``) decides when the estimate is good enough to
drive on.

Frames: SLAM poses are ``map -> base_link``. If ``map_origin_world`` gives the world pose where
the SLAM map was started (where sim_odometry began integrating during the mapping run), poses
are re-expressed in the simulator world frame, so a race map built in that frame still applies.
That is one fixed alignment constant, like surveying the start line, not live ground truth.
Ground truth is otherwise used only for the ``localization_error_m`` diagnostic.
"""

from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time

SOURCES = ("ground_truth", "slam")
# SLAM Toolbox only matches a scan after the car travels minimum_travel_distance or turns
# minimum_travel_heading. The health monitor needs a fresh SLAM pose before the bridge
# releases commands, so with the car parked at the start neither would ever happen. Zero
# thresholds make localization match every scan, rate-limited by minimum_time_interval.
# The car's mapping search (0.5 m window at 1 cm) took ~0.5 s per scan on one core beside
# the simulator, so poses arrived 2 s late. Localization starts from wheel odometry, so a
# 0.3 m window at 2.5 cm (~17x fewer candidates) is enough and keeps up in real time.
# Loop closure ties live scans to the saved graph; from a known start pose a 3 m window
# (not the mapping run's 8 m) is enough. Matching at 5 Hz is plenty with odometry between.
DEFAULT_SLAM_OVERRIDES = {"minimum_travel_distance": 0.0, "minimum_travel_heading": 0.0,
                          "correlation_search_space_dimension": 0.3,
                          "correlation_search_space_resolution": 0.025,
                          "loop_search_space_dimension": 3.0,
                          "minimum_time_interval": 0.2}
# SLAM reports a scan's pose and covariance after matching it, a few hundred ms later.
# The driven pose stays current (map -> odom composed with live odometry); the covariance
# describing it may be this old before the estimate counts as unreported.
COVARIANCE_MAX_AGE_S = 1.0


@dataclass(frozen=True)
class LocalizationConfig:
    profile: Path            # sim_profile.yaml (frames, topics, thresholds)
    pose_graph: Path         # serialized pose graph, without extension
    map_yaml: Path           # map_saver YAML of the same map, for scan alignment
    slam_params: Path        # SLAM Toolbox base parameters
    start_pose: tuple        # (x, y, yaw) of the car at startup, in the SLAM map frame
    map_origin_world: tuple = None  # world pose of the SLAM map origin, if known
    log_dir: Path = Path("/tmp/avlite_localization")
    slam_overrides: tuple = tuple(DEFAULT_SLAM_OVERRIDES.items())  # (name, value) pairs


def _overrides(value):
    """Merge localization.slam_overrides over the defaults; values must be scalars."""
    if value is None:
        value = {}
    if not isinstance(value, dict) or not all(
            isinstance(k, str) and isinstance(v, (bool, int, float, str))
            and not (isinstance(v, float) and not math.isfinite(v))
            for k, v in value.items()):
        raise ValueError("localization.slam_overrides must map parameter names to scalars")
    return tuple({**DEFAULT_SLAM_OVERRIDES, **value}.items())


def _pose(value, name):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       and math.isfinite(v) for v in value)):
        raise ValueError(f"localization.{name} must be [x, y, yaw] with finite numbers")
    return tuple(float(v) for v in value)


def localization_config(config, config_path):
    """Return a LocalizationConfig for ``source: slam``, or None for ground truth.

    Relative paths are resolved against the directory of the config file.
    """
    block = config.get("localization") or {}
    source = block.get("source", "ground_truth")
    if source not in SOURCES:
        raise ValueError(f"localization.source must be one of {', '.join(SOURCES)}")
    if source == "ground_truth":
        return None
    base = Path(config_path).parent

    def path(name, default=None):
        value = block.get(name, default)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"localization.{name} must be a file path for source: slam")
        return (base / value).resolve()

    origin = block.get("map_origin_world")
    return LocalizationConfig(
        profile=path("profile", "roboracer/sim_profile.yaml"),
        pose_graph=path("pose_graph"),
        map_yaml=path("map_yaml"),
        slam_params=path("slam_params", "roboracer/slam_toolbox.yaml"),
        start_pose=_pose(block.get("start_pose", [0.0, 0.0, 0.0]), "start_pose"),
        map_origin_world=None if origin is None else _pose(origin, "map_origin_world"),
        log_dir=Path(block.get("log_dir", "/tmp/avlite_localization")),
        slam_overrides=_overrides(block.get("slam_overrides")),
    )


def apply_slam_overrides(params_file, overrides):
    """Rewrite an effective SLAM Toolbox parameter file with the given overrides."""
    import yaml
    path = Path(params_file)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    data["slam_toolbox"]["ros__parameters"].update(dict(overrides))
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def to_world(map_pose, origin):
    """Express an (x, y, yaw) SLAM map pose in the world frame, if the origin is known."""
    from avlite_roboracer.geometry import compose
    return tuple(map_pose) if origin is None else compose(origin, map_pose)


def to_map(world_pose, origin):
    """Express a world pose in the SLAM map frame, if the origin is known."""
    from avlite_roboracer.geometry import relative
    return tuple(world_pose) if origin is None else relative(origin, world_pose)


class SlamPoseSource:
    """Run SLAM Toolbox localization and report the car's pose and its health."""

    def __init__(self, node, settings):
        import rclpy.time
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener
        from avlite_roboracer.localization import LocalizationMonitor, LocalizationThresholds
        from avlite_roboracer.occupancy import OccupancyGrid
        from avlite_roboracer.profile import HardwareProfile
        from avlite_roboracer.slam import SlamProcess

        self.node, self.settings = node, settings
        self.rclpy_time = rclpy.time
        self.profile = HardwareProfile.load(settings.profile)
        self.frames = {k: self.profile.frame(k) for k in ("map", "odom", "base_link")}
        self.monitor = LocalizationMonitor(LocalizationThresholds.from_profile(self.profile))
        self.grid = OccupancyGrid.load(settings.map_yaml, self.frames["map"])
        self.lock = threading.Lock()
        self.covariance = None  # (stamp_s, (var_x, var_y, var_yaw))
        self.slam = SlamProcess(settings.log_dir)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.initial_pose = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        node.create_subscription(PoseWithCovarianceStamped, self.profile.topic("slam_pose"),
                                 self.on_slam_pose, 10)
        node.create_subscription(LaserScan, self.profile.topic("scan"), self.on_scan,
                                 qos_profile_sensor_data)

    # ------------------------------------------------------------- SLAM process
    def start(self):
        """Start SLAM Toolbox in localization mode on the saved pose graph."""
        from avlite_roboracer.slam import effective_parameters
        s = self.settings
        params = effective_parameters(s.slam_params, self.profile, "localization",
                                      s.log_dir / "slam_localization.yaml",
                                      map_file=s.pose_graph, start_pose=s.start_pose)
        apply_slam_overrides(params, s.slam_overrides)
        self.slam.start("localization", params)

    def stop(self):
        self.slam.stop()

    def reinitialize(self):
        """After a simulator reset the car is back at the start: tell SLAM and forget health."""
        from geometry_msgs.msg import PoseWithCovarianceStamped
        x, y, yaw = self.settings.start_pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frames["map"]
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (math.sin(yaw / 2),
                                                                    math.cos(yaw / 2))
        self.initial_pose.publish(msg)
        with self.lock:
            self.monitor.reset()

    # ---------------------------------------------------------------- callbacks
    def on_slam_pose(self, msg):
        c = msg.pose.covariance
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.covariance = (stamp, (c[0], c[7], c[35]))

    def on_scan(self, msg):
        """Score each scan against the map at the SLAM pose for the same instant."""
        from avlite_roboracer.localization import scan_alignment, scan_points
        stamp = self.rclpy_time.Time.from_msg(msg.header.stamp)
        map_pose, odom_pose = self.map_pose(stamp), self.lookup("odom", stamp)
        if map_pose is None or odom_pose is None:
            return
        points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                             msg.range_max)
        alignment, _ = scan_alignment(
            self.grid, points, map_pose[:3], self.profile.laser_mount,
            self.profile.get("localization.scan_alignment_tolerance_m"))
        scan_s = stamp.nanoseconds * 1e-9
        with self.lock:
            covariance = (self.covariance[1] if self.covariance
                          and 0 <= scan_s - self.covariance[0] <= COVARIANCE_MAX_AGE_S
                          else None)
            self.monitor.update(time.monotonic(), map_pose[:3], odom_pose[:3], covariance,
                                alignment)

    # ------------------------------------------------------------------ queries
    def lookup(self, target, stamp=None):
        """Return base_link in the target frame as (x, y, yaw, age_s), or None."""
        from avlite_roboracer.geometry import yaw_from_quaternion
        try:
            t = self.tf.lookup_transform(self.frames[target], self.frames["base_link"],
                                         stamp or self.rclpy_time.Time())
        except Exception:  # tf2 raises several lookup/extrapolation exception types
            return None
        q, p = t.transform.rotation, t.transform.translation
        age = (self.node.get_clock().now()
               - self.rclpy_time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w), age)
        return pose if all(math.isfinite(v) for v in pose) else None

    def map_pose(self, stamp=None):
        """base_link in the map frame at stamp (latest if None) as (x, y, yaw, age_s).

        SLAM Toolbox stamps map -> odom with the last matched scan plus transform_timeout,
        which trails live odometry by its matching latency, so a single tf lookup at a
        recent time fails (no extrapolation). map -> odom is a slowly changing correction:
        use its latest value with odom -> base_link at the requested time. The age is the
        odometry's; a stalled SLAM shows up as a missing covariance instead.
        """
        from avlite_roboracer.geometry import compose, yaw_from_quaternion
        odom_pose = self.lookup("odom", stamp)
        try:
            t = self.tf.lookup_transform(self.frames["map"], self.frames["odom"],
                                         self.rclpy_time.Time())
        except Exception:  # tf2 raises several lookup/extrapolation exception types
            return None
        if odom_pose is None:
            return None
        q, p = t.transform.rotation, t.transform.translation
        correction = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        pose = (*compose(correction, odom_pose[:3]), odom_pose[3])
        return pose if all(math.isfinite(v) for v in pose) else None

    def pose(self):
        """Latest (x, y, yaw, age_s) for the controller, in the race map's frame."""
        pose = self.map_pose()
        if pose is None:
            return None
        return (*to_world(pose[:3], self.settings.map_origin_world), pose[3])

    def failures(self):
        """Reasons the estimate must not be driven on; empty when healthy."""
        with self.lock:
            return self.monitor.failures(time.monotonic())

    def diagnostics(self, truth_world=None):
        """Numeric localization health for /avlite/controller_diagnostics."""
        with self.lock:
            snap = self.monitor.snapshot(time.monotonic())
        values = {"localization_ok": float(not snap["failures"]),
                  "localization_pose_age_s": snap.get("pose_age_s"),
                  "localization_scan_alignment": snap.get("scan_alignment"),
                  "localization_position_stddev_m": snap.get("position_stddev_m"),
                  "localization_motion_mismatch_m": snap.get("motion_mismatch_m")}
        pose = self.pose()
        # Without the map origin the SLAM pose is not in the world frame: no comparison.
        if (truth_world is not None and pose is not None
                and self.settings.map_origin_world is not None):
            values["localization_error_m"] = math.hypot(pose[0] - truth_world[0],
                                                        pose[1] - truth_world[1])
            values["localization_yaw_error_rad"] = abs(math.atan2(
                math.sin(pose[2] - truth_world[2]), math.cos(pose[2] - truth_world[2])))
        return values
