"""Jetson supervisor runtime (ROS 2): map one lap, finalize, wait for race-start, race.

    roboracer_supervisor --profile config/roboracer/hardware.yaml \
        --race-config config/roboracer/race.yaml --slam-params config/roboracer/slam_toolbox.yaml

Runs locally on the Jetson; the laptop only sends commands (avlite-roboracer over SSH)
and reads status. It owns:

- the Supervisor state machine and the local command socket;
- one SLAM Toolbox process (mapping, then localization against the saved pose graph);
- AVLite Follow the Gap for the mapping lap and AVLite Pure Pursuit for racing;
- the CommandArbiter that lets exactly one source command the actuator process.

It never resumes motion after a restart; see supervisor.py for the command rules.
"""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import threading
import time
import uuid

import numpy as np

from avlite_autodrive.configuration import load_config

from .command import CommandArbiter, CommandSource, SpeedDemand
from .lap_detector import LapDetector, LapSettings
from .localization import (
    LocalizationMonitor, LocalizationThresholds, scan_alignment, scan_points,
)
from .icp_pose import IcpPose, icp_settings, localization_source
from .map_builder import build_hardware_map
from .occupancy import OccupancyGrid
from .profile import HardwareProfile
from .protocol import CommandServer, default_socket_path, dispatch
from .racing import hardware_race_config, mapping_control_config, mapping_racing_config
from .readiness import actuator_failures, mapping_failures, race_failures
from .slam import SlamProcess, effective_parameters, require_save_success
from .profiling import StackSampler, TickStats
from .supervisor import Action, State, Supervisor

CONTROL_DT = 0.05


# How far ahead of this computer's clock a sensor stamp may be. The Hokuyo stamps its scans
# about 1 ms ahead of the Jetson clock, so a strict "age must not be negative" test rejected
# 96 % of them and the stale-LiDAR stop fired at every mapping start (23 Nov). The upper limit
# (the sensor timeout) is not loosened, and a stamp further ahead than this is still refused.
CLOCK_TOLERANCE_S = 0.05


def sensor_stamp_is_fresh(stamp_ns, previous_ns, now_ns, timeout_s):
    """True for a newer-than-previous stamp that is at most timeout_s old at now_ns.

    A stamp up to CLOCK_TOLERANCE_S ahead of now_ns counts as brand new (clock jitter).
    """
    return (stamp_ns > 0 and (previous_ns is None or stamp_ns > previous_ns)
            and -CLOCK_TOLERANCE_S <= (now_ns - stamp_ns) * 1e-9 <= timeout_s)


class FinalizeCancelled(Exception):
    pass


def apply_control_settings(control):
    """Set AVLite's class-level control settings before constructing a controller."""
    from avlite.c30_control.c39_settings import ControlSettings
    for key, value in control.items():
        if not hasattr(ControlSettings, key):
            raise ValueError("Unknown AVLite control setting: " + key)
        setattr(ControlSettings, key, value)


class Runtime:
    def __init__(self, node, profile, race_config_path, slam_params, data_dir, socket_path):
        from ackermann_msgs.msg import AckermannDriveStamped
        from nav_msgs.msg import OccupancyGrid as GridMsg, Odometry
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        from visualization_msgs.msg import MarkerArray

        self.node = node
        self.profile = profile
        self.race_config_path = Path(race_config_path)
        self.race_config = load_config(race_config_path)
        # Validate now so a bad lap count fails at startup, not at map-start.
        mapping = self.race_config.get("mapping") or {}
        self.lap_settings = LapSettings(laps=mapping.get("laps", 1))
        # icp: AVLite ICP on the saved map owns the map-frame pose after the mapping lap.
        # slam: SLAM Toolbox localization does, as before.
        self.localization_source = localization_source(self.race_config)
        self.icp_settings = icp_settings(self.race_config)
        self.icp = None
        self.icp_trace = None
        self.icp_trace_name = None
        self.slam_params = Path(slam_params)
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.frames = {k: profile.frame(k) for k in ("map", "odom", "base_link")}
        self.lock = threading.RLock()
        self.Drive = AckermannDriveStamped
        self.String = String

        self.scan = None
        self.scan_time = self.odom_time = -math.inf
        self.last_scan_stamp = self.last_odom_stamp = None
        self.reset_scan_stats()
        self.tick_stats = TickStats()
        self.sampler = StackSampler(threading.get_ident())
        self.sampler.start()
        self.pending_scans = deque(maxlen=100)
        self.odom_speed = 0.0
        self.slam_pose = None
        self.graph_nodes = []
        self.graph_time = -math.inf
        self.map_msg = None
        self.actuator = None
        self.slam_nodes = 0
        self.slam_stopped_at = -math.inf
        self.last_graph_check = -math.inf

        self.slam = SlamProcess(self.data_dir / "logs",
                                nice=mapping.get("slam_nice", 0))
        self.arbiter = CommandArbiter(profile.get("timing.command_timeout_s"))
        self.speed = SpeedDemand(0.0)
        self.lap = LapDetector()
        self.lap_started = False
        self.lap_rejections_logged = 0
        self.mapping_session = None  # the session start_mapping last set up
        self.stopping_after_lap = False
        self.stopped_since = None
        self._pose_seen = {}  # frame -> (latest transform stamp, monotonic time it first arrived)
        self.mapper = None
        self.racer = None
        self.prepared = None
        self.grid = None
        self.session_dir = None
        self.finalize_thread = None
        self.finalize_cancel = threading.Event()
        self.finalize_step = None
        self.pose_invalid_since = None
        try:
            self.monitor = LocalizationMonitor(
                LocalizationThresholds.from_profile(profile),
                covariance_required=self.localization_source == "slam")
        except ValueError as exc:
            self.monitor = None
            node.get_logger().warning(f"{exc}; autonomy disabled until measured")

        previous = None
        state_file = self.data_dir / "state.json"
        if state_file.exists():
            try:
                previous = json.loads(state_file.read_text())
            except ValueError:
                previous = {"state": "unreadable"}
        self.supervisor = Supervisor(uuid.uuid4().hex[:8],
                                     profile.get("timing.authorization_timeout_s"),
                                     self.race_readiness, self.mapping_readiness, previous)
        self._persisted = None
        self._persist()

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, node)
        node.create_subscription(LaserScan, profile.topic("scan"), self.on_scan,
                                 qos_profile_sensor_data)
        node.create_subscription(Odometry, profile.topic("odom"), self.on_odom,
                                 qos_profile_sensor_data)
        node.create_subscription(String, "/roboracer/actuator_status", self.on_actuator, 10)
        node.create_subscription(PoseWithCovarianceStamped, profile.topic("slam_pose"),
                                 self.on_slam_pose, 10)
        node.create_subscription(MarkerArray, profile.topic("slam_graph"), self.on_graph, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(GridMsg, "/map", self.on_map, latched)
        self.command_pub = node.create_publisher(AckermannDriveStamped, "/roboracer/command", 1)
        self.status_pub = node.create_publisher(String, "/roboracer/status", 1)
        self.plan_pub = node.create_publisher(String, "/roboracer/race_plan", latched)

        from rclpy.clock import Clock, ClockType
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(CONTROL_DT, self.tick, clock=steady)
        node.create_timer(0.5, self.publish_status, clock=steady)
        self.server = CommandServer(socket_path, self.handle_request)
        self.server.start()
        node.get_logger().info(f"Supervisor ready on {socket_path}; session "
                               f"{self.supervisor.session}")

    # -------------------------------------------------------------- callbacks
    @property
    def lap_active(self):
        """True once this session's mapping lap has started.

        tick() checks faults before it runs the start action that resets the lap state, so a
        second mapping session on the same supervisor still saw the previous session's
        lap_started flag and was faulted for a missing map pose before it had begun (23 Nov).
        """
        return bool(self.lap_started
                    and getattr(self, "mapping_session", None) == self.supervisor.session)

    def reset_scan_stats(self):
        """Counters for the stale-LiDAR diagnosis: scans seen, rejected, and header-stamp lag."""
        self.scan_stats = {"seen": 0, "rejected": 0, "lag_last_s": None, "lag_max_s": 0.0,
                           "gap_max_s": 0.0}
        self.odom_stats = {"seen": 0, "rejected": 0, "lag_last_s": None, "lag_max_s": 0.0,
                           "gap_max_s": 0.0}

    def scan_stats_text(self):
        stats = getattr(self, "scan_stats", None)
        if not stats or not stats["seen"]:
            return "no scans seen since the session started"
        return (f"scans seen {stats['seen']}, rejected as stale or out of order "
                f"{stats['rejected']}, stamp lag last {stats['lag_last_s']:.3f} s "
                f"max {stats['lag_max_s']:.3f} s, longest gap between accepted scans "
                f"{stats.get('gap_max_s', 0.0):.3f} s")

    def odom_stats_text(self):
        stats = getattr(self, "odom_stats", None)
        if not stats or not stats["seen"]:
            return "no odometry seen since the session started"
        return (f"odometry seen {stats['seen']}, rejected as stale or out of order "
                f"{stats['rejected']}, stamp lag last {stats['lag_last_s']:.3f} s "
                f"max {stats['lag_max_s']:.3f} s, longest gap between accepted messages "
                f"{stats['gap_max_s']:.3f} s")

    def on_scan(self, msg):
        with self.lock:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            now_ns = self.node.get_clock().now().nanoseconds
            lag = (now_ns - stamp) * 1e-9
            self.scan_stats["seen"] += 1
            self.scan_stats["lag_last_s"] = lag
            self.scan_stats["lag_max_s"] = max(self.scan_stats["lag_max_s"], lag)
            if not sensor_stamp_is_fresh(stamp, self.last_scan_stamp, now_ns,
                                         self.profile.get("timing.sensor_timeout_s")):
                self.scan_stats["rejected"] += 1
                return
            self.last_scan_stamp = stamp
            self.scan = msg
            arrived = time.monotonic()
            if math.isfinite(self.scan_time) and getattr(self, "scan_stats", None):
                self.scan_stats["gap_max_s"] = max(self.scan_stats["gap_max_s"],
                                                   arrived - self.scan_time)
            self.scan_time = arrived
            self.pending_scans.append((self.scan_time, msg))

    def on_odom(self, msg):
        speed = msg.twist.twist.linear.x
        with self.lock:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            now_ns = self.node.get_clock().now().nanoseconds
            stats = getattr(self, "odom_stats", None)
            if stats is not None:
                lag = (now_ns - stamp) * 1e-9
                stats["seen"] += 1
                stats["lag_last_s"] = lag
                stats["lag_max_s"] = max(stats["lag_max_s"], lag)
            if not math.isfinite(speed) or not sensor_stamp_is_fresh(
                    stamp, self.last_odom_stamp, now_ns,
                    self.profile.get("timing.sensor_timeout_s")):
                if stats is not None:
                    stats["rejected"] += 1
                return
            self.last_odom_stamp = stamp
            self.odom_speed = speed
            arrived = time.monotonic()
            if math.isfinite(self.odom_time) and stats is not None:
                stats["gap_max_s"] = max(stats["gap_max_s"], arrived - self.odom_time)
            self.odom_time = arrived

    def on_actuator(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(status, dict):
            return
        with self.lock:
            self.actuator = (time.monotonic(), status)

    def on_slam_pose(self, msg):
        c = msg.pose.covariance
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.slam_pose = (stamp, [c[0], c[7], c[35]])

    def on_graph(self, msg):
        nodes = sorted([m.id, m.pose.position.x, m.pose.position.y]
                       for m in msg.markers if m.type == m.SPHERE)
        if nodes:
            with self.lock:
                self.graph_nodes = nodes
                self.graph_time = time.monotonic()

    def on_map(self, msg):
        with self.lock:
            self.map_msg = msg

    def handle_request(self, request):
        with self.lock:
            reply = dispatch(self.supervisor, request, time.monotonic(), self.status_extra)
        if (isinstance(request, dict) and reply.get("ok")
                and request.get("command") in ("map-start", "map-load", "race-start", "stop")):
            self.node.get_logger().info(f"{request['command']}: {reply.get('message')}")
        return reply

    # ------------------------------------------------------------ measurements
    def ages(self, now):
        return now - self.scan_time, now - self.odom_time

    def pose(self, target, stamp=None):
        """Return base_link in target ('map' or 'odom') as (x, y, yaw, age_s), or None."""
        if target == "map" and self.icp is not None:
            pose = self.icp.pose  # ICP owns the map frame once finalization has seeded it
            return None if pose is None else (*pose, self.icp.age(time.monotonic()))
        import rclpy.time
        from .geometry import yaw_from_quaternion
        try:
            t = self.tf.lookup_transform(self.frames[target], self.frames["base_link"],
                                         stamp or rclpy.time.Time())
        except Exception:  # tf2 raises several lookup/extrapolation exception types
            return None
        q, p = t.transform.rotation, t.transform.translation
        age = (self.node.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds
        age_s = age * 1e-9
        if stamp is None:
            # Latest pose: freshness is the monotonic time since a new transform arrived. A
            # wall-clock step (NTP, a VM clock correction) or SLAM Toolbox stamping map -> odom
            # slightly ahead would otherwise read as a negative or huge age and stop the car.
            stamp_ns = rclpy.time.Time.from_msg(t.header.stamp).nanoseconds
            seen = self._pose_seen.get(target)
            if seen is None or seen[0] != stamp_ns:
                seen = self._pose_seen[target] = (stamp_ns, time.monotonic())
            age_s = time.monotonic() - seen[1]
        pose = p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w), age_s
        return pose if all(math.isfinite(v) for v in pose) else None

    def stop_slam(self):
        """Stop our SLAM process and remember when, so its lingering graph entry is ours."""
        self.slam.stop()
        self.slam_stopped_at = time.monotonic()

    def foreign_slam_nodes(self):
        """slam_toolbox nodes in the ROS graph that this runtime did not start.

        A node we just stopped stays in the graph for a few seconds until DDS notices it is
        gone; counting it made READY fail with "found 2" right after ICP started.
        """
        lingering = 1 if (not self.slam.running
                          and time.monotonic() - self.slam_stopped_at < 15.0) else 0
        return max(0, self.slam_nodes - (1 if self.slam.running else lingering))

    def localization_processes(self):
        """Processes that could publish map -> odom; READY needs exactly our one."""
        own = 1 if (self.slam.running and self.slam.mode == "localization"
                    or self.icp is not None) else 0
        return own + self.foreign_slam_nodes() if own else 0

    def race_readiness(self, now):
        scan_age, odom_age = self.ages(now)
        localization = (self.monitor.failures(now) if self.monitor and self.grid is not None
                        else ["no localization against a finalized map"])
        return race_failures(self.profile, scan_age, odom_age, self.actuator, localization,
                             self.prepared is not None, self.localization_processes(), now)

    def mapping_readiness(self, now):
        scan_age, odom_age = self.ages(now)
        # Our own localization process is replaced by map-start; anything else blocks it.
        failures = mapping_failures(self.profile, scan_age, odom_age, self.actuator,
                                    self.foreign_slam_nodes() > 0, now)
        if self.monitor is None and not any("not commissioned" in f for f in failures):
            failures.append("localization thresholds not established")
        if self.finalize_thread is not None and self.finalize_thread.is_alive():
            failures.append("previous finalization is still stopping; retry after it exits")
        return failures

    def status_extra(self):
        now = time.monotonic()
        scan_age, odom_age = self.ages(now)
        return {
            "localization": self.monitor.snapshot(now) if self.monitor else None,
            "mapping_lap": self.lap.summary() if self.lap_active else None,
            "finalize_step": self.finalize_step,
            "localizer": self.localization_source,
            "icp_alignment": self.icp.alignment if self.icp is not None else None,
            "icp_recoveries": self.icp.recoveries if self.icp is not None else None,
            "slam": {"mode": self.slam.mode, "running": self.slam.running,
                     "slam_toolbox_nodes": self.slam_nodes},
            "actuator": self.actuator[1] if self.actuator else None,
            "sensor_age_s": {"scan": scan_age if math.isfinite(scan_age) else None,
                             "odom": odom_age if math.isfinite(odom_age) else None},
            "command_owner": self.arbiter.owner.value,
            "profile": self.profile.summary(),
            "session_dir": str(self.session_dir) if self.session_dir else None,
        }

    # ----------------------------------------------------------------- control
    def tick(self):
        now = time.monotonic()
        marks = {}
        with self.lock:
            if now - self.last_graph_check > 1.0:
                self.last_graph_check = now
                self.slam_nodes = sum(name == "slam_toolbox"
                                      for name in self.node.get_node_names())
            while self.pending_scans:
                received, scan = self.pending_scans[0]
                if now - received > self.profile.get("timing.sensor_timeout_s"):
                    self.pending_scans.popleft()
                elif self.on_new_scan(received, scan):
                    self.pending_scans.popleft()
                else:
                    break  # asynchronous SLAM may provide this scan's TF on the next tick
            marks["scans"] = time.monotonic()
            self.supervisor.tick(now)
            marks["supervisor"] = time.monotonic()
            self.check_faults(now)
            marks["faults"] = time.monotonic()
            self.run_actions(now)
            actions_done = marks["actions"] = time.monotonic()
            try:
                command = self.control(now)
                elapsed = time.monotonic() - now
                if elapsed > 0.5 * self.profile.get("timing.sensor_timeout_s"):
                    # Callbacks wait behind this tick, so a slow step can starve sensors.
                    self.node.get_logger().warning(
                        f"Control tick took {elapsed:.3f} s in {self.supervisor.state.value} "
                        f"(scans and actions {actions_done - now:.3f} s)")
            except Exception as exc:  # any controller failure must stop, not crash
                self.node.get_logger().error(f"Controller failed: {exc!r}")
                self.supervisor.fault(f"controller error: {exc}", now)
                self.run_actions(now)
                command = None
            marks["control"] = time.monotonic()
            self.run_actions(now)  # control may have stopped or finalized the session
            if command is not None and self.supervisor.state in (State.MAPPING, State.RACING):
                self.arbiter.submit(*command, now)
            out = self.arbiter.output(now)
            self._persist()
            if self.supervisor.state == State.MAPPING:
                self.tick_stats.record(now, dict(marks, publish=time.monotonic()))
            elif self.sampler.active:
                self.dump_timing_report()
        msg = self.Drive()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = out.source.value
        msg.drive.steering_angle = out.steering_rad
        msg.drive.speed = out.speed_mps
        self.command_pub.publish(msg)

    def dump_timing_report(self):
        """Mapping has ended: save where the supervisor thread spent its time, then switch off."""
        self.sampler.active = False
        report = f"tick timing: {self.tick_stats.text()}\n{self.sampler.report()}\n"
        self.node.get_logger().info(report)
        if self.session_dir is not None:
            try:
                (self.session_dir / "timing-report.txt").write_text(report, encoding="utf-8")
            except OSError:
                pass

    def log_icp(self, stamp_s, map_pose):
        """Append one map-frame pose to icp_trace.csv (racing) or slam_trace.csv (mapping).

        Compare it with the true path after a test lap. Logging must never stop the car.
        """
        if map_pose is None or self.session_dir is None:
            return
        try:
            name = "icp_trace.csv" if self.icp is not None else "slam_trace.csv"
            if self.icp_trace is not None and self.icp_trace_name != name:
                self.icp_trace.close()  # mapping ended and ICP took over: start the racing file
                self.icp_trace = None
            if self.icp_trace is None:
                self.icp_trace = open(self.session_dir / name, "a", buffering=1)
                self.icp_trace_name = name
                self.icp_trace.write("stamp_s,x,y,yaw,alignment,recoveries\n")
            alignment = self.icp.alignment if self.icp is not None else None
            recoveries = self.icp.recoveries if self.icp is not None else 0
            self.icp_trace.write("%.3f,%.4f,%.4f,%.4f,%s,%d\n" % (
                stamp_s, map_pose[0], map_pose[1], map_pose[2],
                "" if alignment is None else "%.3f" % alignment, recoveries))
        except OSError as exc:
            self.node.get_logger().warning(f"ICP trace not written: {exc}")
            self.icp_trace = None

    def on_new_scan(self, now, scan):
        """Update localization health and the lap detector from the newest scan."""
        import rclpy.time
        stamp = rclpy.time.Time.from_msg(scan.header.stamp)
        odom_pose = self.pose("odom", stamp)
        if self.icp is not None:
            if odom_pose is None:
                return False
            # Align this scan to the saved map, starting from the odometry-predicted pose.
            self.icp.update(self.sensor_frame(now, scan), odom_pose[:3], self.odom_speed, now)
            map_pose = self.pose("map")
            self.log_icp(stamp.nanoseconds / 1e9, map_pose)
        else:
            map_pose = self.pose("map", stamp)
            self.log_icp(stamp.nanoseconds / 1e9, map_pose)
        if map_pose is None or odom_pose is None:
            return False
        state = self.supervisor.state
        if state == State.MAPPING and self.lap_active:
            result = self.lap.update(map_pose[:3], odom_pose[:3], list(scan.ranges))
            self.log_lap_progress(map_pose)
            if result == "complete":
                self.supervisor.lap_completed(now)
            elif result == "failed":
                self.supervisor.fault("mapping lap not confirmed within the travel limit", now)
        if self.monitor is not None and self.grid is not None:
            s = scan
            points = scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                 s.range_max)
            alignment, _ = scan_alignment(
                self.grid, points, map_pose[:3], self.profile.laser_mount,
                self.profile.get("localization.scan_alignment_tolerance_m"))
            covariance = None
            scan_stamp = stamp.nanoseconds * 1e-9
            if self.slam_pose is not None and abs(self.slam_pose[0] - scan_stamp) <= 0.2:
                covariance = self.slam_pose[1]
            self.monitor.update(now, map_pose[:3], odom_pose[:3], covariance, alignment)
        return True

    def check_faults(self, now):
        state = self.supervisor.state
        if state in (State.IDLE, State.STOPPED):
            return
        scan_age, odom_age = self.ages(now)
        timeout = self.profile.get("timing.sensor_timeout_s")
        # Releasing the dead-man ends a moving session; a new start is then required.
        actuator = actuator_failures(self.actuator, now,
                                     2 * self.profile.get("timing.command_timeout_s"),
                                     require_deadman=state in (State.MAPPING, State.RACING))
        if self.slam.died:
            self.supervisor.fault("SLAM Toolbox process exited", now)
        elif state in (State.MAPPING, State.RACING, State.READY, State.FINALIZING):
            if not (0 <= scan_age <= timeout and 0 <= odom_age <= timeout):
                self.supervisor.fault(f"stale LiDAR or odometry (scan {scan_age:.3f} s, "
                                      f"odometry {odom_age:.3f} s, limit {timeout} s; "
                                      f"{self.scan_stats_text()}; {self.odom_stats_text()}; ticks: "
                                      f"{self.tick_stats.text()})", now)
            elif actuator:
                self.supervisor.fault("; ".join(actuator), now)
            elif state in (State.RACING, State.READY):
                failures = [f for f in self.monitor.failures(now) if "stable for only" not in f]
                if failures:
                    self.supervisor.fault("lost localization: " + "; ".join(failures), now)
            elif state == State.MAPPING and self.lap_active:
                poses = (self.pose("map"), self.pose("odom"))
                if any(p is None or not 0 <= p[3] <= self.profile.get("timing.pose_timeout_s")
                       for p in poses):
                    detail = ", ".join(
                        f"{name} " + ("missing" if p is None else f"{p[3]:.3f} s old")
                        for name, p in zip(("map pose", "odom pose"), poses))
                    self.supervisor.fault(f"mapping pose stale ({detail}; limit "
                                          f"{self.profile.get('timing.pose_timeout_s')} s)", now)
            elif state == State.FINALIZING and abs(self.odom_speed) > 0.1:
                self.supervisor.fault("car moved during finalization", now)

    def run_actions(self, now):
        for action in self.supervisor.take_actions():
            if action == Action.START_MAPPING:
                try:
                    self.start_mapping(now)
                except Exception as exc:
                    self.node.get_logger().error(f"Mapping start failed: {exc!r}")
                    self.arbiter.release()
                    self.stop_slam()
                    self.supervisor.fault(f"mapping start failed: {exc}", now)
            elif action == Action.STOP_AFTER_LAP:
                self.stopping_after_lap = True
                self.stopped_since = None
            elif action == Action.FINALIZE:
                self.arbiter.release()
                self.finalize_cancel = threading.Event()
                self.finalize_thread = threading.Thread(
                    target=self.finalize, args=(self.finalize_cancel, self.supervisor.session),
                    daemon=True)
                self.finalize_thread.start()
            elif action == Action.LOAD_MAP:
                self.arbiter.release()
                self.finalize_cancel = threading.Event()
                self.finalize_thread = threading.Thread(
                    target=self.finalize, args=(self.finalize_cancel, self.supervisor.session,
                                                self.supervisor.load_request),
                    daemon=True)
                self.finalize_thread.start()
            elif action == Action.START_RACING:
                self.racer.reset()
                self.speed = SpeedDemand(self.prepared.settings.speed_limit)
                self.pose_invalid_since = None
                self.arbiter.select(CommandSource.RACING)
            elif action == Action.STOP_MOTION:
                self.arbiter.release()
                self.speed.reset()
                if self.slam.mode == "mapping":
                    self.rescue_and_stop_mapping()
            elif action == Action.ABORT_FINALIZE:
                # The worker checks this under the same lock before switching processes.
                self.finalize_cancel.set()
                self.stop_slam()
                self.grid = self.icp = None

    def log_lap_progress(self, map_pose):
        """Say why the finish line did not count (25 Nov: six laps, none confirmed, no trace)."""
        crossings = self.lap.rejected_crossings
        for reason in crossings[self.lap_rejections_logged:]:
            self.node.get_logger().warning(
                f"Finish line crossed but not counted: {reason} (map pose x={map_pose[0]:.2f} "
                f"y={map_pose[1]:.2f} yaw={map_pose[2]:.2f}; lap travel "
                f"{self.lap.lap_travel_m:.1f} m, laps done {self.lap.laps_completed})")
        self.lap_rejections_logged = len(crossings)

    def rescue_and_stop_mapping(self):
        """A mapping session ended before its lap was confirmed: keep what SLAM has built.

        Stopping SLAM Toolbox threw the map of a 261 s lap away on 25 Nov. Save the pose graph and
        the occupancy grid next to the session first, in a thread so the supervisor stays
        responsive, then stop that same SLAM process.
        """
        process = self.slam.process
        try:
            if self.lap_started and self.lap.start_pose is not None:
                (self.session_dir / "lap.json").write_text(
                    json.dumps(self.lap.summary(), default=str, indent=2))
        except OSError as exc:
            self.node.get_logger().warning(f"Could not write lap.json: {exc!r}")
        threading.Thread(target=self._rescue_map, args=(process, self.session_dir),
                         daemon=True).start()

    def _rescue_map(self, process, session_dir):
        cancel = threading.Event()
        try:
            from slam_toolbox.srv import SaveMap, SerializePoseGraph
            self._call(cancel, SerializePoseGraph, "/slam_toolbox/serialize_map",
                       SerializePoseGraph.Request(filename=str(session_dir / "posegraph-partial")),
                       timeout_s=8.0)
            request = SaveMap.Request()
            request.name.data = str(session_dir / "map-partial")
            self._call(cancel, SaveMap, "/slam_toolbox/save_map", request, timeout_s=8.0)
            self.node.get_logger().info(f"Saved the unfinished map to {session_dir}/map-partial*")
        except Exception as exc:  # best effort: the car is already stopped
            self.node.get_logger().warning(f"Could not save the unfinished map: {exc!r}")
        finally:
            with self.lock:
                if self.slam.process is process:  # not replaced by a newer SLAM process
                    self.stop_slam()

    def start_mapping(self, now):
        began = time.monotonic()
        from avlite_autodrive.plugin.controller import AutoDRIVEFollowTheGap
        self.stop_slam()  # a previous session's localization process
        self.prepared = self.racer = self.grid = self.icp = None
        self.graph_nodes = []
        self.graph_time = -math.inf
        self.map_msg = self.slam_pose = None
        self.pending_scans.clear()
        self.reset_scan_stats()
        self.tick_stats.reset()
        self.sampler.reset()
        self.sampler.active = True
        self.tf.clear()
        self._pose_seen.clear()
        if self.monitor is not None:
            self.monitor.reset()
        self.plan_pub.publish(self.String(data="{}"))
        self.session_dir = self.data_dir / "sessions" / self.supervisor.session
        self.session_dir.mkdir(parents=True, exist_ok=True)
        if self.icp_trace is not None:
            self.icp_trace.close()
        self.icp_trace = None
        apply_control_settings(mapping_control_config(self.race_config, self.profile))
        self.mapper = AutoDRIVEFollowTheGap(
            racing=mapping_racing_config(self.race_config, self.profile))
        self.speed = SpeedDemand(self.profile.get("limits.mapping_speed_mps"))
        self.lap = LapDetector(self.lap_settings)
        self.lap_started = False
        self.lap_rejections_logged = 0
        self.mapping_session = self.supervisor.session
        self.stopping_after_lap = False
        params = effective_parameters(self.slam_params, self.profile, "mapping",
                                      self.session_dir / "slam_mapping.yaml")
        prepared_s = time.monotonic() - began
        self.slam.start("mapping", params)
        self.arbiter.select(CommandSource.MAPPING)
        total_s = time.monotonic() - began
        # Scans are not processed while this runs; a slow start shows up here first.
        self.node.get_logger().info(
            f"mapping start took {total_s:.3f} s (setup {prepared_s:.3f} s, SLAM launch "
            f"{total_s - prepared_s:.3f} s)")

    def sensor_frame(self, now, scan=None):
        from avlite.c50_common.c52_world_sensor_datatypes import Lidar
        from avlite_autodrive.plugin.planned_controller import AutoDRIVESensorFrame
        from avlite_autodrive.sensors import scan_cloud, scan_hit_mask
        x, y, yaw = self.profile.laser_mount
        mount = np.eye(4)
        mount[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        mount[:2, 3] = [x, y]
        scan_age, odom_age = self.ages(now)
        scan = self.scan if scan is None else scan
        return AutoDRIVESensorFrame(
            lidar=scan_cloud(scan), lidar_sensor=Lidar(base_to_sensor=mount),
            frame_id=self.frames["base_link"], lidar_hit_mask=scan_hit_mask(scan),
            lidar_age_s=scan_age, odom_age_s=odom_age)

    def control(self, now):
        """Return (source, steering, speed) for the owner, or None to let commands expire."""
        from avlite.c10_perception.c11_perception_model import EgoState
        state = self.supervisor.state
        if state == State.MAPPING and self.scan is not None:
            if not self.lap_started:
                map_pose, odom_pose = self.pose("map"), self.pose("odom")
                fresh = (map_pose is not None and odom_pose is not None
                         and all(0 <= p[3] <= self.profile.get("timing.pose_timeout_s")
                                 for p in (map_pose, odom_pose)))
                if not fresh:
                    return CommandSource.MAPPING, 0.0, 0.0  # wait for SLAM's first pose
                self.lap.start(map_pose[:3], odom_pose[:3], list(self.scan.ranges))
                self.lap_started = True
            cmd = self.mapper.control(EgoState(x=0.0, y=0.0, theta=0.0,
                                               velocity=self.odom_speed),
                                      None, CONTROL_DT, None, self.sensor_frame(now))
            if self.stopping_after_lap:
                braking = self.profile.get("measurements.braking_deceleration_mps2")
                speed = self.speed.update(-0.5 * braking, self.odom_speed, CONTROL_DT)
                if abs(self.odom_speed) < 0.05 and speed == 0.0:
                    self.stopped_since = self.stopped_since or now
                    if now - self.stopped_since >= 1.0:
                        self.supervisor.vehicle_stopped(now)
                else:
                    self.stopped_since = None
            else:
                speed = self.speed.update(cmd.acceleration, self.odom_speed, CONTROL_DT)
            return CommandSource.MAPPING, float(cmd.steer), speed
        if state == State.RACING:
            pose = self.pose("map")
            if pose is None or not 0 <= pose[3] <= self.profile.get("timing.pose_timeout_s"):
                self.supervisor.fault("localization pose stale", now)
                return None
            ego = EgoState(x=pose[0], y=pose[1], theta=pose[2], velocity=self.odom_speed)
            cmd = self.racer.control(ego, self.prepared.global_plan, CONTROL_DT, None,
                                     self.sensor_frame(now))
            # Reason 3: pose outside the corridor or facing the wrong way.
            if self.racer.diagnostics.get("speed_limit_reason") == 3:
                self.pose_invalid_since = self.pose_invalid_since or now
                if now - self.pose_invalid_since > 0.5:
                    self.supervisor.fault("car is outside the validated corridor", now)
            else:
                self.pose_invalid_since = None
            speed = self.speed.update(cmd.acceleration, self.odom_speed, CONTROL_DT)
            return CommandSource.RACING, float(cmd.steer), speed
        return None

    # ------------------------------------------------------------ finalization
    def finalize(self, cancel, session, load_request=None):
        """Optimize, save, build and validate the map and plan, then switch to localization.

        With load_request, nothing is mapped: a saved map is loaded and the car localizes on it.
        """
        try:
            plan = (self._load_steps(cancel, load_request) if load_request
                    else self._finalize_steps(cancel))
        except FinalizeCancelled:
            return
        except Exception as exc:  # any worker failure must leave FINALIZING
            self.node.get_logger().error(f"Finalization failed: {exc}")
            with self.lock:
                if self.supervisor.session == session:
                    self.stop_slam()
                    self.supervisor.finalize_failed([str(exc)], time.monotonic())
            return
        with self.lock:
            if not cancel.is_set() and self.supervisor.session == session:
                failures = self.race_readiness(time.monotonic())
                if failures:
                    self.stop_slam()
                    self.supervisor.finalize_failed(failures, time.monotonic())
                else:
                    self.supervisor.finalize_succeeded(plan, time.monotonic())
                    self.finalize_step = "complete"

    def _step(self, cancel, name):
        if cancel.is_set():
            raise FinalizeCancelled
        self.finalize_step = name
        self.node.get_logger().info(f"Finalize: {name}")

    def _wait(self, cancel, condition, timeout_s, message, period_s=0.2):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise FinalizeCancelled
            result = condition()
            if result:
                return result
            time.sleep(period_s)
        raise TimeoutError(message)

    def _call_retry(self, cancel, srv_type, name, request, tries=3, delay_s=2.0):
        """_call again after a pause: save_map fails now and then with result 255 (map_saver
        could not get the map subscription) and works on the next try."""
        for attempt in range(1, tries + 1):
            try:
                return self._call(cancel, srv_type, name, request)
            except RuntimeError as exc:
                if attempt == tries:
                    raise
                self.node.get_logger().warning(f"{exc}; retrying ({attempt}/{tries - 1})")
                if cancel.wait(delay_s):
                    raise FinalizeCancelled

    def _call(self, cancel, srv_type, name, request, timeout_s=30.0):
        client = self.node.create_client(srv_type, name)
        try:
            self._wait(cancel, lambda: client.service_is_ready(), 10.0, f"{name} unavailable")
            future = client.call_async(request)
            response = self._wait(cancel, lambda: future.result() if future.done() else None,
                                  timeout_s, f"{name} timed out")
            return require_save_success(response, name)
        finally:
            self.node.destroy_client(client)

    def _finalize_steps(self, cancel):
        from slam_toolbox.srv import SaveMap, SerializePoseGraph

        self._step(cancel, "waiting for loop-closure optimization to settle")
        history = []

        def settled():
            transform = None
            try:
                import rclpy.time
                t = self.tf.lookup_transform(self.frames["map"], self.frames["odom"],
                                             rclpy.time.Time())
                transform = (t.transform.translation.x, t.transform.translation.y,
                             t.transform.rotation.z, t.transform.rotation.w)
            except Exception:
                return False
            history.append((time.monotonic(), transform))
            window = [h for h in history if h[0] >= history[-1][0] - 3.0]
            if history[-1][0] - history[0][0] < 3.0:
                return False
            xs, ys = [w[1][0] for w in window], [w[1][1] for w in window]
            yaws = [2 * math.atan2(w[1][2], w[1][3]) for w in window]
            return (max(xs) - min(xs) < 0.01 and max(ys) - min(ys) < 0.01
                    and max(yaws) - min(yaws) < math.radians(0.5))

        self._wait(cancel, settled, 60.0, "map -> odom did not settle after the lap")
        graph_after = time.monotonic()
        self._wait(cancel, lambda: self.graph_time > graph_after, 10.0,
                   "no pose-graph update received; check enable_interactive_mode: false")

        self._step(cancel, "saving pose graph and occupancy map")
        base = self.session_dir / "posegraph"
        self._call(cancel, SerializePoseGraph, "/slam_toolbox/serialize_map",
                   SerializePoseGraph.Request(filename=str(base)))
        request = SaveMap.Request()
        request.name.data = str(self.session_dir / "map")
        self._call_retry(cancel, SaveMap, "/slam_toolbox/save_map", request)
        with self.lock:
            msg, nodes = self.map_msg, list(self.graph_nodes)
            start = self.lap.start_pose
        if msg is None:
            raise RuntimeError("no occupancy grid received on /map")
        if msg.header.frame_id != self.frames["map"]:
            raise RuntimeError(f"map frame {msg.header.frame_id!r} differs from profile")
        grid = OccupancyGrid.load(self.session_dir / "map.yaml", self.frames["map"])

        self._step(cancel, "extracting track boundaries from the corrected route")
        planning = self.race_config.get("planning", {})
        racemap = build_hardware_map(grid, nodes, self.profile.vehicle_radius_m,
                                     planning.get("tracking_allowance_m", 0.05),
                                     start_xy=start[:2] if start else None)
        map_path = self.session_dir / "racemap.json"
        map_path.write_text(json.dumps(racemap, indent=2, allow_nan=False) + "\n")
        (self.session_dir / "session.json").write_text(json.dumps({
            "session": self.supervisor.session, "profile": self.profile.summary(),
            "mapping_lap": self.lap.summary(), "pose_graph": str(base),
            "map_yaml": str(self.session_dir / "map.yaml"),
        }, indent=2) + "\n")

        return self._activate(cancel, grid, map_path, racemap, pose=None, base=base)

    def _load_steps(self, cancel, request):
        """Load a saved map and put the standing car on it; the plan comes from that map."""
        from .icp_replay import find_start
        if self.localization_source != "icp":
            raise RuntimeError("map-load needs the ICP localizer (localization.source: icp)")
        map_dir = Path(request["map_dir"]).expanduser()
        self._step(cancel, f"loading the saved map from {map_dir}")
        with self.lock:
            self.stop_slam()
            self.prepared = self.racer = self.grid = self.icp = None
            self.map_msg = self.slam_pose = None
            self.speed.reset()
            if self.monitor is not None:
                self.monitor.reset()
            self.plan_pub.publish(self.String(data="{}"))
            self.session_dir = self.data_dir / "sessions" / self.supervisor.session
            self.session_dir.mkdir(parents=True, exist_ok=True)
            if self.icp_trace is not None:
                self.icp_trace.close()
            self.icp_trace = None
            scan = self.scan
        for name in ("map.yaml", "racemap.json"):
            if not (map_dir / name).is_file():
                raise RuntimeError(f"{map_dir / name} not found; map-load needs a map saved by "
                                   "a finished mapping session (or built from one offline)")
        grid = OccupancyGrid.load(map_dir / "map.yaml", self.frames["map"])
        racemap = json.loads((map_dir / "racemap.json").read_text())
        if request.get("start") is not None:
            pose = tuple(float(v) for v in request["start"])
            found = "given"
        else:
            if scan is None:
                raise RuntimeError("no LiDAR scan to find the start pose from")
            self._step(cancel, "finding where the car is on the map from one scan")
            pose, alignment, runner_up = find_start(grid, scan, self.profile.laser_mount,
                                                    self.icp_settings)
            found = (f"searched: alignment {alignment:.2f}, best elsewhere on the map "
                     f"{runner_up:.2f}")
            if runner_up > 0.9 * alignment:
                raise RuntimeError(f"the start pose is ambiguous ({found}); the place looks like "
                                   "another place on the map. Give --start X Y YAW")
        self.node.get_logger().info(
            f"map-load start pose {pose[0]:.2f} {pose[1]:.2f} {pose[2]:.2f} ({found})")
        (self.session_dir / "session.json").write_text(json.dumps({
            "session": self.supervisor.session, "profile": self.profile.summary(),
            "loaded_map": str(map_dir), "start_pose": list(pose), "start_pose_source": found,
        }, indent=2) + "\n")
        return self._activate(cancel, grid, map_dir / "racemap.json", racemap, pose=pose)

    def _activate(self, cancel, grid, map_path, racemap, pose, base=None):
        """Generate and validate the plan, start localization and wait until it is stable."""
        from avlite_autodrive.plugin.planned_controller import AutoDRIVEPlannedController
        from avlite_autodrive.race_planning import prepare_plan
        self._step(cancel, "generating and validating the racing line")
        config = hardware_race_config(self.race_config, self.profile, map_path)
        apply_control_settings(config["c30_control"])
        prepared = prepare_plan(config, map_path)
        prepared.save(self.session_dir / "plan.json")
        racer = AutoDRIVEPlannedController(prepared)

        if pose is None:
            pose = self.pose("map")
        if pose is None:
            raise RuntimeError("no map pose available to transfer to localization")
        if self.localization_source == "icp":
            self._step(cancel, "starting AVLite ICP localization on the saved map")
            icp = IcpPose(grid.occupied_points(), pose[:3], self.icp_settings)
            with self.lock:
                if cancel.is_set():
                    raise FinalizeCancelled
                self.stop_slam()  # the mapping SLAM process is no longer needed
                self.icp = icp
                self.grid = grid
                self.monitor.reset()
        else:
            self._step(cancel, "switching SLAM Toolbox to localization")
            params = effective_parameters(self.slam_params, self.profile, "localization",
                                          self.session_dir / "slam_localization.yaml",
                                          map_file=base, start_pose=pose[:3])
            with self.lock:
                if cancel.is_set():
                    raise FinalizeCancelled
                self.stop_slam()  # mapping process stops before localization starts
                self.slam.start("localization", params)
                self.grid = grid
                self.monitor.reset()

        self._step(cancel, "waiting for stable localization")
        self._wait(cancel, lambda: not self.monitor.failures(time.monotonic()), 45.0,
                   "localization did not become stable on the saved map: "
                   + "; ".join(self.monitor.failures(time.monotonic())))
        with self.lock:
            if cancel.is_set() or self.supervisor.state != State.FINALIZING:
                raise FinalizeCancelled
            self.prepared, self.racer = prepared, racer
            self.plan_pub.publish(self.String(data=json.dumps(prepared.artifact, allow_nan=False)))
        return {"plan_sha256": prepared.artifact["map_sha256"],
                "length_m": prepared.path.length,
                "speed_range_mps": [float(min(prepared.global_plan.velocity)),
                                    float(max(prepared.global_plan.velocity))],
                "track_width_m": racemap["track_width_m"],
                "session_dir": str(self.session_dir)}

    # ------------------------------------------------------------------ status
    def publish_status(self):
        with self.lock:
            status = self.supervisor.status(time.monotonic(), self.status_extra())
        self.status_pub.publish(self.String(data=json.dumps(status, default=str)))

    def _persist(self):
        snapshot = self.supervisor.to_dict()
        if snapshot != self._persisted:
            (self.data_dir / "state.json").write_text(json.dumps(snapshot, indent=2) + "\n")
            self._persisted = snapshot

    def close(self):
        with self.lock:
            self.supervisor.stop(time.monotonic(), "supervisor shutting down")
            self.arbiter.release()
            self.finalize_cancel.set()
            self._persist()
        self.server.close()
        self.sampler.stop()
        self.stop_slam()


WARM_UP_MODULES = (
    "avlite.c10_perception.c11_perception_model",
    "avlite.c50_common.c52_world_sensor_datatypes",
    "avlite.c30_control.c39_settings",
    "avlite_autodrive.sensors",
    "avlite_autodrive.plugin.controller",
    "avlite_autodrive.plugin.planned_controller",
    "avlite_autodrive.race_planning",
    "avlite_autodrive.avlite_localization",
    "slam_toolbox.srv",
)


def warm_up_imports(log=None):
    """Import the heavy AVLite and SLAM modules before the control loop starts.

    The control step imports them the first time it needs them. On the Jetson that first
    import took about 0.6 s inside a control tick, long enough to starve the scan callbacks
    and trip the 0.25 s stale-LiDAR safety stop the moment mapping started (23 Nov). Doing it
    here, while the car is idle and nothing is authorized, moves that cost out of the tick.
    A module that is not installed is skipped: the step that needs it reports it then.
    """
    import importlib
    started = time.monotonic()
    skipped = []
    for name in WARM_UP_MODULES:
        try:
            importlib.import_module(name)
        except ImportError:
            skipped.append(name)
    if log is not None:
        log(f"warm-up imports took {time.monotonic() - started:.2f} s"
            + (f"; not available: {', '.join(skipped)}" if skipped else ""))
    return skipped


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--race-config", required=True)
    parser.add_argument("--slam-params", required=True)
    parser.add_argument("--data-dir", default=str(Path.home() / ".local/share/avlite-roboracer"))
    parser.add_argument("--socket", default=None)
    options, ros_args = parser.parse_known_args(argv)
    profile = HardwareProfile.load(options.profile)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("roboracer_supervisor")
    warm_up_imports(node.get_logger().info)
    runtime = Runtime(node, profile, options.race_config, options.slam_params,
                      options.data_dir, options.socket or default_socket_path())
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        runtime.close()
        if rclpy.ok():
            for _ in range(3):  # STOP commands; the actuator also expires on its own
                runtime.tick()
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
