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
from .lap_detector import LapDetector
from .localization import (
    LocalizationMonitor, LocalizationThresholds, scan_alignment, scan_points,
)
from .map_builder import build_hardware_map
from .occupancy import OccupancyGrid
from .profile import HardwareProfile
from .protocol import CommandServer, default_socket_path, dispatch
from .racing import hardware_race_config, mapping_control_config, mapping_racing_config
from .readiness import actuator_failures, mapping_failures, race_failures
from .slam import SlamProcess, effective_parameters, require_save_success
from .supervisor import Action, State, Supervisor

CONTROL_DT = 0.05


def sensor_stamp_is_fresh(stamp_ns, previous_ns, now_ns, timeout_s):
    return (stamp_ns > 0 and (previous_ns is None or stamp_ns > previous_ns)
            and 0 <= (now_ns - stamp_ns) * 1e-9 <= timeout_s)


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
        self.pending_scans = deque(maxlen=100)
        self.odom_speed = 0.0
        self.slam_pose = None
        self.graph_nodes = []
        self.graph_time = -math.inf
        self.map_msg = None
        self.actuator = None
        self.slam_nodes = 0
        self.last_graph_check = -math.inf

        self.slam = SlamProcess(self.data_dir / "logs")
        self.arbiter = CommandArbiter(profile.get("timing.command_timeout_s"))
        self.speed = SpeedDemand(0.0)
        self.lap = LapDetector()
        self.lap_started = False
        self.stopping_after_lap = False
        self.stopped_since = None
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
            self.monitor = LocalizationMonitor(LocalizationThresholds.from_profile(profile))
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
    def on_scan(self, msg):
        with self.lock:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if not sensor_stamp_is_fresh(stamp, self.last_scan_stamp,
                                         self.node.get_clock().now().nanoseconds,
                                         self.profile.get("timing.sensor_timeout_s")):
                return
            self.last_scan_stamp = stamp
            self.scan = msg
            self.scan_time = time.monotonic()
            self.pending_scans.append((self.scan_time, msg))

    def on_odom(self, msg):
        speed = msg.twist.twist.linear.x
        with self.lock:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if not math.isfinite(speed) or not sensor_stamp_is_fresh(
                    stamp, self.last_odom_stamp, self.node.get_clock().now().nanoseconds,
                    self.profile.get("timing.sensor_timeout_s")):
                return
            self.last_odom_stamp = stamp
            self.odom_speed = speed
            self.odom_time = time.monotonic()

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
                and request.get("command") in ("map-start", "race-start", "stop")):
            self.node.get_logger().info(f"{request['command']}: {reply.get('message')}")
        return reply

    # ------------------------------------------------------------ measurements
    def ages(self, now):
        return now - self.scan_time, now - self.odom_time

    def pose(self, target, stamp=None):
        """Return base_link in target ('map' or 'odom') as (x, y, yaw, age_s), or None."""
        import rclpy.time
        from .geometry import yaw_from_quaternion
        try:
            t = self.tf.lookup_transform(self.frames[target], self.frames["base_link"],
                                         stamp or rclpy.time.Time())
        except Exception:  # tf2 raises several lookup/extrapolation exception types
            return None
        q, p = t.transform.rotation, t.transform.translation
        age = (self.node.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds
        pose = p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w), age * 1e-9
        return pose if all(math.isfinite(v) for v in pose) else None

    def foreign_slam_nodes(self):
        """slam_toolbox nodes in the ROS graph that this runtime did not start."""
        return max(0, self.slam_nodes - (1 if self.slam.running else 0))

    def localization_processes(self):
        """Processes that could publish map -> odom; READY needs exactly our one."""
        own = 1 if self.slam.running and self.slam.mode == "localization" else 0
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
            "mapping_lap": self.lap.summary() if self.lap_started else None,
            "finalize_step": self.finalize_step,
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
            self.supervisor.tick(now)
            self.check_faults(now)
            self.run_actions(now)
            try:
                command = self.control(now)
            except Exception as exc:  # any controller failure must stop, not crash
                self.node.get_logger().error(f"Controller failed: {exc!r}")
                self.supervisor.fault(f"controller error: {exc}", now)
                self.run_actions(now)
                command = None
            self.run_actions(now)  # control may have stopped or finalized the session
            if command is not None and self.supervisor.state in (State.MAPPING, State.RACING):
                self.arbiter.submit(*command, now)
            out = self.arbiter.output(now)
            self._persist()
        msg = self.Drive()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = out.source.value
        msg.drive.steering_angle = out.steering_rad
        msg.drive.speed = out.speed_mps
        self.command_pub.publish(msg)

    def on_new_scan(self, now, scan):
        """Update localization health and the lap detector from the newest scan."""
        import rclpy.time
        stamp = rclpy.time.Time.from_msg(scan.header.stamp)
        map_pose = self.pose("map", stamp)
        odom_pose = self.pose("odom", stamp)
        if map_pose is None or odom_pose is None:
            return False
        state = self.supervisor.state
        if state == State.MAPPING and self.lap_started:
            result = self.lap.update(map_pose[:3], odom_pose[:3], list(scan.ranges))
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
        actuator = actuator_failures(self.actuator, now,
                                     2 * self.profile.get("timing.command_timeout_s"))
        if self.slam.died:
            self.supervisor.fault("SLAM Toolbox process exited", now)
        elif state in (State.MAPPING, State.RACING, State.READY, State.FINALIZING):
            if not (0 <= scan_age <= timeout and 0 <= odom_age <= timeout):
                self.supervisor.fault("stale LiDAR or odometry", now)
            elif actuator:
                self.supervisor.fault("; ".join(actuator), now)
            elif state in (State.RACING, State.READY):
                failures = [f for f in self.monitor.failures(now) if "stable for only" not in f]
                if failures:
                    self.supervisor.fault("lost localization: " + "; ".join(failures), now)
            elif state == State.MAPPING and self.lap_started:
                poses = (self.pose("map"), self.pose("odom"))
                if any(p is None or not 0 <= p[3] <= self.profile.get("timing.pose_timeout_s")
                       for p in poses):
                    self.supervisor.fault("mapping pose stale", now)
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
                    self.slam.stop()
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
            elif action == Action.START_RACING:
                self.racer.reset()
                self.speed = SpeedDemand(self.prepared.settings.speed_limit)
                self.pose_invalid_since = None
                self.arbiter.select(CommandSource.RACING)
            elif action == Action.STOP_MOTION:
                self.arbiter.release()
                self.speed.reset()
                if self.slam.mode == "mapping":
                    self.slam.stop()
            elif action == Action.ABORT_FINALIZE:
                # The worker checks this under the same lock before switching processes.
                self.finalize_cancel.set()
                self.slam.stop()
                self.grid = None

    def start_mapping(self, now):
        from avlite_autodrive.plugin.controller import AutoDRIVEFollowTheGap
        self.slam.stop()  # a previous session's localization process
        self.prepared = self.racer = self.grid = None
        self.graph_nodes = []
        self.graph_time = -math.inf
        self.map_msg = self.slam_pose = None
        self.pending_scans.clear()
        self.tf.clear()
        if self.monitor is not None:
            self.monitor.reset()
        self.plan_pub.publish(self.String(data="{}"))
        self.session_dir = self.data_dir / "sessions" / self.supervisor.session
        self.session_dir.mkdir(parents=True, exist_ok=True)
        apply_control_settings(mapping_control_config(self.race_config, self.profile))
        self.mapper = AutoDRIVEFollowTheGap(
            racing=mapping_racing_config(self.race_config, self.profile))
        self.speed = SpeedDemand(self.profile.get("limits.mapping_speed_mps"))
        self.lap = LapDetector()
        self.lap_started = False
        self.stopping_after_lap = False
        params = effective_parameters(self.slam_params, self.profile, "mapping",
                                      self.session_dir / "slam_mapping.yaml")
        self.slam.start("mapping", params)
        self.arbiter.select(CommandSource.MAPPING)

    def sensor_frame(self, now):
        from avlite.c50_common.c52_world_sensor_datatypes import Lidar
        from avlite_autodrive.plugin.planned_controller import AutoDRIVESensorFrame
        from avlite_autodrive.sensors import scan_cloud, scan_hit_mask
        x, y, yaw = self.profile.laser_mount
        mount = np.eye(4)
        mount[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        mount[:2, 3] = [x, y]
        scan_age, odom_age = self.ages(now)
        return AutoDRIVESensorFrame(
            lidar=scan_cloud(self.scan), lidar_sensor=Lidar(base_to_sensor=mount),
            frame_id=self.frames["base_link"], lidar_hit_mask=scan_hit_mask(self.scan),
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
    def finalize(self, cancel, session):
        """Optimize, save, build and validate the map and plan, then switch to localization."""
        try:
            plan = self._finalize_steps(cancel)
        except FinalizeCancelled:
            return
        except Exception as exc:  # any worker failure must leave FINALIZING
            self.node.get_logger().error(f"Finalization failed: {exc}")
            with self.lock:
                if self.supervisor.session == session:
                    self.slam.stop()
                    self.supervisor.finalize_failed([str(exc)], time.monotonic())
            return
        with self.lock:
            if not cancel.is_set() and self.supervisor.session == session:
                failures = self.race_readiness(time.monotonic())
                if failures:
                    self.slam.stop()
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
        from avlite_autodrive.plugin.planned_controller import AutoDRIVEPlannedController
        from avlite_autodrive.race_planning import prepare_plan

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
        self._call(cancel, SaveMap, "/slam_toolbox/save_map", request)
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

        self._step(cancel, "generating and validating the racing line")
        config = hardware_race_config(self.race_config, self.profile, map_path)
        apply_control_settings(config["c30_control"])
        prepared = prepare_plan(config, map_path)
        prepared.save(self.session_dir / "plan.json")
        racer = AutoDRIVEPlannedController(prepared)

        self._step(cancel, "switching SLAM Toolbox to localization")
        pose = self.pose("map")
        if pose is None:
            raise RuntimeError("no map pose available to transfer to localization")
        params = effective_parameters(self.slam_params, self.profile, "localization",
                                      self.session_dir / "slam_localization.yaml",
                                      map_file=base, start_pose=pose[:3])
        with self.lock:
            if cancel.is_set():
                raise FinalizeCancelled
            self.slam.stop()  # mapping process stops before localization starts
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
        self.slam.stop()


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
