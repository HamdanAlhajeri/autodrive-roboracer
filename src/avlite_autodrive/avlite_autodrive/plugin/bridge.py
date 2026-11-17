"""ROS transport for the simulator; no bicycle-model simulation is run here."""

import copy
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from avlite.c40_execution.c41_world_bridge import WorldBridge
from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c50_common.c51_capabilities import StackCapability, WorldCapability
from avlite.c50_common.c52_world_sensor_datatypes import Lidar
from avlite_autodrive.ros_utils import PREFIX, odom_state, pose_discontinuous, valid_scan
from avlite_autodrive.sensors import scan_cloud, scan_hit_mask
from .planned_controller import AutoDRIVESensorFrame


class AutoDRIVEWorldBridge(WorldBridge):
    world_capabilities = frozenset({WorldCapability.LIDAR_2D})
    stack_capabilities = frozenset({StackCapability.LOCALIZATION})

    def __init__(self, ego_state=None, pm=None, localization=None, stack_localization=False,
                 **kwargs):
        """Connect AVLite to ROS sensor and command topics in a background thread.

        Store the LiDAR mounting transform from the rear axle, initialize empty sensor
        history, and create a retained plan publisher so a recorder can join later.
        localization is a slam_localization.LocalizationConfig to drive on SLAM poses and
        wheel odometry; None keeps the simulator's ground-truth pose. stack_localization
        means an AVLite localization strategy owns the pose: the bridge then reads ground
        truth only for diagnostics and takes speed from wheel odometry.
        """
        self.ego_state = ego_state or EgoState(x=0.0, y=0.0, theta=0.0)
        self.perception_model = pm or PerceptionModel(ego_vehicle=self.ego_state)
        self.reference_point = None
        self.map = None
        self.lock = threading.Lock()
        self.cloud = None
        self.hit_mask = None
        self.scan_time = self.odom_time = -math.inf
        self.snapshot_ego = copy.deepcopy(self.ego_state)
        self.reset_generation = 0
        self.last_pose = None
        mount = np.eye(4)
        # Published by AutoDRIVE's broadcast_transforms(): rear axle -> LiDAR.
        mount[:3, 3] = [0.2733, 0.0, 0.096]
        self.sensor = Lidar(base_to_sensor=mount)
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("avlite_autodrive_bridge")
        self.publisher = self.node.create_publisher(
            AckermannDriveStamped, "/avlite/control_command", 1
        )
        self.diagnostics_publisher = self.node.create_publisher(
            String, "/avlite/controller_diagnostics", 1
        )
        self.plan_publisher = self.node.create_publisher(
            String, "/avlite/race_plan",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.node.create_subscription(
            LaserScan, PREFIX + "/lidar", self.on_scan, qos_profile_sensor_data
        )
        self.node.create_subscription(
            Odometry, PREFIX + "/odom", self.on_odom, qos_profile_sensor_data
        )
        self.node.create_subscription(Bool, "/autodrive/reset_command", self.on_reset, 1)
        self.truth = None  # world (x, y, yaw); only for the localization error diagnostic
        self.pose_source = None
        self.truth_only = bool(stack_localization)
        if self.truth_only:
            from avlite_autodrive.sim_odometry import ODOM_TOPIC
            self.stack_capabilities = frozenset()  # never offer ground truth as LOCALIZATION
            self.node.create_subscription(Odometry, ODOM_TOPIC, self.on_wheel_odom,
                                          qos_profile_sensor_data)
        if localization is not None:
            from avlite_autodrive.slam_localization import SlamPoseSource
            self.pose_source = SlamPoseSource(self.node, localization)
            self.pose_timeout_s = self.pose_source.profile.get("timing.pose_timeout_s")
            self.node.create_subscription(
                Odometry, self.pose_source.profile.topic("odom"), self.on_wheel_odom,
                qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    @property
    def ready(self):
        """Report whether a LiDAR cloud and odometry have both arrived within 0.5 seconds.

        With SLAM localization, also require a fresh map pose that the health monitor
        accepts; otherwise commands are withheld exactly as for stale sensors.
        """
        with self.lock:
            sensors = (
                self.cloud is not None
                and min(self.scan_time, self.odom_time) > time.monotonic() - 0.5
            )
        if not sensors or self.pose_source is None:
            return sensors
        pose = self.pose_source.pose()
        if pose is None:
            reasons = ["no map -> base_link transform"]
        elif not 0 <= pose[3] <= self.pose_timeout_s:
            reasons = [f"map pose is {pose[3]:.2f}s old"]
        else:
            reasons = self.pose_source.failures()
        if reasons and time.monotonic() - getattr(self, "_reasons_logged", 0.0) > 2.0:
            self._reasons_logged = time.monotonic()  # throttled: ready is polled at 20 Hz
            self.node.get_logger().warning("Withholding commands: " + "; ".join(reasons))
        return not reasons

    def on_scan(self, msg):
        """Convert a valid LaserScan into sensor-frame points and a real-hit mask.

        Update both under the same lock so the controller cannot read points from one scan
        with a mask from another.
        """
        with self.lock:
            self.cloud = scan_cloud(msg) if valid_scan(msg) else None
            self.hit_mask = scan_hit_mask(msg) if self.cloud is not None else None
            self.scan_time = time.monotonic() if self.cloud is not None else -math.inf

    def on_odom(self, msg):
        """Update AVLite's world pose, heading and forward speed from simulator odometry.

        Invalid data is marked stale. An unexpected jump increments the reset counter and
        requires a new scan before driving continues.
        """
        try:
            x, y, theta, velocity = odom_state(msg)
        except ValueError:
            with self.lock:
                self.odom_time = -math.inf
                self.last_pose = None
                self.truth = None
            return
        if self.pose_source is not None or self.truth_only:
            with self.lock:
                self.truth = (x, y, theta)  # reference only; never drives the car
            return
        with self.lock:
            now = time.monotonic()
            pose = (x, y, velocity, now)
            if pose_discontinuous(self.last_pose, pose):
                self.reset_generation += 1
                self.scan_time = -math.inf
            self.last_pose = pose
            self.ego_state.x, self.ego_state.y = x, y
            self.ego_state.theta, self.ego_state.velocity = theta, velocity
            self.odom_time = now

    def wheel_speed(self):
        """Latest wheel-odometry speed (m/s), or None when it is stale."""
        with self.lock:
            fresh = self.odom_time > time.monotonic() - 0.5
            return self.ego_state.velocity if fresh else None

    def true_pose(self):
        """Simulator pose (x, y, yaw) for diagnostics only, or None."""
        with self.lock:
            return self.truth

    def on_wheel_odom(self, msg):
        """Take forward speed and odometry freshness from wheel odometry (SLAM/AVLite mode)."""
        speed = msg.twist.twist.linear.x
        with self.lock:
            if math.isfinite(speed):
                self.ego_state.velocity = speed
                self.odom_time = time.monotonic()
            else:
                self.odom_time = -math.inf

    def update_slam_pose(self):
        """Copy the latest SLAM pose into ego_state; a jump resets tracking like a teleport."""
        pose = self.pose_source.pose()
        with self.lock:
            if pose is None:
                self.last_pose = None
                return
            now = time.monotonic()
            current = (pose[0], pose[1], self.ego_state.velocity, now)
            if pose_discontinuous(self.last_pose, current):
                self.reset_generation += 1
                self.scan_time = -math.inf
            self.last_pose = current
            self.ego_state.x, self.ego_state.y, self.ego_state.theta = pose[:3]

    def on_reset(self, msg):
        """Invalidate the current sensor state when a simulator reset is announced."""
        if msg.data:
            self.reset()
            if self.pose_source is not None:
                self.pose_source.reinitialize()

    def reset(self):
        """Clear sensor history and increment the reset generation.

        The runner watches this generation to reset planner progress and controller memory
        even if sensors recover between two control ticks.
        """
        with self.lock:
            self.reset_generation += 1
            self.cloud = None
            self.hit_mask = None
            self.scan_time = self.odom_time = -math.inf
            self.last_pose = None

    def get_sensor_frame(self, agent_id=None):
        # Capture sensor and ego snapshots together for a single AVLite tick.
        """Copy the latest scan and pose together for one AVLite control update.

        The returned frame includes sensor ages and a hit mask. Copies prevent the ROS
        callback thread from changing data while the controller is using it.
        """
        if self.pose_source is not None:
            self.update_slam_pose()
        with self.lock:
            self.snapshot_ego = copy.deepcopy(self.ego_state)
            now = time.monotonic()
            return AutoDRIVESensorFrame(
                lidar=None if self.cloud is None else self.cloud.copy(),
                lidar_sensor=self.sensor,
                frame_id="roboracer_1",
                lidar_hit_mask=None if self.hit_mask is None else self.hit_mask.copy(),
                lidar_age_s=now - self.scan_time,
                odom_age_s=now - self.odom_time,
            )

    def get_ego_state(self):
        """Return the pose snapshot captured with the most recent sensor frame."""
        return copy.deepcopy(self.snapshot_ego)

    def publish_plan(self, artifact):
        """Publish the validated plan as retained JSON for recording and later analysis."""
        self.plan_publisher.publish(String(data=json.dumps(artifact, allow_nan=False)))

    def control_ego_state(self, cmd, dt=0.05):
        """Forward AVLite steering and acceleration to the independent ROS actuator.

        Do not publish when sensors are stale; the actuator watchdog then removes throttle.
        dt is accepted for the WorldBridge interface but is not used to simulate motion.
        """
        if not self.ready:
            return  # The independent adapter times out instead of holding throttle.
        msg = AckermannDriveStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "roboracer_1"
        msg.drive.steering_angle = float(cmd.steer)
        msg.drive.acceleration = float(cmd.acceleration)
        self.publisher.publish(msg)

    def publish_diagnostics(self, values):
        """Add sensor ages to controller diagnostics and publish a JSON snapshot.

        Represent unavailable or nonfinite values as null so recorded data does not confuse
        missing information with a measured zero.
        """
        with self.lock:
            now = time.monotonic()
            values = {
                **values,
                "controller_lidar_age_s": now - self.scan_time,
                "controller_odom_age_s": now - self.odom_time,
            }
            truth = self.truth
        if self.pose_source is not None:
            values.update(self.pose_source.diagnostics(truth))
        # Missing inputs remain null rather than JSON Infinity or a fresh zero.
        values = {key: value if value is None or math.isfinite(value) else None
                  for key, value in values.items()}
        self.diagnostics_publisher.publish(String(data=json.dumps(values, allow_nan=False)))

    def close(self):
        """Stop SLAM, the ROS executor thread and release this bridge's node."""
        if self.pose_source is not None:
            self.pose_source.stop()
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node()
