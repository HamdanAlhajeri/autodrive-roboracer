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
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from avlite.c40_execution.c41_world_bridge import WorldBridge
from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c50_common.c51_capabilities import StackCapability, WorldCapability
from avlite.c50_common.c52_world_sensor_datatypes import Lidar, SensorFrame
from avlite_autodrive.ros_utils import PREFIX, odom_state, valid_scan
from avlite_autodrive.sensors import scan_cloud


class AutoDRIVEWorldBridge(WorldBridge):
    world_capabilities = frozenset({WorldCapability.LIDAR_2D})
    stack_capabilities = frozenset({StackCapability.LOCALIZATION})

    def __init__(self, ego_state=None, pm=None, **kwargs):
        self.ego_state = ego_state or EgoState(x=0.0, y=0.0, theta=0.0)
        self.perception_model = pm or PerceptionModel(ego_vehicle=self.ego_state)
        self.reference_point = None
        self.map = None
        self.lock = threading.Lock()
        self.cloud = None
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
        self.node.create_subscription(
            LaserScan, PREFIX + "/lidar", self.on_scan, qos_profile_sensor_data
        )
        self.node.create_subscription(
            Odometry, PREFIX + "/odom", self.on_odom, qos_profile_sensor_data
        )
        self.node.create_subscription(Bool, "/autodrive/reset_command", self.on_reset, 1)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    @property
    def ready(self):
        with self.lock:
            return (
                self.cloud is not None
                and min(self.scan_time, self.odom_time) > time.monotonic() - 0.5
            )

    def on_scan(self, msg):
        with self.lock:
            self.cloud = scan_cloud(msg) if valid_scan(msg) else None
            self.scan_time = time.monotonic() if self.cloud is not None else -math.inf

    def on_odom(self, msg):
        try:
            x, y, theta, velocity = odom_state(msg)
        except ValueError:
            with self.lock:
                self.odom_time = -math.inf
            return
        with self.lock:
            if self.last_pose and math.hypot(x - self.last_pose[0], y - self.last_pose[1]) > 1.0:
                self.reset_generation += 1
                self.scan_time = -math.inf
            self.last_pose = (x, y)
            self.ego_state.x, self.ego_state.y = x, y
            self.ego_state.theta, self.ego_state.velocity = theta, velocity
            self.odom_time = time.monotonic()

    def on_reset(self, msg):
        if msg.data:
            self.reset()

    def reset(self):
        with self.lock:
            self.reset_generation += 1
            self.cloud = None
            self.scan_time = self.odom_time = -math.inf
            self.last_pose = None

    def get_sensor_frame(self, agent_id=None):
        # Capture sensor and ego snapshots together for a single AVLite tick.
        with self.lock:
            self.snapshot_ego = copy.deepcopy(self.ego_state)
            return SensorFrame(
                lidar=None if self.cloud is None else self.cloud.copy(),
                lidar_sensor=self.sensor,
                frame_id="roboracer_1",
            )

    def get_ego_state(self):
        return copy.deepcopy(self.snapshot_ego)

    def control_ego_state(self, cmd, dt=0.05):
        if not self.ready:
            return  # The independent adapter times out instead of holding throttle.
        msg = AckermannDriveStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "roboracer_1"
        msg.drive.steering_angle = float(cmd.steer)
        msg.drive.acceleration = float(cmd.acceleration)
        self.publisher.publish(msg)

    def publish_diagnostics(self, values):
        with self.lock:
            now = time.monotonic()
            values = {
                **values,
                "controller_lidar_age_s": now - self.scan_time,
                "controller_odom_age_s": now - self.odom_time,
            }
        # Missing inputs remain null rather than JSON Infinity or a fresh zero.
        values = {key: value if value is None or math.isfinite(value) else None
                  for key, value in values.items()}
        self.diagnostics_publisher.publish(String(data=json.dumps(values, allow_nan=False)))

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node()
