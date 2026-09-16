import math

import numpy as np
import pytest

from avlite.c30_control.c39_settings import ControlSettings
from avlite.c30_control.c35_pure_pursuit import FollowTheGapController
from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c50_common.c52_world_sensor_datatypes import SensorFrame, Lidar
from avlite.c40_execution.c44_sync_executer import SyncExecuter
from avlite.c40_execution.c49_settings import ExecutionSettings
from avlite.c50_common.c51_capabilities import StackCapability, WorldCapability
from avlite_autodrive.plugin.controller import AutoDRIVEFollowTheGap


@pytest.fixture
def controller():
    ControlSettings.c32_ego_distance_front_axle = 0.324
    ControlSettings.c32_ego_max_steering = math.pi / 6
    ControlSettings.c32_ego_min_steering = -math.pi / 6
    ControlSettings.c32_ego_max_acceleration = 1.0
    ControlSettings.c32_ego_min_acceleration = -2.0
    ControlSettings.c35_lookahead_distance = 1.0
    ControlSettings.c35_lookahead_speed_gain = 0.0
    ControlSettings.c35_cruise_velocity = 0.5
    ControlSettings.c35_bubble_radius = 0.24
    return AutoDRIVEFollowTheGap()


def corridor(left=1, right=1, front=8):
    a = np.linspace(-math.pi / 2 + 0.001, math.pi / 2 - 0.001, 720)
    side = np.where(a >= 0, left, right) / np.maximum(abs(np.sin(a)), 1e-8)
    r = np.minimum(side, front / np.cos(a))
    return np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros((len(a), 2))))


def run(c, points, speed=0):
    return c.control(
        EgoState(x=0, y=0, velocity=speed), sensors=SensorFrame(lidar=points), control_dt=0.05
    )


def test_real_upstream_control_with_straight_dense_scan(controller):
    assert isinstance(controller, FollowTheGapController)
    cmd = run(controller, corridor())
    assert abs(cmd.steer) < 0.02
    assert cmd.acceleration > 0


@pytest.mark.parametrize("left,right,sign", [(0.4, 1.5, -1), (1.5, 0.4, 1)])
def test_turn_away_from_near_wall(controller, left, right, sign):
    cmd = run(controller, corridor(left, right))
    assert cmd.steer * sign > 0.01
    assert abs(cmd.steer) <= math.pi / 6


def test_blocked_and_missing_scan_request_deceleration(controller):
    assert run(controller, corridor(left=0.3, right=0.3, front=0.2), 0.5).acceleration < 0
    assert run(controller, np.zeros((0, 4)), 0.5).acceleration < 0


def test_lidar_mount_and_noisy_scan(controller):
    mount = np.eye(4)
    mount[:3, 3] = [0.2733, 0, 0.096]
    pts = Lidar(base_to_sensor=mount).to_base(np.array([[1.0, 0, 0, 0]]))
    np.testing.assert_allclose(pts[0, :3], [1.2733, 0, 0.096])
    cloud = corridor()
    cloud[:, :2] += np.random.default_rng(7).normal(0, 0.01, cloud[:, :2].shape)
    cmd = run(controller, cloud)
    assert abs(cmd.steer) < 0.1


def test_real_syncexecuter_calls_controller_and_bridge(controller):
    class World:
        world_capabilities = frozenset({WorldCapability.LIDAR_2D})
        stack_capabilities = frozenset({StackCapability.LOCALIZATION})
        stack_requirements = frozenset()

        def get_sensor_frame(self):
            return SensorFrame(lidar=corridor())

        def get_ego_state(self):
            return EgoState(x=0, y=0, velocity=0)

        def control_ego_state(self, cmd, dt):
            self.command = cmd

    world = World()
    ExecutionSettings.c41_world_stack_capabilities = ["LOCALIZATION"]
    stack = SyncExecuter(
        PerceptionModel(ego_vehicle=EgoState(x=0, y=0)), controller=controller, world=world
    )
    stack.step(control_dt=0.05, sim_dt=0.05, call_perceive=False, call_replan=False)
    assert world.command.acceleration > 0
    assert abs(world.command.steer) < 0.02
