import math
import json

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
def controller(monkeypatch):
    settings = {
        "c32_ego_distance_front_axle": 0.324,
        "c32_ego_max_steering": math.pi / 6,
        "c32_ego_min_steering": -math.pi / 6,
        "c32_ego_max_acceleration": 1.0,
        "c32_ego_min_acceleration": -2.0,
        "c35_lookahead_distance": 1.0,
        "c35_lookahead_speed_gain": 0.0,
        "c35_min_lookahead": 0.6,
        "c35_max_lookahead": 2.5,
        "c35_cruise_velocity": 0.5,
        "c35_bubble_radius": 0.24,
        "c35_valpha": 1.0,
        "c35_vbeta": 0.0,
        "c35_vgamma": 0.0,
    }
    for name, value in settings.items():
        monkeypatch.setattr(ControlSettings, name, value)
    return AutoDRIVEFollowTheGap()


def corridor(left=1, right=1, front=8):
    a = np.linspace(-math.pi / 2 + 0.001, math.pi / 2 - 0.001, 720)
    side = np.where(a >= 0, left, right) / np.maximum(abs(np.sin(a)), 1e-8)
    r = np.minimum(side, front / np.cos(a))
    return np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros((len(a), 2))))


def l_bend(direction=1, offset=0.0, width=1.2):
    # Ray-cast a 1.2 m wide left-turning corridor. The inside corner lies at
    # (0.3, 0.6); mirror the cloud for a right turn.
    angles = np.linspace(-math.pi / 2 + 0.001, math.pi / 2 - 0.001, 720)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    ranges = np.full(len(angles), 8.0)
    inner = 0.3 + offset
    front = inner + width
    for axis, value, low, high in [
        (1, -width / 2, -8, front),
        (0, front, -width / 2, 8),
        (1, width / 2, -8, inner),
        (0, inner, width / 2, 8),
    ]:
        distance = value / directions[:, axis]
        other = distance * directions[:, 1 - axis]
        hit = (distance > 0) & (other >= low) & (other <= high)
        ranges = np.minimum(ranges, np.where(hit, distance, np.inf))
    cloud = np.column_stack((directions * ranges[:, None], np.zeros((len(angles), 2))))
    cloud[:, 1] *= direction
    return cloud


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


@pytest.fixture
def racing_controller(controller, monkeypatch):
    monkeypatch.setattr(ControlSettings, "c35_cruise_velocity", 2.5)
    return AutoDRIVEFollowTheGap(racing={})


def test_racing_retains_full_straight_speed(racing_controller):
    cmd = run(racing_controller, corridor(front=12), speed=2.0)
    assert abs(cmd.steer) < 0.02
    assert cmd.acceleration > 0
    assert racing_controller.diagnostics["target_velocity_mps"] == pytest.approx(2.5)
    assert racing_controller.diagnostics["acceleration_saturated"] is False


def test_racing_slows_before_wall_requires_tight_steering(racing_controller):
    cmd = run(racing_controller, corridor(front=2.5), speed=2.5)
    assert racing_controller.blocked is False  # Still a navigable local target.
    assert abs(cmd.steer) < 0.02
    assert 0 < racing_controller.diagnostics["target_velocity_mps"] < 2.5
    assert cmd.acceleration <= -1.5
    # The speed envelope reserves both disk radius and reaction/braking space.
    diag = racing_controller.diagnostics
    v = diag["clearance_speed_limit_mps"]
    assert v * 0.25 + v**2 / (2 * 1.5) <= diag["clearance_m"] - 0.15 + 1e-8


def test_racing_curvature_speed_cap(racing_controller):
    racing_controller.racing["lateral_acceleration_mps2"] = 0.25
    run(racing_controller, corridor(left=0.4, right=1.5), speed=2.0)
    diag = racing_controller.diagnostics
    assert diag["curvature_1pm"] < 0
    assert 0 < diag["curvature_speed_limit_mps"] < 2.5
    assert diag["target_velocity_mps"] <= diag["curvature_speed_limit_mps"]
    assert diag["target_velocity_mps"]**2 * abs(diag["curvature_1pm"]) <= 0.25 + 1e-8


def test_long_lookahead_can_shorten_without_deadlocking(racing_controller):
    racing_controller.lookahead_speed_gain = 0.9
    racing_controller.min_lookahead = 0.6
    racing_controller.max_lookahead = 2.5
    # No 2.25 m straight ray fits, but a shorter target is traversable.
    cmd = run(racing_controller, corridor(left=0.45, right=1.5, front=1.4), speed=2.5)
    diag = racing_controller.diagnostics
    assert racing_controller.blocked is False
    assert 0.6 <= diag["lookahead_m"] < 2.25
    assert 0 < diag["target_velocity_mps"] < 2.5
    assert cmd.acceleration < 0
    # Steering uses the shortened radius, not upstream's original radius.
    expected = math.atan(0.324 * diag["curvature_1pm"])
    assert cmd.steer == pytest.approx(np.clip(expected, -math.pi / 6, math.pi / 6))


def test_l_bend_shortens_target_and_brakes_before_entry(racing_controller):
    racing_controller.lookahead_speed_gain = 0.9
    cmd = run(racing_controller, l_bend(), speed=2.5)
    assert not racing_controller.blocked
    assert cmd.steer > 0.0
    assert cmd.acceleration <= -1.5
    assert racing_controller.diagnostics["lookahead_m"] < 2.25
    assert 0 < racing_controller.diagnostics["target_velocity_mps"] < 2.0


def test_racing_does_not_look_beyond_observed_scan(racing_controller):
    a = np.linspace(-math.pi / 2, math.pi / 2, 720)
    cloud = np.column_stack((1.5 * np.cos(a), 1.5 * np.sin(a), np.zeros((len(a), 2))))
    run(racing_controller, cloud, speed=2.5)
    assert racing_controller.diagnostics["clearance_m"] <= 1.5 - 0.24 + 1e-8
    assert racing_controller.diagnostics["target_velocity_mps"] < 2.5
    # A cloud in one side direction gives no evidence that forward is free.
    unsupported = cloud[(a > 0.2) & (a < 0.5)]
    cmd = run(racing_controller, unsupported, speed=2.5)
    assert racing_controller.diagnostics["target_velocity_mps"] == 0
    assert cmd.acceleration < 0


def test_racing_lidar_transform_matches_body_cloud(racing_controller):
    cloud = corridor(left=0.45, right=1.5, front=2.5)
    expected = run(racing_controller, cloud, speed=2.0)
    expected_diag = dict(racing_controller.diagnostics)
    racing_controller.reset()
    mount = np.eye(4)
    angle = 0.3
    mount[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
    mount[:3, 3] = [0.2733, 0, 0.096]
    sensor_cloud = cloud.copy()
    sensor_cloud[:, :3] = (cloud[:, :3] - mount[:3, 3]) @ mount[:3, :3]
    actual = racing_controller.control(
        EgoState(x=23, y=-17, theta=1.2, velocity=2.0),
        sensors=SensorFrame(lidar=sensor_cloud, lidar_sensor=Lidar(base_to_sensor=mount)),
        control_dt=0.05,
    )
    assert actual.steer == pytest.approx(expected.steer)
    assert actual.acceleration == pytest.approx(expected.acceleration)
    assert racing_controller.diagnostics["clearance_m"] == pytest.approx(
        expected_diag["clearance_m"]
    )


@pytest.mark.parametrize(
    "points", [None, np.empty((0, 4)), np.full((20, 4), np.nan), corridor(0.3, 0.3, 0.2)]
)
def test_racing_blocked_and_missing_data_clear_previous_intent(racing_controller, points):
    run(racing_controller, corridor(), speed=1.0)
    cmd = run(racing_controller, points, speed=1.0)
    assert racing_controller.blocked
    assert cmd.acceleration == -2.0
    assert cmd.steer == 0.0
    assert racing_controller.diagnostics["target_velocity_mps"] == 0.0
    assert racing_controller.diagnostics["acceleration_saturated"] is True
    assert racing_controller.cte_v_sum == 0.0
    json.dumps(racing_controller.diagnostics, allow_nan=False)


def test_racing_reset_clears_pid_bearing_and_diagnostics(racing_controller):
    run(racing_controller, corridor(left=0.4, right=1.5), speed=2.0)
    assert racing_controller.bearing != 0
    racing_controller.reset()
    assert racing_controller.bearing == 0
    assert racing_controller.blocked
    assert racing_controller.cte_v_sum == 0
    assert racing_controller.cte_velocity == 0
    assert racing_controller.diagnostics["target_velocity_mps"] == 0


@pytest.mark.parametrize("invalid", [0, -1, math.nan, math.inf, True, "1.5", None])
@pytest.mark.parametrize("setting", AutoDRIVEFollowTheGap.RACING_DEFAULTS)
def test_invalid_racing_settings_fail_early(controller, setting, invalid):
    with pytest.raises(ValueError, match="positive finite"):
        AutoDRIVEFollowTheGap(racing={setting: invalid})


@pytest.mark.parametrize("settings", [{"typo": 1}, [], {"braking_deceleration_mps2": 2.1}])
def test_racing_schema_and_braking_feasibility(controller, settings):
    with pytest.raises(ValueError):
        AutoDRIVEFollowTheGap(racing=settings)


@pytest.fixture
def preview_controller(racing_controller, monkeypatch):
    monkeypatch.setattr(ControlSettings, "c35_lookahead_speed_gain", 0.4)
    monkeypatch.setattr(ControlSettings, "c35_max_lookahead", 1.8)
    return AutoDRIVEFollowTheGap(racing={"gap_preview_min_m": 1.5})


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("speed", [0.6, 1.0])
def test_preview_turns_before_short_pursuit_detects_bend(preview_controller, direction, speed):
    baseline = AutoDRIVEFollowTheGap(racing={})
    old = run(baseline, l_bend(direction, offset=0.2), speed=speed)
    new = run(preview_controller, l_bend(direction, offset=0.2), speed=speed)
    assert old.steer == pytest.approx(0, abs=1e-8)
    assert new.steer * direction > math.radians(5)
    diag = preview_controller.diagnostics
    assert diag["gap_preview_requested_m"] == 1.5
    assert diag["gap_preview_selected_m"] > diag["lookahead_m"]
    assert diag["lookahead_m"] == pytest.approx(0.6)
    assert diag["target_bearing_raw_rad"] * direction > 0
    assert diag["target_bearing_rad"] == preview_controller.bearing
    assert diag["curvature_1pm"] == pytest.approx(2 * math.sin(preview_controller.bearing) / 0.6)
    expected = math.atan(0.324 * diag["curvature_1pm"])
    assert new.steer == pytest.approx(np.clip(expected, -math.pi / 6, math.pi / 6))
    raw_curvature = abs(2 * math.sin(diag["target_bearing_raw_rad"]) / 0.6)
    assert diag["curvature_speed_limit_mps"] == pytest.approx(
        min(2.5, math.sqrt(3.0 / max(abs(diag["curvature_1pm"]), raw_curvature)))
    )


def test_preview_persists_while_slowing_without_changing_pursuit_radius(preview_controller):
    for speed, pursuit in [(2.5, 1.0), (1.0, 0.6), (0.6, 0.6), (0.0, 0.6)]:
        cmd = run(preview_controller, corridor(front=12), speed=speed)
        diag = preview_controller.diagnostics
        assert abs(cmd.steer) < 0.02
        assert diag["target_velocity_mps"] == 2.5
        assert diag["gap_preview_requested_m"] == 1.5
        assert diag["gap_preview_selected_m"] == 1.5
        assert diag["gap_preview_fallback"] is False
        assert diag["lookahead_m"] == pytest.approx(pursuit)


@pytest.mark.parametrize("width", [0.8, 1.0])
def test_preview_can_shorten_at_tight_bend(preview_controller, width):
    cmd = run(preview_controller, l_bend(width=width), speed=2.5)
    diag = preview_controller.diagnostics
    assert not preview_controller.blocked
    assert cmd.steer > 0
    assert cmd.acceleration <= -1.5
    assert diag["gap_preview_requested_m"] == 1.5
    assert 0.6 <= diag["gap_preview_selected_m"] < 1.5
    assert diag["gap_preview_fallback"] is True
    assert diag["lookahead_m"] == min(1.0, diag["gap_preview_selected_m"])
    expected = math.atan(0.324 * diag["curvature_1pm"])
    assert cmd.steer == pytest.approx(np.clip(expected, -math.pi / 6, math.pi / 6))


def test_preview_uses_longer_speed_lookahead_up_to_maximum(preview_controller):
    for speed, requested in [(4.0, 1.6), (5.0, 1.8)]:
        run(preview_controller, corridor(front=12), speed=speed)
        diag = preview_controller.diagnostics
        assert diag["gap_preview_requested_m"] == pytest.approx(requested)
        assert diag["gap_preview_selected_m"] == pytest.approx(requested)
        assert diag["lookahead_m"] == pytest.approx(requested)


@pytest.mark.parametrize("speed", [0.0, 1.0, 2.5, 5.0])
def test_omitted_preview_preserves_existing_racing_behavior(preview_controller, speed):
    baseline = AutoDRIVEFollowTheGap(racing={})
    inert_preview = AutoDRIVEFollowTheGap(racing={"gap_preview_min_m": 0.6})
    # With a floor equal to the existing minimum, all horizons and commands
    # must remain identical, including stateful smoothing and blocked behavior.
    for points in [corridor(), l_bend(), l_bend(-1), corridor(0.3, 0.3, 0.2), corridor()]:
        old = run(baseline, points, speed=speed)
        new = run(inert_preview, points, speed=speed)
        assert new.steer == pytest.approx(old.steer)
        assert new.acceleration == pytest.approx(old.acceleration)
        assert inert_preview.diagnostics == baseline.diagnostics


@pytest.mark.parametrize("points", [None, np.full((20, 4), np.nan), corridor(0.3, 0.3, 0.2)])
def test_preview_blocked_scan_clears_target_diagnostics(preview_controller, points):
    run(preview_controller, l_bend(), speed=1)
    assert preview_controller.diagnostics["target_bearing_rad"] != 0
    cmd = run(preview_controller, points, speed=1)
    assert cmd.acceleration == -2
    assert cmd.steer == 0
    assert preview_controller.blocked
    diag = preview_controller.diagnostics
    for key in ["lookahead_m", "gap_preview_selected_m", "target_bearing_raw_rad",
                "target_bearing_rad", "curvature_1pm", "target_velocity_mps"]:
        assert diag[key] == 0
    assert diag["gap_preview_fallback"] is False
    json.dumps(diag, allow_nan=False)


def test_preview_does_not_make_unsupported_direction_free(preview_controller):
    cloud = corridor()
    bearings = np.arctan2(cloud[:, 1], cloud[:, 0])
    cloud = cloud[(bearings > 0.2) & (bearings < 0.5)]
    cmd = run(preview_controller, cloud, speed=1)
    assert preview_controller.diagnostics["target_velocity_mps"] == 0
    assert cmd.acceleration < 0


def test_preview_reset_clears_intent(preview_controller):
    run(preview_controller, l_bend(), speed=1)
    preview_controller.reset()
    diag = preview_controller.diagnostics
    for key in ["lookahead_m", "gap_preview_requested_m", "gap_preview_selected_m",
                "target_bearing_raw_rad", "target_bearing_rad"]:
        assert diag[key] == 0
    assert diag["gap_preview_fallback"] is False
    assert preview_controller.bearing == 0
    assert preview_controller.blocked


@pytest.mark.parametrize("invalid", [0, -1, math.nan, math.inf, True, "1.5", None])
def test_invalid_preview_rejected(controller, invalid):
    with pytest.raises(ValueError, match="positive finite"):
        AutoDRIVEFollowTheGap(racing={"gap_preview_min_m": invalid})


@pytest.mark.parametrize("invalid", [0.5, 2.6])
def test_preview_outside_lookahead_bounds_rejected(controller, invalid):
    with pytest.raises(ValueError, match="lookahead bounds"):
        AutoDRIVEFollowTheGap(racing={"gap_preview_min_m": invalid})


@pytest.mark.parametrize("minimum,maximum", [
    (math.nan, 2.5), (0.6, math.inf), (2.5, 0.6), (True, 2.5), (0, 2.5), (0.6, "2.5"),
])
def test_preview_requires_valid_lookahead_bounds(controller, monkeypatch, minimum, maximum):
    monkeypatch.setattr(ControlSettings, "c35_min_lookahead", minimum)
    monkeypatch.setattr(ControlSettings, "c35_max_lookahead", maximum)
    with pytest.raises(ValueError, match="lookahead bounds"):
        AutoDRIVEFollowTheGap(racing={"gap_preview_min_m": 1.5})
