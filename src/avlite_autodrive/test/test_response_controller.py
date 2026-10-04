import json
import math

import numpy as np
import pytest
from avlite.c30_control.c39_settings import ControlSettings

from avlite_autodrive.plugin.response_controller import AutoDRIVEResponseController
from avlite_autodrive.race_planning import prepare_plan
from test_race_planning import state_at, clear_scan


@pytest.fixture
def response_controller(tmp_path, monkeypatch):
    def stadium(radius):
        angles = np.linspace(-math.pi / 2, math.pi / 2, 60, endpoint=False)
        right = np.column_stack((3 + radius * np.cos(angles), radius * np.sin(angles)))
        top = np.column_stack((np.linspace(3, -3, 60, endpoint=False), np.full(60, radius)))
        return np.vstack((right, top, -right, -top)).tolist()

    map_path = tmp_path / "map.json"
    map_path.write_text(json.dumps({"LeftBound": stadium(1.3), "RightBound": stadium(2.7),
                                    "ReferencePoint": [0, 0]}))
    control = {"c32_ego_distance_front_axle": 0.324, "c32_ego_max_velocity": 1.5,
               "c32_ego_max_acceleration": 1.0, "c32_ego_min_acceleration": -2.0,
               "c32_ego_max_steering": math.pi / 6, "c32_ego_min_steering": -math.pi / 6,
               "c35_cruise_velocity": 1.5, "c35_valpha": 1.0,
               "c35_lookahead_speed_gain": 0.4, "c35_min_lookahead": 0.6,
               "c35_max_lookahead": 1.8}
    for key, value in control.items():
        monkeypatch.setattr(ControlSettings, key, value)
    prepared = prepare_plan({"c30_control": control, "planning": {
        "map_path": str(map_path), "max_velocity_mps": 1.5}}, tmp_path / "avlite.yaml")
    now = [0.0]
    return AutoDRIVEResponseController(prepared, 1.5, 3, clock=lambda: now[0]), now


def test_real_planned_controller_coasts_on_straights_once_each_circuit(response_controller):
    controller, now = response_controller
    prepared = controller.prepared
    attempts = set()
    for s in np.arange(0, prepared.path.length * 3, 0.075):
        now[0] += 0.05
        ego = state_at(prepared, s, speed=1.5)
        cmd = controller.control(ego, prepared.global_plan, 0.05, sensors=clear_scan())
        if controller.diagnostics["response_phase"] == 2:
            attempts.add(controller.diagnostics["response_trial_id"])
            assert cmd.acceleration == -2
            assert abs(cmd.steer) <= 0.05 * controller.ego_max_steering
    assert attempts == {1, 2, 3}
    assert controller.experiment.phase == controller.experiment.FINISHED


def test_invalid_sensors_abort_and_never_resume_after_reset(response_controller):
    controller, now = response_controller
    prepared = controller.prepared
    ego = state_at(prepared, 1, speed=1.5)
    controller.control(ego, prepared.global_plan, 0.05, sensors=clear_scan())
    frame = clear_scan()
    frame.lidar_age_s = 0.501
    cmd = controller.control(ego, prepared.global_plan, 0.05, sensors=frame)
    assert cmd.acceleration < 0
    assert controller.diagnostics["response_aborted"]
    controller.reset()
    now[0] += 1
    cmd = controller.control(ego, prepared.global_plan, 0.05, sensors=clear_scan())
    assert cmd.acceleration < 0
    assert controller.diagnostics["response_aborted"]


@pytest.mark.parametrize("elapsed,progress", [(1.0, 1.1), (0.05, 6.0)])
def test_stale_tick_or_pose_jump_aborts_response(response_controller, elapsed, progress):
    controller, now = response_controller
    prepared = controller.prepared
    controller.control(state_at(prepared, 1, 1.5), prepared.global_plan,
                       0.05, sensors=clear_scan())
    now[0] += elapsed
    command = controller.control(state_at(prepared, progress, 1.5), prepared.global_plan,
                                 0.05, sensors=clear_scan())
    assert controller.experiment.phase == controller.experiment.ABORTED
    assert command.acceleration == controller.ego_min_acceleration
