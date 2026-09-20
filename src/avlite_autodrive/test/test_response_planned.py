"""Check actual pinned AVLite steering/guards on the bundled practice map."""

from pathlib import Path

import pytest
from avlite.c30_control.c39_settings import ControlSettings
from avlite.c50_common.c52_world_sensor_datatypes import Lidar

from avlite_autodrive.configuration import load_config
from avlite_autodrive.race_planning import prepare_plan
from avlite_autodrive.response_test import measurement_config
from avlite_autodrive.plugin.planned_controller import AutoDRIVEPlannedController
from avlite_autodrive.plugin.response_controller import ResponseController
from test_race_planning import clear_scan, state_at


@pytest.mark.parametrize("speed", [1.5, 2.0, 2.5])
def test_coasting_on_real_plan_preserves_steering_and_invalid_pose_stop(monkeypatch, speed):
    config_dir = Path("/config") if Path("/config/avlite.yaml").exists() else Path(__file__).resolve().parents[3] / "config"
    profile = config_dir / "avlite.yaml"
    config = measurement_config(load_config(profile), speed)
    for key, value in config["c30_control"].items():
        monkeypatch.setattr(ControlSettings, key, value)
    prepared = prepare_plan(config, profile)
    controller = ResponseController(prepared, speed, 3)
    ordinary = AutoDRIVEPlannedController(prepared)
    sensors = clear_scan()
    sensors.lidar_sensor = Lidar()
    # Find a point with enough straight road and a normal target at test speed.
    for s in prepared.path.s[:-1]:
        ego = state_at(prepared, s, speed)
        expected = ordinary.control(ego, prepared.global_plan, 0.05, sensors=sensors)
        if (controller.experiment.straight_ahead(s)
                and abs(expected.steer) <= controller.experiment.steer_limit
                and ordinary.diagnostics["target_velocity_mps"] >= speed - 0.01):
            break
    else:
        pytest.fail("Practice map has no usable measurement location")
    command = controller.control(ego, prepared.global_plan, 0.05, sensors=sensors)
    assert controller.diagnostics["response_phase"] == 1
    assert command.steer == pytest.approx(expected.steer)
    assert command.acceleration <= -0.2
    ego.x = 100
    command = controller.control(ego, prepared.global_plan, 0.05, sensors=sensors)
    assert controller.diagnostics["plan_valid"] == 0
    assert command.acceleration == controller.ego_min_acceleration
