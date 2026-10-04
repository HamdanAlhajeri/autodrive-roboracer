import math
from pathlib import Path

import pytest
import yaml

from avlite_autodrive.configuration import load_config
from avlite_roboracer.actuator_node import ActuatorLogic
from avlite_roboracer.command import CommandSource, DriveCommand, STOP
from avlite_roboracer.commissioning import analyze
from avlite_roboracer.profile import HardwareProfile
from avlite_roboracer.racing import (
    hardware_race_config, mapping_control_config, mapping_racing_config,
)
from avlite_roboracer.slam import effective_parameters
from roboracer_fixtures import PROFILE_PATH, ROOT, commissioned_profile

RACE = ROOT / "config" / "roboracer" / "race.yaml"
SLAM = ROOT / "config" / "roboracer" / "slam_toolbox.yaml"
MOVE = DriveCommand(0.1, 1.0, CommandSource.RACING)


def test_actuator_is_silent_until_commanded_and_holds_stop_after_expiry():
    logic = ActuatorLogic(commissioned_profile(), stop_hold_s=1.0)
    assert logic.tick(0.0) is None  # silent: driver teleop may use the topic
    logic.receive_command(STOP, 0.01, 0.0)
    assert logic.tick(0.01) is None
    logic.receive_command(MOVE, 0.01, 1.0)
    assert logic.tick(1.05) == pytest.approx((0.1, 1.0))
    logic.speed = 1.0
    assert logic.tick(1.30) == (0.0, 0.0)  # expired after 0.25 s: actively stop
    assert logic.tick(2.50) == (0.0, 0.0)  # hold elapsed but still moving
    logic.speed = 0.0
    assert logic.tick(2.60) == (0.0, 0.0)
    assert logic.tick(2.70) is None and logic.state == "idle"


def test_actuator_rejects_delayed_commands():
    logic = ActuatorLogic(commissioned_profile())
    logic.receive_command(MOVE, 0.4, 0.0)  # stamp older than the 0.25 s timeout
    logic.receive_command(MOVE, -0.5, 0.0)  # stamp from the future
    assert logic.tick(0.01) is None


def test_explicit_stop_transitions_to_stopping():
    logic = ActuatorLogic(commissioned_profile())
    logic.receive_command(MOVE, 0.0, 0.0)
    logic.tick(0.01)
    logic.receive_command(STOP, 0.0, 0.02)
    assert logic.tick(0.03) == (0.0, 0.0) and logic.state == "stopping"


def test_manual_override_wins_and_is_reported():
    logic = ActuatorLogic(commissioned_profile())
    logic.receive_command(MOVE, 0.0, 0.0)
    logic.receive_manual(-0.2, 0.5, 0.0)
    assert logic.tick(0.01) == pytest.approx((-0.2, 0.5))
    assert logic.status(0.01)["manual_override"]
    assert not logic.status(0.5)["manual_override"] or logic.tick(0.5) is not None
    logic.tick(0.5)  # manual input expired
    assert not logic.manual_override


def test_competing_publisher_latches_a_fault():
    logic = ActuatorLogic(commissioned_profile())
    logic.receive_command(MOVE, 0.0, 0.0)
    assert logic.tick(0.01, competing_publishers=1) == (0.0, 0.0)
    assert "another node" in logic.fault
    logic.receive_command(MOVE, 0.0, 0.02)
    assert logic.tick(0.03) == (0.0, 0.0)


def test_uncommissioned_actuator_never_publishes():
    logic = ActuatorLogic(HardwareProfile.load(PROFILE_PATH))
    logic.receive_command(MOVE, 0.0, 0.0)
    logic.receive_manual(0.1, 1.0, 0.0)
    assert logic.tick(0.01) is None
    assert "not commissioned" in logic.status(0.01)["fault"]


def test_race_config_takes_geometry_from_profile_and_respects_limits(tmp_path):
    profile = commissioned_profile()
    config = hardware_race_config(load_config(RACE), profile, tmp_path / "racemap.json")
    planning, control = config["planning"], config["c30_control"]
    assert planning["frame_id"] == "map"
    assert planning["wheelbase_m"] == control["c32_ego_distance_front_axle"] == 0.33
    assert planning["vehicle_radius_m"] == pytest.approx(math.hypot(0.15, 0.12))
    assert planning["max_velocity_mps"] == control["c32_ego_max_velocity"] == 1.5


@pytest.mark.parametrize("section, key, value, message", [
    ("planning", "max_velocity_mps", 2.5, "racing_speed_mps"),
    ("planning", "braking_deceleration_mps2", 2.5, "braking_deceleration"),
    ("planning", "acceleration_mps2", 2.0, "acceleration_mps2"),
    ("planning", "reaction_time_s", 0.05, "command latency"),
    ("c30_control", "c32_ego_max_steering", 0.5, "steering"),
    ("c30_control", "c32_ego_min_acceleration", -3.0, "braking deceleration"),
])
def test_race_config_rejects_values_beyond_measurements(tmp_path, section, key, value, message):
    config = load_config(RACE)
    config[section][key] = value
    with pytest.raises(ValueError, match=message):
        hardware_race_config(config, commissioned_profile(), tmp_path / "m.json")


def test_mapping_control_uses_mapping_speed():
    control = mapping_control_config(load_config(RACE), commissioned_profile())
    assert control["c35_cruise_velocity"] == control["c32_ego_max_velocity"] == 0.8
    with pytest.raises(ValueError, match="not commissioned"):
        mapping_control_config(load_config(RACE), HardwareProfile.load(PROFILE_PATH))


def test_mapping_stopping_model_uses_measured_braking_and_latency():
    profile = commissioned_profile(**{"measurements.braking_deceleration_mps2": 0.4,
                                      "measurements.command_latency_s": 0.6})
    racing = mapping_racing_config(load_config(RACE), profile)
    assert racing["braking_deceleration_mps2"] == 0.4
    assert racing["reaction_time_s"] == 0.6


def test_actuator_status_remains_serializable_with_invalid_odometry():
    import json
    logic = ActuatorLogic(commissioned_profile())
    logic.speed = math.inf
    assert json.loads(json.dumps(logic.status(0), allow_nan=False))["measured_speed_mps"] is None


def test_slam_parameters_follow_profile(tmp_path):
    profile = commissioned_profile(**{"frames.base_link": "base_footprint",
                                      "topics.scan": "/lidar/scan"})
    path = effective_parameters(SLAM, profile, "localization", tmp_path / "p.yaml",
                                map_file="/data/s1/posegraph", start_pose=(1.0, 2.0, 0.5),
                                use_sim_time=True)
    params = yaml.safe_load(Path(path).read_text())["slam_toolbox"]["ros__parameters"]
    assert params["base_frame"] == "base_footprint" and params["scan_topic"] == "/lidar/scan"
    assert params["mode"] == "localization" and params["use_sim_time"] is True
    assert params["map_file_name"] == "/data/s1/posegraph"
    assert params["map_start_pose"] == [1.0, 2.0, 0.5]
    assert params["enable_interactive_mode"] is False
    with pytest.raises(ValueError, match="pose graph"):
        effective_parameters(SLAM, profile, "localization", tmp_path / "q.yaml")


def test_commissioning_measures_latency_and_stopping():
    records = []
    t = 0.0
    speed, x = 0.0, 0.0
    commanded = 0.0
    events = [(1.0, 1.0), (5.0, 0.0)]  # step to 1 m/s at 1 s, stop at 5 s
    for i in range(800):
        t = i * 0.01
        commanded = next((v for at, v in reversed(events) if t >= at - 1e-9), 0.0)
        if i % 2 == 0:  # the command topic is published continuously at 50 Hz
            records.append({"kind": "drive", "rx_ns": int(t * 1e9), "stamp_ns": int(t * 1e9),
                            "speed": commanded, "steering": 0.0})
        # 0.1 s latency, 2 m/s^2 acceleration and braking.
        target = next((v for at, v in reversed(events) if t >= at + 0.1), 0.0)
        speed += max(-0.02, min(0.02, target - speed))
        x += speed * 0.01
        records.append({"kind": "odom", "rx_ns": int(t * 1e9), "stamp_ns": int(t * 1e9),
                        "vx": speed, "x": x, "y": 0.0})
    assert commanded == 0.0
    result = analyze(records)
    start, stop = result["starts"][0], result["stops"][0]
    assert start["latency_s"] == pytest.approx(0.15, abs=0.02)
    assert stop["initial_speed_mps"] == pytest.approx(1.0)
    assert stop["deceleration_mps2"] == pytest.approx(2.0, rel=0.05)
    # 0.1 s at 1 m/s plus 1^2/(2*2) = 0.35 m.
    assert stop["stopping_distance_m"] == pytest.approx(0.35, abs=0.03)
    suggestion = result["suggested_profile"]
    assert suggestion["measurements.stop_test_speed_mps"] == pytest.approx(1.0)
    assert math.isfinite(suggestion["measurements.command_latency_s"])
