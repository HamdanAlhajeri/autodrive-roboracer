import math
from pathlib import Path

import pytest
import yaml

from avlite_autodrive.configuration import load_config
from avlite_roboracer.actuator_node import ActuatorLogic
from avlite_roboracer.command import CommandSource, DriveCommand, STOP
from avlite_roboracer.commissioning import analyze
from avlite_roboracer.profile import HardwareProfile
from avlite_roboracer.readiness import actuator_failures
from avlite_roboracer.racing import (
    hardware_race_config, mapping_control_config, mapping_racing_config,
)
from avlite_roboracer.slam import effective_parameters
from roboracer_fixtures import PROFILE_PATH, ROOT, commissioned_profile

RACE = ROOT / "config" / "roboracer" / "race.yaml"
SLAM = ROOT / "config" / "roboracer" / "slam_toolbox.yaml"
MOVE = DriveCommand(0.1, 1.0, CommandSource.RACING)
HELD = [0, 0, 0, 0, 0, 1, 0, 0]      # fixture dead-man button 5 pressed
RELEASED = [0] * 8


def test_actuator_is_silent_until_commanded_and_holds_stop_after_expiry():
    logic = ActuatorLogic(commissioned_profile(**{"deadman.stop_on_press": False}), stop_hold_s=1.0)
    assert logic.tick(0.0) is None  # silent: driver teleop may use the topic
    logic.receive_command(STOP, 0.01, 0.0)
    assert logic.tick(0.01) is None
    logic.receive_command(MOVE, 0.01, 1.0)
    logic.receive_joystick(HELD, 1.0)
    assert logic.tick(1.05) == pytest.approx((0.1, 1.0))
    logic.speed = 1.0
    assert logic.tick(1.30) == (0.0, 0.0)  # expired after 0.25 s: actively stop
    assert logic.tick(2.50) == (0.0, 0.0)  # hold elapsed but still moving
    logic.speed = 0.0
    assert logic.tick(2.60) == (0.0, 0.0)
    assert logic.tick(2.70) is None and logic.state == "idle"


def test_actuator_rejects_delayed_commands():
    logic = hold_logic()
    logic.receive_command(MOVE, 0.4, 0.0)  # stamp older than the 0.25 s timeout
    logic.receive_command(MOVE, -0.5, 0.0)  # stamp from the future
    assert logic.tick(0.01) is None


def test_explicit_stop_transitions_to_stopping():
    logic = hold_logic()
    logic.receive_command(MOVE, 0.0, 0.0)
    logic.tick(0.01)
    logic.receive_command(STOP, 0.0, 0.02)
    assert logic.tick(0.03) == (0.0, 0.0) and logic.state == "stopping"


def test_manual_override_wins_and_is_reported():
    logic = hold_logic()
    logic.receive_command(MOVE, 0.0, 0.0)
    logic.receive_manual(-0.2, 0.5, 0.0)
    assert logic.tick(0.01) == pytest.approx((-0.2, 0.5))
    assert logic.status(0.01)["manual_override"]
    assert not logic.status(0.5)["manual_override"] or logic.tick(0.5) is not None
    logic.tick(0.5)  # manual input expired
    assert not logic.manual_override


def test_competing_publisher_latches_a_fault():
    logic = hold_logic()
    logic.receive_command(MOVE, 0.0, 0.0)
    assert logic.tick(0.01, competing_publishers=1) == (0.0, 0.0)
    assert "another node" in logic.fault
    logic.receive_command(MOVE, 0.0, 0.02)
    assert logic.tick(0.03) == (0.0, 0.0)


def test_releasing_the_deadman_stops_and_blocks_restart():
    logic = hold_logic()
    logic.receive_joystick(HELD, 0.0)
    logic.receive_command(MOVE, 0.0, 0.0)
    assert logic.tick(0.01) == pytest.approx((0.1, 1.0))
    logic.receive_joystick(RELEASED, 0.02)
    logic.receive_command(MOVE, 0.0, 0.02)
    assert logic.tick(0.03) == (0.0, 0.0) and logic.state == "stopping"
    logic.receive_command(MOVE, 0.0, 0.04)  # fresh commands alone cannot resume
    assert logic.tick(0.05) == (0.0, 0.0)
    assert not logic.status(0.05)["deadman_held"]


def test_silent_joystick_stops_even_with_fresh_commands():
    logic = hold_logic()
    logic.receive_joystick(HELD, 0.0)  # e.g. last message before the pad lost its link
    for step in range(1, 30):
        now = step * 0.01
        logic.receive_command(MOVE, 0.0, now)
        expected = (0.0, 0.0) if now > 0.25 else pytest.approx((0.1, 1.0))
        assert logic.tick(now) == expected


def test_deadman_button_out_of_range_counts_as_released():
    logic = hold_logic()
    logic.receive_joystick([1, 1], 0.0)  # pad reports fewer buttons than the index
    logic.receive_command(MOVE, 0.0, 0.0)
    logic.state = "active"
    assert logic.tick(0.01) == (0.0, 0.0)


def test_autonomy_requires_a_deadman_button():
    profile = commissioned_profile(**{"deadman.button": None})
    assert "deadman.button" in profile.missing_for_autonomy()
    for bad in (-1, True, 1.5, "5"):
        with pytest.raises(ValueError, match="deadman.button"):
            commissioned_profile(**{"deadman.button": bad})


def test_moving_session_requires_the_deadman_but_ready_does_not():
    status = (0.0, {"fault": None, "manual_override": False, "deadman_held": False})
    assert actuator_failures(status, 0.1, 0.5) == []
    assert actuator_failures(status, 0.1, 0.5, require_deadman=True) == [
        "dead-man button released or joystick silent"]
    held = (0.0, {**status[1], "deadman_held": True})
    assert actuator_failures(held, 0.1, 0.5, require_deadman=True) == []


def test_uncommissioned_actuator_never_publishes():
    logic = ActuatorLogic(HardwareProfile.load(PROFILE_PATH))
    logic.receive_command(MOVE, 0.0, 0.0)
    logic.receive_manual(0.1, 1.0, 0.0)
    assert logic.tick(0.01) is None
    assert "not commissioned" in logic.status(0.01)["fault"]


def test_committed_profile_names_f1tenth_interfaces_but_stays_uncommissioned():
    profile = HardwareProfile.load(PROFILE_PATH)
    assert [profile.topic(name) for name in ("scan", "odom", "drive_output", "manual_input")] == [
        "/scan", "/odom", "/drive", "/teleop"]
    assert [profile.frame(name) for name in ("odom", "base_link", "laser")] == [
        "odom", "base_link", "laser"]
    # Interface defaults are not measurements: autonomy stays refused.
    assert "vehicle.wheelbase_m" in profile.missing_for_autonomy()
    with pytest.raises(ValueError, match="not commissioned"):
        profile.require_commissioned()
    assert load_config(RACE)["planning"]["acceleration_feedforward"] is False


def test_race_config_takes_geometry_from_profile_and_respects_limits(tmp_path):
    profile = commissioned_profile()
    config = hardware_race_config(load_config(RACE), profile, tmp_path / "racemap.json")
    planning, control = config["planning"], config["c30_control"]
    assert planning["frame_id"] == "map"
    assert planning["wheelbase_m"] == control["c32_ego_distance_front_axle"] == 0.33
    assert planning["vehicle_radius_m"] == pytest.approx(math.hypot(0.15, 0.12))
    assert planning["max_velocity_mps"] == control["c32_ego_max_velocity"] == 1.0


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
    logic = hold_logic()
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
    # The car speeds up at 2 m/s^2 once it responds (the 0.02 per 10 ms step in the simulation
    # above).
    assert start["acceleration_mps2"] == pytest.approx(2.0, rel=0.1)
    assert suggestion["measurements.max_acceleration_mps2"] == pytest.approx(2.0, rel=0.1)


def _teleop_records(events, seconds=20.0, latency=0.1):
    """Records for a car that follows the commanded speed (2 m/s^2, 0.1 s latency)."""
    records, speed, x = [], 0.0, 0.0
    for i in range(int(seconds * 100)):
        t = i * 0.01
        if i % 2 == 0:
            commanded = next((v for at, v in reversed(events) if t >= at - 1e-9), 0.0)
            records.append({"kind": "drive", "rx_ns": int(t * 1e9), "stamp_ns": int(t * 1e9),
                            "speed": commanded, "steering": 0.0})
        target = next((v for at, v in reversed(events) if t >= at + latency), 0.0)
        speed += max(-0.02, min(0.02, target - speed))
        x += speed * 0.01
        records.append({"kind": "odom", "rx_ns": int(t * 1e9), "stamp_ns": int(t * 1e9),
                        "vx": speed, "x": x, "y": 0.0})
    return records


def test_commissioning_ignores_stick_chatter_and_starts_while_rolling():
    # Clean push 1 to 4 s. Second push at 8 s, released at 10 s, but the driver pushes again at
    # 10.2 s while the car is still rolling (an interrupted stop, then a start from motion).
    events = [(1.0, 1.0), (4.0, 0.0), (8.0, 1.0), (10.0, 0.0), (10.2, 1.0), (13.0, 0.0)]
    result = analyze(_teleop_records(events))
    flags = [s["interrupted"] for s in result["stops"]]
    assert flags == [False, True, False]
    assert [s["from_rest"] for s in result["starts"]] == [True, True, False]
    suggestion = result["suggested_profile"]
    # Only clean events count: the braking value is the clean 2 m/s^2, not the interrupted stop's.
    assert suggestion["measurements.braking_deceleration_mps2"] == pytest.approx(2.0, rel=0.1)
    assert suggestion["measurements.command_latency_s"] < 0.3
    assert suggestion["stop_events"] == 2


def test_commissioning_with_only_chatter_suggests_no_braking_values():
    events = [(1.0, 1.0), (3.0, 0.0), (3.2, 1.0), (5.0, 0.0), (5.2, 1.0)]
    result = analyze(_teleop_records(events, seconds=6.0))
    suggestion = result["suggested_profile"]
    assert all(s["interrupted"] for s in result["stops"])
    assert "measurements.braking_deceleration_mps2" not in suggestion
    assert "measurements.stopping_distance_m" not in suggestion


def test_deadman_fault_says_whether_the_button_was_released_or_the_joystick_went_silent():
    logic = hold_logic()
    logic.receive_joystick(HELD, 1.0)
    logic.receive_joystick(HELD, 1.1)
    status = logic.status(1.2)
    assert status["deadman_held"] and status["deadman_held_for_s"] == pytest.approx(0.2)
    assert status["joy_messages"] == 2 and status["joy_releases"] == 0
    # Silent: no further messages for longer than the timeout.
    silent = actuator_failures((0.0, logic.status(1.6)), 0.1, 0.5, require_deadman=True)
    assert len(silent) == 1 and "last /joy message 0.50 s ago" in silent[0]
    assert "button pressed" in silent[0] and "0 with the button not pressed" in silent[0]
    # Released: the last message had the button up.
    logic.receive_joystick(RELEASED, 1.3)
    released = actuator_failures((0.0, logic.status(1.31)), 0.1, 0.5, require_deadman=True)
    assert "button NOT pressed" in released[0] and "1 with the button not pressed" in released[0]
    assert "last release 0.01 s ago" in released[0]


def hold_logic():
    """The hold-the-button mode (stop_on_press false); the shipped hardware.yaml uses the other."""
    return ActuatorLogic(commissioned_profile(**{"deadman.stop_on_press": False}))


def stop_on_press_logic():
    return ActuatorLogic(commissioned_profile(**{"deadman.stop_on_press": True}))


def test_stop_on_press_runs_without_holding_the_button_while_the_joystick_publishes():
    logic = stop_on_press_logic()
    for step in range(0, 40):
        now = step * 0.01
        logic.receive_joystick(RELEASED, now)  # pad publishes, button up
        logic.receive_command(MOVE, 0.0, now)
        assert logic.tick(now) == pytest.approx((0.1, 1.0))
    assert logic.status(0.39)["deadman_held"] and not logic.status(0.39)["stop_latched"]


def test_stop_on_press_pressing_the_button_stops_at_once_and_stays_stopped():
    logic = stop_on_press_logic()
    logic.receive_joystick(RELEASED, 0.0)
    logic.receive_command(MOVE, 0.0, 0.0)
    assert logic.tick(0.01) == pytest.approx((0.1, 1.0))
    logic.receive_joystick(HELD, 0.02)   # the press
    logic.receive_joystick(RELEASED, 0.03)  # released again straight away
    logic.receive_command(MOVE, 0.0, 0.03)  # the supervisor has not noticed yet
    assert logic.tick(0.04) == (0.0, 0.0) and logic.state == "stopping"
    assert logic.status(0.04)["stop_latched"] and not logic.status(0.04)["deadman_held"]
    # While the supervisor keeps sending motion, releasing the button does not resume.
    for step in range(5, 20):
        now = step * 0.01
        logic.receive_joystick(RELEASED, now)
        logic.receive_command(MOVE, 0.0, now)
        assert logic.tick(now) == (0.0, 0.0)
    assert logic.status(0.19)["stop_latched"]


def test_stop_on_press_latch_clears_only_after_motion_commands_stop():
    logic = stop_on_press_logic()
    logic.receive_joystick(RELEASED, 0.0)
    logic.receive_command(MOVE, 0.0, 0.0)
    logic.tick(0.01)
    logic.receive_joystick(HELD, 0.02)
    logic.tick(0.03)
    assert logic.stop_latched
    # The button is still down: the latch holds even with no commands.
    logic.receive_joystick(HELD, 0.50)
    logic.tick(0.60)
    assert logic.stop_latched
    # Button up and no motion command for longer than the timeout: cleared.
    logic.receive_joystick(RELEASED, 0.70)
    logic.tick(0.71)
    assert not logic.stop_latched and logic.status(0.71)["deadman_held"]
    logic.receive_command(MOVE, 0.0, 0.72)  # a new session may now move
    logic.receive_joystick(RELEASED, 0.72)
    assert logic.tick(0.73) == pytest.approx((0.1, 1.0)) or logic.state == "stopping"


def test_stop_on_press_still_stops_when_the_joystick_goes_silent():
    logic = stop_on_press_logic()
    logic.receive_joystick(RELEASED, 0.0)
    for step in range(1, 40):
        now = step * 0.01
        logic.receive_command(MOVE, 0.0, now)
        expected = (0.0, 0.0) if now > 0.25 else pytest.approx((0.1, 1.0))
        assert logic.tick(now) == expected
    assert not logic.status(0.39)["deadman_held"]


def test_stop_on_press_needs_a_joystick_message_before_it_moves_at_all():
    logic = stop_on_press_logic()
    logic.receive_command(MOVE, 0.0, 0.0)
    logic.state = "active"
    assert logic.tick(0.01) == (0.0, 0.0)  # no /joy yet: refused


def test_stop_on_press_is_reported_as_a_stop_not_a_release():
    status = (0.0, {"fault": None, "manual_override": False, "deadman_held": False,
                    "stop_latched": True})
    assert actuator_failures(status, 0.1, 0.5, require_deadman=True) == ["stop button pressed"]


def test_stop_on_press_profile_value_must_be_true_or_false():
    assert commissioned_profile(**{"deadman.stop_on_press": True}).get("deadman.stop_on_press")
    with pytest.raises(ValueError, match="stop_on_press"):
        commissioned_profile(**{"deadman.stop_on_press": "yes"})


def test_the_shipped_hardware_profile_stops_on_press_instead_of_requiring_a_hold():
    assert commissioned_profile().get("deadman.stop_on_press") is True
