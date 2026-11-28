import json
import shutil

import pytest
import yaml

from avlite_roboracer.doctor import inspect
from avlite_roboracer.profile import AUTONOMY_REQUIRED
from avlite_roboracer.profile_edit import (
    LOCALIZATION_STARTING_POINT, commissioning_updates, main, parse_tf_echo, parse_value,
    pressed_button, set_values)
from roboracer_fixtures import PROFILE_PATH

REPO_PROFILE = PROFILE_PATH  # the repository profile with its measurements cleared

# A complete, valid commissioning of a small car (invented numbers, for the checks only).
COMPLETE = {
    "mounts.laser.x_m": 0.27, "mounts.laser.y_m": 0.0, "mounts.laser.yaw_rad": 0.0,
    "vehicle.wheelbase_m": 0.33, "vehicle.width_m": 0.30, "vehicle.length_m": 0.55,
    "vehicle.rear_axle_to_front_m": 0.45,
    "steering.max_angle_rad": 0.35, "steering.positive_left": True,
    "motor.max_command_speed_mps": 1.0,
    "measurements.command_latency_s": 0.12, "measurements.braking_deceleration_mps2": 2.0,
    "measurements.max_acceleration_mps2": 1.5, "measurements.stop_test_speed_mps": 1.0,
    "measurements.stopping_distance_m": 0.4,
    "limits.mapping_speed_mps": 0.6, "limits.racing_speed_mps": 1.0, "deadman.button": 4,
    **LOCALIZATION_STARTING_POINT,
}


@pytest.fixture
def profile(tmp_path):
    path = tmp_path / "hardware.yaml"
    shutil.copy(REPO_PROFILE, path)
    return path


def test_every_autonomy_key_can_be_written_and_the_profile_becomes_commissioned(profile):
    assert set(COMPLETE) == set(AUTONOMY_REQUIRED)
    changes = set_values(profile, COMPLETE)
    assert len(changes) == len(AUTONOMY_REQUIRED)
    report = inspect(profile)
    assert report["commissioned"] is True and report["missing"] == []
    data = yaml.safe_load(profile.read_text())
    assert data["mounts"]["laser"] == {"x_m": 0.27, "y_m": 0.0, "yaw_rad": 0.0}
    assert data["steering"]["positive_left"] is True and data["deadman"]["button"] == 4


def test_comments_and_other_lines_survive_and_a_backup_is_kept(profile):
    before = profile.read_text()
    set_values(profile, {"vehicle.wheelbase_m": 0.33, "mounts.laser.x_m": 0.27})
    after = profile.read_text()
    assert "# Never copy simulator values into this file" in after
    assert "deadman:" in after and "Find the index with" in after
    changed = [(a, b) for a, b in zip(before.splitlines(), after.splitlines()) if a != b]
    assert len(changed) == 2  # only the two edited lines differ
    assert (profile.parent / "hardware.yaml.bak").read_text() == before
    assert "  wheelbase_m: 0.33" in after and "x_m: 0.27" in after


def test_unknown_keys_bad_values_and_invalid_results_change_nothing(profile):
    before = profile.read_bytes()
    with pytest.raises(ValueError, match="not a setting"):
        set_values(profile, {"frames.map": 3})
    with pytest.raises(ValueError, match="not a setting"):
        set_values(profile, {"vehicle.colour": 1})
    with pytest.raises(ValueError, match="invalid"):
        set_values(profile, {"vehicle.wheelbase_m": -0.3})
    with pytest.raises(ValueError, match="invalid"):
        set_values(profile, {"limits.mapping_speed_mps": 1.2, "limits.racing_speed_mps": 0.8})
    with pytest.raises(ValueError, match="geometry"):
        set_values(profile, {"vehicle.wheelbase_m": 0.6, "vehicle.length_m": 0.5,
                             "vehicle.rear_axle_to_front_m": 0.55})
    with pytest.raises(ValueError, match="invalid"):
        set_values(profile, {"localization.min_scan_alignment": 1.5})
    with pytest.raises(ValueError, match="invalid"):
        set_values(profile, {"deadman.button": -1})
    assert profile.read_bytes() == before
    assert not (profile.parent / "hardware.yaml.bak").exists()


def test_speeds_above_1_5_need_an_explicit_flag(profile):
    set_values(profile, {"motor.max_command_speed_mps": 3.0,
                         "measurements.stop_test_speed_mps": 3.0,
                         "limits.racing_speed_mps": 1.0}, allow_fast=True)
    with pytest.raises(ValueError, match="allow-fast"):
        set_values(profile, {"limits.racing_speed_mps": 2.5})
    set_values(profile, {"limits.racing_speed_mps": 2.5}, allow_fast=True)


def test_dry_run_writes_nothing_and_windows_line_endings_are_kept(profile):
    set_values(profile, {"vehicle.width_m": 0.3}, dry_run=True)
    assert "width_m: null" in profile.read_text()
    crlf = profile.read_text().replace("\n", "\r\n")
    profile.write_bytes(crlf.encode())
    set_values(profile, {"vehicle.width_m": 0.3})
    raw = profile.read_bytes()
    assert b"width_m: 0.3" in raw and raw.count(b"\r\n") == crlf.count("\r\n")
    assert b"\n" not in raw.replace(b"\r\n", b"")


def test_values_can_be_set_back_to_null_and_parse_value_is_strict():
    assert parse_value("true") is True and parse_value("False") is False
    assert parse_value("null") is None and parse_value("4") == 4 and parse_value("0.25") == 0.25
    for bad in ("nan", "inf", "fast", ""):
        with pytest.raises(ValueError):
            parse_value(bad)


def test_cli_set_reports_each_change_and_rejects_cleanly(profile, capsys):
    args = ["--profile", str(profile), "set", "vehicle.wheelbase_m=0.33", "deadman.button=4"]
    assert main(args) == 0
    out = capsys.readouterr().out
    assert "set vehicle.wheelbase_m: null -> 0.33" in out and "set deadman.button: null -> 4" in out
    assert main(["--profile", str(profile), "set", "vehicle.wheelbase_m=-1"]) == 1
    assert "Not changed" in capsys.readouterr().err
    assert main(["--profile", str(profile), "set", "vehicle.wheelbase_m"]) == 1
    assert main(["--profile", str(profile), "defaults-localization"]) == 0
    assert inspect(profile)["present"]["localization.min_scan_alignment"] == 0.7


def test_commissioning_results_are_applied_but_only_measured_numbers(profile, tmp_path):
    result = tmp_path / "stops.json"
    result.write_text(json.dumps({"suggested_profile": {
        "measurements.command_latency_s": 0.11, "measurements.braking_deceleration_mps2": 1.8,
        "measurements.max_acceleration_mps2": None, "measurements.stop_test_speed_mps": 1.0,
        "measurements.stopping_distance_m": 0.45, "stop_events": 3}}))
    updates = commissioning_updates(result)
    assert set(updates) == {"measurements.command_latency_s",
                            "measurements.braking_deceleration_mps2",
                            "measurements.stop_test_speed_mps",
                            "measurements.stopping_distance_m"}
    assert main(["--profile", str(profile), "commissioning", str(result)]) == 0
    assert inspect(profile)["present"]["measurements.stopping_distance_m"] == 0.45


def test_the_laser_transform_and_dead_man_button_are_read_from_ros_output():
    sample = """At time 0.0
- Translation: [0.270, -0.010, 0.100]
- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
- Rotation: in RPY (radian) [0.000, -0.000, 1.571]
- Rotation: in RPY (degree) [0.000, -0.000, 90.000]
"""
    x, y, yaw = parse_tf_echo(sample)
    assert (x, y) == (0.27, -0.01) and yaw == pytest.approx(1.571)
    with pytest.raises(ValueError, match="bringup"):
        parse_tf_echo("Invalid frame ID laser passed to canTransform")
    assert pressed_button([0, 0, 0, 0, 0], [0, 0, 0, 0, 1]) == 4
    for before, after in (([0, 0], [0, 0]), ([0, 0], [1, 1]), ([1, 0], [1, 0])):
        with pytest.raises(ValueError, match="exactly one"):
            pressed_button(before, after)
