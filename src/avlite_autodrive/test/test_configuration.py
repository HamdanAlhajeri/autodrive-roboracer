"""Shared settings reach both profiles and invalid inputs fail before startup."""

import pytest

from avlite_autodrive.configuration import load_config


@pytest.fixture
def profiles(tmp_path):
    (tmp_path / "driving.yaml").write_text("speed_mps: 5\nmax_throttle: 0.2\n")
    (tmp_path / "avlite.yaml").write_text(
        "shared_settings: driving.yaml\n"
        "c30_control:\n"
        "  c32_ego_max_velocity: ${speed_mps}\n"
        "  c35_cruise_velocity: ${speed_mps}\n"
        "  c35_bubble_radius: 0.24\n"
    )
    (tmp_path / "actuator.yaml").write_text(
        "shared_settings: driving.yaml\n"
        "avlite_actuator_adapter:\n"
        "  ros__parameters:\n"
        "    max_speed: ${speed_mps}\n"
        "    max_throttle: ${max_throttle}\n"
    )
    return tmp_path


def test_editing_one_file_updates_both_profiles(profiles, monkeypatch, tmp_path):
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    for speed, throttle in [(5, 0.2), (0.75, 0.03)]:
        (profiles / "driving.yaml").write_text(
            f"speed_mps: {speed}\nmax_throttle: {throttle}\n"
        )
        avlite = load_config(profiles / "avlite.yaml")["c30_control"]
        actuator = load_config(profiles / "actuator.yaml")
        parameters = actuator["avlite_actuator_adapter"]["ros__parameters"]
        assert avlite["c32_ego_max_velocity"] == avlite["c35_cruise_velocity"] == speed
        assert parameters["max_speed"] == speed
        assert isinstance(parameters["max_speed"], float)  # ROS expects a double, even for 5.
        assert parameters["max_throttle"] == throttle
        assert avlite["c35_bubble_radius"] == 0.24
    assert "${speed_mps}" in (profiles / "avlite.yaml").read_text()


@pytest.mark.parametrize(
    "settings",
    [
        "speed_mps: 0\nmax_throttle: 0.2",
        "speed_mps: -1\nmax_throttle: 0.2",
        "speed_mps: .nan\nmax_throttle: 0.2",
        "speed_mps: .inf\nmax_throttle: 0.2",
        "speed_mps: true\nmax_throttle: 0.2",
        "speed_mps: '5'\nmax_throttle: 0.2",
        "speed_mps: 5\nmax_throttle: 1.1",
        "speed_mps: 5\nmax_throttle: 0",
        "speed_mps: 5",  # No silent fallback if a shared setting is missing.
        "speed_mps: 5\nmax_throtle: 0.2",
    ],
)
def test_invalid_shared_values_reject_both_profiles(profiles, settings):
    (profiles / "driving.yaml").write_text(settings)
    for name in ("avlite.yaml", "actuator.yaml"):
        with pytest.raises(ValueError):
            load_config(profiles / name)


def test_missing_file_and_unknown_reference_fail(profiles):
    profile = profiles / "avlite.yaml"
    profile.write_text(profile.read_text().replace("${speed_mps}", "${speed_typo}"))
    with pytest.raises(ValueError, match="unknown shared reference"):
        load_config(profile)
    (profiles / "driving.yaml").unlink()
    with pytest.raises(FileNotFoundError):
        load_config(profiles / "actuator.yaml")


def test_numeric_profiles_still_work(tmp_path):
    profile = tmp_path / "numeric.yaml"
    profile.write_text("c30_control:\n  c35_cruise_velocity: 0.5\n")
    assert load_config(profile) == {"c30_control": {"c35_cruise_velocity": 0.5}}
