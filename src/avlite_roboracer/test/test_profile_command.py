import math

import pytest

from avlite_roboracer.command import (
    STOP, CommandArbiter, CommandSource, DriveCommand, ExpiryWatchdog, SpeedDemand, to_hardware,
)
from avlite_roboracer.profile import AUTONOMY_REQUIRED, HardwareProfile
from roboracer_fixtures import PROFILE_PATH, commissioned_data, commissioned_profile, raw_profile


def test_repository_profile_loads_but_refuses_autonomy_until_measured():
    profile = HardwareProfile.load(PROFILE_PATH)
    assert set(profile.missing_for_autonomy()) == set(AUTONOMY_REQUIRED)
    with pytest.raises(ValueError, match="not commissioned"):
        profile.require_commissioned()
    assert profile.laser_mount is None


def test_commissioned_profile_and_stopping_prediction():
    profile = commissioned_profile()
    profile.require_commissioned()
    assert profile.laser_mount == (0.25, 0.0, 0.0)
    assert profile.stopping_distance(2.0) == pytest.approx(2.0 * 0.08 + 4.0 / 4.0)
    assert len(profile.sha256) == 64


@pytest.mark.parametrize("overrides, message", [
    ({"limits.mapping_speed_mps": 2.5}, "mapping_speed_mps cannot exceed"),
    ({"limits.racing_speed_mps": 2.8}, "stop-test speed"),
    ({"limits.racing_speed_mps": 3.5, "measurements.stop_test_speed_mps": 4.0},
     "exceeds motor"),
    ({"steering.max_angle_rad": 2.0}, "below pi/2"),
    ({"vehicle.wheelbase_m": -1}, "positive"),
    ({"steering.positive_left": "yes"}, "positive_left"),
    ({"localization.min_scan_alignment": 1.5}, "min_scan_alignment"),
    ({"motor.stop_command_speed_mps": 0.2}, "must be 0"),
])
def test_profile_rejects_inconsistent_values(overrides, message):
    with pytest.raises(ValueError, match=message):
        HardwareProfile(commissioned_data(**overrides))


def test_profile_requires_recording_interfaces():
    data = raw_profile()
    data["topics"]["scan"] = None
    with pytest.raises(ValueError, match="topics.scan"):
        HardwareProfile(data)


def test_single_owner_and_expiry():
    arbiter = CommandArbiter(timeout_s=0.25)
    assert not arbiter.submit(CommandSource.MAPPING, 0.1, 1.0, 0.0)  # nobody owns it yet
    arbiter.select(CommandSource.MAPPING)
    assert arbiter.submit(CommandSource.MAPPING, 0.1, 1.0, 0.0)
    assert not arbiter.submit(CommandSource.RACING, 0.0, 3.0, 0.05)
    assert not arbiter.submit(CommandSource.MANUAL, 0.0, 3.0, 0.05)
    assert arbiter.output(0.1) == DriveCommand(0.1, 1.0, CommandSource.MAPPING)
    assert arbiter.output(0.3) == STOP  # expired: rejected sources never refreshed it
    assert arbiter.rejected == 3


def test_ownership_change_discards_previous_command():
    arbiter = CommandArbiter(timeout_s=1.0)
    arbiter.select("mapping")
    arbiter.submit("mapping", 0.0, 1.0, 0.0)
    arbiter.select("racing")
    assert arbiter.output(0.1) == STOP
    arbiter.release()
    assert not arbiter.submit("racing", 0.0, 1.0, 0.2)


@pytest.mark.parametrize("steering, speed", [(math.nan, 1.0), (0.0, math.inf), (0.0, -0.5)])
def test_arbiter_rejects_invalid_commands(steering, speed):
    arbiter = CommandArbiter(timeout_s=1.0)
    arbiter.select("racing")
    assert not arbiter.submit("racing", steering, speed, 0.0)
    assert arbiter.output(0.0) == STOP


def test_watchdog_expires_independently():
    watchdog = ExpiryWatchdog(0.25)
    assert watchdog.output(0.0) == STOP and watchdog.expired
    command = DriveCommand(0.1, 1.0, CommandSource.RACING)
    watchdog.feed(command, 1.0)
    assert watchdog.output(1.2) == command and not watchdog.expired
    assert watchdog.output(1.26) == STOP and watchdog.expired
    watchdog.feed(command, 2.0)
    assert watchdog.output(1.9) == STOP  # clock went backwards: never trusted


def test_speed_demand_braking_cannot_restore_stale_demand():
    demand = SpeedDemand(limit_mps=2.0)
    for _ in range(40):
        demand.update(1.0, 0.0, 0.05)
    assert demand.demand_mps == pytest.approx(2.0)
    # Car only reached 0.5 m/s; braking caps demand there, so it cannot launch back to 2.
    assert demand.update(-1.0, 0.5, 0.05) == pytest.approx(0.45)
    assert demand.update(1.0, 0.45, 0.05) == pytest.approx(0.5)
    assert demand.update(math.nan, 0.0, 0.05) == 0.0


def test_hardware_conversion_clamps_and_applies_sign():
    profile = commissioned_profile(**{"steering.positive_left": False,
                                      "steering.center_offset_rad": 0.02})
    steering, speed = to_hardware(DriveCommand(1.0, 5.0, CommandSource.RACING), profile)
    assert steering == pytest.approx(-0.40 + 0.02)
    assert speed == 3.0
    assert to_hardware(STOP, profile)[1] == 0.0


def test_hardware_conversion_refuses_uncommissioned_profile():
    with pytest.raises(ValueError, match="not commissioned"):
        to_hardware(STOP, HardwareProfile.load(PROFILE_PATH))
