import json
import math

import pytest

from avlite_autodrive.actuation import Actuation, Limits


def ready(control, now=10, speed=0, steering=0, acceleration=1):
    control.receive_speed(speed, now)
    control.scan_time = now
    control.receive_command(steering, acceleration, now)


def test_startup_and_each_watchdog():
    c = Actuation()
    assert c.tick(10, 0.05) == (0, 0)
    for missing in ("command_time", "scan_time", "odom_time"):
        ready(c)
        setattr(c, missing, 9)
        assert c.tick(10, 0.05) == (0, 0)


def test_steering_units_and_saturation():
    c = Actuation()
    for angle, expected in ((0, 0), (math.radians(15), 0.5), (-math.pi, -1), (math.pi, 1)):
        ready(c, steering=angle)
        assert c.tick(10, 0.05)[1] == pytest.approx(expected)


def test_acceleration_is_integrated_not_used_as_throttle():
    c = Actuation()
    ready(c, acceleration=1)
    throttle, _ = c.tick(10, 0.05)
    assert c.target_speed == pytest.approx(0.05)
    assert 0 < throttle < 0.1
    for i in range(200):
        ready(c, now=10 + i * 0.05)
        throttle, _ = c.tick(10 + i * 0.05, 0.05)
        assert 0 <= throttle <= c.limits.max_throttle
    assert c.target_speed == c.limits.max_speed


def test_large_acceleration_commands_still_use_existing_limits():
    c = Actuation(Limits(max_speed=5.0, max_throttle=0.2))
    ready(c, acceleration=100)
    c.tick(10, 0.05)
    assert c.target_speed == pytest.approx(0.1)  # +2 m/s² limit.
    ready(c, speed=3.0, acceleration=-100)
    assert c.tick(10, 0.05)[0] == 0
    assert c.target_speed == pytest.approx(2.85)  # -3 m/s² limit.


def test_braking_never_reverses_and_overspeed_cuts_throttle():
    c = Actuation()
    ready(c, acceleration=-2, speed=0.5)
    assert c.tick(10, 0.05)[0] == 0
    ready(c, acceleration=1, speed=2)
    assert c.tick(10, 0.05)[0] == 0


def test_corner_deceleration_cuts_feedforward_and_keeps_steering():
    c = Actuation(Limits(max_speed=5.0, max_throttle=0.2))
    c.target_speed = 5.0
    c.integral = 0.8
    ready(c, speed=4.3, acceleration=-2.0, steering=math.radians(29))
    throttle, steering = c.tick(10, 0.05)
    assert throttle == 0.0  # Old feedforward must not fight this deceleration.
    assert steering == pytest.approx(29 / 30)
    assert c.target_speed == pytest.approx(4.2)
    assert c.integral == 0.0
    assert c.diagnostics["braking_requested"] is True
    assert c.diagnostics["throttle_saturated"] is False
    assert c.diagnostics["actuator_target_speed_mps"] == pytest.approx(4.2)


def test_braking_tracks_feedback_and_resumes_without_old_demand():
    c = Actuation(Limits(max_speed=5.0, max_throttle=0.2))
    c.target_speed = 5.0
    for i, speed in enumerate((4.3, 3.0, 2.0)):
        now = 10 + i * 0.05
        ready(c, now=now, speed=speed, acceleration=-2.0)
        assert c.tick(now, 0.05)[0] == 0.0
        assert c.target_speed == pytest.approx(speed - 0.1)
        assert c.integral == 0.0
    ready(c, now=10.15, speed=1.8, acceleration=1.0)
    throttle, _ = c.tick(10.15, 0.05)
    assert c.target_speed == pytest.approx(1.85)
    assert 0 < throttle < 0.08
    assert c.diagnostics["braking_requested"] is False


@pytest.mark.parametrize(
    "acceleration,coasting", [(-0.2, True), (-0.1, True), (-0.05, False), (0, False)]
)
def test_deceleration_threshold_allows_small_pid_corrections(acceleration, coasting):
    c = Actuation(Limits(max_speed=5.0, max_throttle=0.2, braking_acceleration_threshold=0.1))
    c.target_speed = 3.0
    ready(c, speed=3.0, acceleration=acceleration)
    throttle, _ = c.tick(10, 0.05)
    assert (throttle == 0) is coasting
    assert c.diagnostics["braking_requested"] is coasting


def test_diagnostics_show_clipping_and_clear_after_watchdog():
    c = Actuation(Limits(max_speed=5.0, max_throttle=0.02))
    c.target_speed = 4.0
    ready(c, speed=1.0, steering=math.pi, acceleration=2.0)
    assert c.tick(10, 0.05) == (0.02, 1.0)
    assert c.diagnostics["throttle_saturated"] is True
    assert c.diagnostics["actuator_steering_saturated"] is True
    assert c.diagnostics["actuator_command_age_s"] == 0
    assert c.diagnostics["actuator_odom_age_s"] == 0
    assert c.diagnostics["actuator_lidar_age_s"] == 0
    assert c.tick(11, 0.05) == (0, 0)
    assert c.diagnostics["actuator_target_speed_mps"] == 0
    assert c.diagnostics["throttle_saturated"] is False
    assert c.diagnostics["actuator_steering_saturated"] is False
    assert c.diagnostics["actuator_command_age_s"] == 1


def test_watchdog_and_reset_override_braking_state():
    c = Actuation(Limits(max_speed=5.0, max_throttle=0.2))
    ready(c, speed=4.0, steering=math.radians(15), acceleration=-2.0)
    assert c.tick(10, 0.05)[1] == pytest.approx(0.5)
    assert c.tick(11, 0.05) == (0, 0)
    assert c.braking_requested is False
    assert c.diagnostics["braking_requested"] is False
    c.reset()
    assert c.diagnostics["actuator_command_age_s"] is None
    json.dumps(c.diagnostics, allow_nan=False)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_invalid_commands_and_feedback_stop(bad):
    c = Actuation()
    ready(c, steering=bad)
    assert c.tick(10, 0.05) == (0, 0)
    ready(c)
    c.receive_speed(bad, 10)
    assert c.tick(10, 0.05) == (0, 0)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_times_stop_and_diagnostics_are_json_safe(bad):
    c = Actuation()
    ready(c)
    c.receive_command(0, 1, bad)
    assert c.tick(10, 0.05) == (0, 0)
    ready(c)
    c.receive_speed(0, bad)
    assert c.tick(10, 0.05) == (0, 0)
    ready(c)
    c.scan_time = bad
    assert c.tick(10, 0.05) == (0, 0)
    assert c.diagnostics["actuator_lidar_age_s"] is None
    json.dumps(c.diagnostics, allow_nan=False)
    ready(c)
    assert c.tick(bad, 0.05) == (0, 0)
    json.dumps(c.diagnostics, allow_nan=False)


def test_reset_and_delayed_tick_discard_integrators():
    c = Actuation()
    ready(c)
    c.tick(10, 0.05)
    assert c.tick(10, 1.0) == (0, 0)
    assert c.target_speed == c.integral == 0
    c.reset()
    assert c.tick(10, 0.05) == (0, 0)


@pytest.mark.parametrize(
    "kwargs", [
        {"max_speed": 0}, {"max_steer": float("nan")}, {"max_throttle": 2}, {"timeout": -1},
        {"braking_acceleration_threshold": 0}, {"braking_acceleration_threshold": -0.1},
        {"braking_acceleration_threshold": float("nan")},
        {"braking_acceleration_threshold": float("inf")},
    ]
)
def test_configuration_rejects_invalid_limits(kwargs):
    with pytest.raises(ValueError):
        Limits(**kwargs)
