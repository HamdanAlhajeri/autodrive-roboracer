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


def test_braking_never_reverses_and_overspeed_cuts_throttle():
    c = Actuation()
    ready(c, acceleration=-2, speed=0.5)
    assert c.tick(10, 0.05)[0] == 0
    ready(c, acceleration=1, speed=2)
    assert c.tick(10, 0.05)[0] == 0


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_invalid_commands_and_feedback_stop(bad):
    c = Actuation()
    ready(c, steering=bad)
    assert c.tick(10, 0.05) == (0, 0)
    ready(c)
    c.receive_speed(bad, 10)
    assert c.tick(10, 0.05) == (0, 0)


def test_reset_and_delayed_tick_discard_integrators():
    c = Actuation()
    ready(c)
    c.tick(10, 0.05)
    assert c.tick(10, 1.0) == (0, 0)
    assert c.target_speed == c.integral == 0
    c.reset()
    assert c.tick(10, 0.05) == (0, 0)


@pytest.mark.parametrize(
    "kwargs", [{"max_speed": 0}, {"max_steer": float("nan")}, {"max_throttle": 2}, {"timeout": -1}]
)
def test_configuration_rejects_invalid_limits(kwargs):
    with pytest.raises(ValueError):
        Limits(**kwargs)
