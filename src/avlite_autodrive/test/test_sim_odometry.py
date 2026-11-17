import math

import pytest

from avlite_autodrive.sim_odometry import WheelOdometry, angle_delta

R = 0.05


def drive(odometry, speed, yaw_rate, seconds, rate_hz=50, start=(0.0, 0.0, 0.0)):
    """Feed cumulative wheel angles for a constant speed and turn rate."""
    left = right = 0.0
    t = 0.0
    odometry.update(left, right, yaw_rate, t)
    for _ in range(int(seconds * rate_hz)):
        t += 1 / rate_hz
        left += speed / R / rate_hz
        right += speed / R / rate_hz
        odometry.update(left, right, yaw_rate, t)


def test_straight_line_distance_and_speed():
    odometry = WheelOdometry(R)
    drive(odometry, 2.0, 0.0, 3.0)
    assert odometry.x == pytest.approx(6.0, rel=1e-6)
    assert odometry.y == pytest.approx(0.0, abs=1e-9)
    assert odometry.speed == pytest.approx(2.0)


def test_constant_turn_follows_the_circle():
    odometry = WheelOdometry(R)
    speed, yaw_rate = 1.0, math.pi / 4  # half a circle in exactly 4 s
    drive(odometry, speed, yaw_rate, 4.0)
    assert odometry.x == pytest.approx(0.0, abs=0.01)
    assert odometry.y == pytest.approx(2 * speed / yaw_rate, abs=0.01)
    assert abs(odometry.yaw) == pytest.approx(math.pi, abs=1e-6)


def test_cumulative_encoders_count_large_steps():
    """At racing speed a wheel turns close to pi between samples; nothing may be lost."""
    odometry = WheelOdometry(R)
    drive(odometry, 8.0, 0.0, 1.0, rate_hz=30)  # 5.3 rad per sample
    assert odometry.x == pytest.approx(8.0, rel=1e-6)


def test_wrapped_encoders_unwrap_small_steps():
    odometry = WheelOdometry(R, wrapped=True)
    odometry.update(3.1, 3.1, 0.0, 0.0)
    odometry.update(-3.1, -3.1, 0.0, 0.1)  # crossed +pi: forwards by 2*pi - 6.2 rad
    assert odometry.x == pytest.approx((2 * math.pi - 6.2) * R)
    assert angle_delta(0.1, 2 * math.pi - 0.1) == pytest.approx(0.2)


def test_bad_samples_reset_without_moving():
    odometry = WheelOdometry(R)
    odometry.update(0.0, 0.0, 0.0, 1.0)
    assert not odometry.update(10.0, 10.0, 0.0, 1.0)      # repeated stamp
    assert not odometry.update(20.0, 20.0, math.nan, 1.1)  # nonfinite gyro
    assert not odometry.update(30.0, 30.0, 0.0, 1.2)       # first sample after the reset
    assert odometry.update(31.0, 31.0, 0.0, 1.3)
    assert odometry.x == pytest.approx(R)
    with pytest.raises(ValueError):
        WheelOdometry(0.0)


def test_injected_errors_drift_like_real_sensors():
    odometry = WheelOdometry(R, speed_scale=1.02, yaw_rate_bias=0.01)
    drive(odometry, 1.0, 0.0, 10.0)
    assert math.hypot(odometry.x, odometry.y) == pytest.approx(10.2, rel=1e-3)
    assert odometry.yaw == pytest.approx(0.1, abs=1e-6)
