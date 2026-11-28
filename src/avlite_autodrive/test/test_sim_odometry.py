import math

import pytest

from avlite_autodrive.sim_odometry import ClockRate, WheelOdometry, angle_delta

R = 0.05


def drive(odometry, speed, yaw_rate, seconds, rate_hz=50, start=(0.0, 0.0, 0.0), accel=None):
    """Feed cumulative wheel angles for a constant speed and turn rate (accel: the IMU
    reading, if any)."""
    left = right = 0.0
    t = 0.0
    odometry.update(left, right, yaw_rate, t, accel)
    for _ in range(int(seconds * rate_hz)):
        t += 1 / rate_hz
        left += speed / R / rate_hz
        right += speed / R / rate_hz
        odometry.update(left, right, yaw_rate, t, accel)


def test_straight_line_distance_and_speed():
    odometry = WheelOdometry(R)
    drive(odometry, 2.0, 0.0, 3.0)
    assert odometry.x == pytest.approx(6.0, rel=1e-6)
    assert odometry.y == pytest.approx(0.0, abs=1e-9)
    assert odometry.speed == pytest.approx(2.0)


def brake_with_imu(odometry, v0, decel, seconds, wheel_factor, rate_hz=100):
    """Straight braking where the wheels read wheel_factor x the true speed and the IMU is exact."""
    left = right = t = 0.0
    odometry.update(left, right, 0.0, t, 0.0)
    speed, truth = v0, 0.0
    for _ in range(int(seconds * rate_hz)):
        t += 1 / rate_hz
        speed = max(0.0, speed - decel / rate_hz)
        truth += speed / rate_hz
        left += speed * wheel_factor / R / rate_hz
        right += speed * wheel_factor / R / rate_hz
        odometry.update(left, right, 0.0, t, -decel if speed > 0 else 0.0)
    return truth


def test_imu_fusion_recovers_distance_when_wheels_under_read_while_braking():
    truth = None
    results = {}
    for fused in (False, True):
        odometry = WheelOdometry(R, 1.0, imu_fusion=fused)
        # cruise 6 m/s for 2 s (wheels exact), then brake at 3 m/s^2 with the wheels reading 40 %
        # low
        drive(odometry, 6.0, 0.0, 2.0, rate_hz=100, accel=0.0)
        x0 = odometry.x
        left_start = 6.0 / R * 2.0
        t = 2.0
        left = right = left_start
        speed = 6.0
        for _ in range(int(1.5 * 100)):
            t += 0.01
            speed = max(0.0, speed - 3.0 * 0.01)
            left += speed * 0.6 / R * 0.01
            right += speed * 0.6 / R * 0.01
            odometry.update(left, right, 0.0, t, -3.0 if speed > 0 else 0.0)
        results[fused] = odometry.x - x0
        truth = (6.0 + 6.0 - 3.0 * 1.5) / 2 * 1.5 if truth is None else truth
    # the wheels alone are 40 % short
    assert results[False] == pytest.approx(0.6 * truth, rel=0.02)
    assert results[True] == pytest.approx(truth, rel=0.12)          # the IMU brings it close


def test_imu_fusion_without_acceleration_uses_the_wheels():
    plain = WheelOdometry(R, 1.1)
    fused = WheelOdometry(R, 1.1, imu_fusion=True)
    drive(plain, 3.0, 0.0, 2.0)
    drive(fused, 3.0, 0.0, 2.0)   # drive() passes no acceleration
    assert fused.x == pytest.approx(plain.x)


def test_imu_fusion_restarts_from_the_wheels_after_a_gap():
    odometry = WheelOdometry(R, 1.0, imu_fusion=True)
    odometry.update(0.0, 0.0, 0.0, 0.0, 0.0)
    odometry.update(1.0, 1.0, 0.0, 0.01, 0.0)
    odometry.update(5.0, 5.0, 0.0, 5.0, 0.0)   # five seconds later: a respawn, not a measurement
    assert odometry.fused_accel == 0.0
    assert odometry.fused_speed == pytest.approx(4.0 * R / 4.99, rel=1e-6)


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


def drive_slow_clock(odometry, speed, yaw_rate, seconds, rate, rate_hz=50):
    """Encoders and gyro advance per simulated second, but stamps count wall seconds.

    In wall time the simulator covers only ``rate`` simulated seconds per second.
    """
    left = right = 0.0
    t = 0.0
    odometry.update(left, right, yaw_rate, t)
    for _ in range(int(seconds * rate_hz)):
        t += 1 / rate_hz                     # wall-clock stamp
        left += speed / R / rate_hz * rate   # but only rate*dt of simulated motion happened
        right += speed / R / rate_hz * rate
        odometry.update(left, right, yaw_rate, t)


def test_speed_reads_low_on_a_slow_simulator_unless_the_clock_rate_is_applied():
    uncorrected = WheelOdometry(R)
    drive_slow_clock(uncorrected, 4.0, 0.0, 2.0, rate=0.5)
    # half the true 4 m/s: what the controller used to see
    assert uncorrected.speed == pytest.approx(2.0)
    corrected = WheelOdometry(R)
    corrected.time_scale = 0.5
    drive_slow_clock(corrected, 4.0, 0.0, 2.0, rate=0.5)
    assert corrected.speed == pytest.approx(4.0)
    assert corrected.x == pytest.approx(uncorrected.x)  # distance never depended on the clock


def test_gyro_integration_uses_simulated_time():
    odometry = WheelOdometry(R)
    odometry.time_scale = 0.5
    # a gyro rate of 1 rad/s per simulated second for 2 wall seconds = 1 simulated second = 1 rad
    drive_slow_clock(odometry, 1.0, 1.0, 2.0, rate=0.5)
    assert odometry.yaw == pytest.approx(1.0, abs=1e-6)


def feed(clock, rate, speed, seconds, hz=30, start=(0.0, 0.0), t0=0.0):
    """Truth samples from a car moving at ``speed`` simulated m/s, clock running at ``rate``."""
    x, y = start
    t = t0
    for _ in range(int(seconds * hz)):
        t += 1 / hz
        x += speed * rate / hz
        clock.update(t, x, y, speed)
    return t, x


def test_clock_rate_is_measured_from_displacement_and_reported_speed():
    clock = ClockRate()
    feed(clock, 0.46, 6.0, 6.0)
    assert clock.known
    assert clock.rate == pytest.approx(0.46, abs=0.01)
    real_time = ClockRate()
    feed(real_time, 1.0, 3.0, 4.0)
    assert real_time.rate == pytest.approx(1.0, abs=0.01)


def test_clock_rate_follows_a_change_and_ignores_slow_driving_and_teleports():
    clock = ClockRate()
    t, x = feed(clock, 0.5, 5.0, 4.0)
    assert clock.rate == pytest.approx(0.5, abs=0.01)
    before = clock.rate
    t, x = feed(clock, 0.5, 0.3, 3.0, start=(x, 0.0), t0=t)   # standing still tells us nothing
    assert clock.rate == pytest.approx(before, abs=0.005)
    # a respawn teleport is not a clock reading
    clock.update(t + 0.03, x + 40.0, 0.0, 5.0)
    t, x = feed(clock, 0.5, 5.0, 1.0, start=(x + 40.0, 0.0), t0=t + 0.03)
    assert clock.rate == pytest.approx(0.5, abs=0.02)
    t, x = feed(clock, 0.8, 5.0, 8.0, start=(x, 0.0), t0=t)    # the simulator gets faster
    assert clock.rate == pytest.approx(0.8, abs=0.03)


def test_clock_rate_survives_bad_samples():
    clock = ClockRate()
    clock.update(0.0, 0.0, 0.0, 5.0)
    clock.update(0.1, math.nan, 0.0, 5.0)
    assert clock.rate == 1.0 and not clock.known
