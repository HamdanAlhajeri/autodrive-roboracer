"""Stall detection: a commanded speed the wheels do not follow."""

from avlite_roboracer.stall import StallMonitor


def run(monitor, command, measured, seconds, start=0.0, dt=0.1):
    reason, t = None, start
    while t <= start + seconds:
        reason = monitor.update(t, command, measured) or reason
        t += dt
    return reason


def test_a_car_that_follows_the_command_never_trips():
    assert run(StallMonitor(), 0.8, 0.7, 60) is None


def test_a_commanded_car_that_barely_moves_trips_after_the_window_with_the_numbers():
    monitor = StallMonitor(window_s=6.0)
    assert run(monitor, 0.8, 0.05, 5.5) is None
    reason = run(monitor, 0.8, 0.05, 1.0, start=5.6)
    assert "commanded 0.80 m/s" in reason and "measured 0.05 m/s" in reason


def test_a_low_command_or_a_moving_moment_restarts_the_clock():
    monitor = StallMonitor(window_s=6.0)
    assert run(monitor, 0.8, 0.05, 5.0) is None
    assert monitor.update(5.1, 0.8, 0.6) is None      # it moved: clock restarts
    assert run(monitor, 0.8, 0.05, 5.0, start=5.2) is None
    assert run(monitor, 0.1, 0.0, 30, start=10.3) is None   # a gentle command is not a stall


def test_reverse_motion_counts_and_zero_window_disables():
    assert run(StallMonitor(window_s=2.0), -0.8, -0.7, 10) is None
    assert run(StallMonitor(window_s=2.0), -0.8, 0.0, 10) is not None
    assert run(StallMonitor(window_s=0), 0.8, 0.0, 100) is None
