"""Screening must not confuse launch delays or missed counters with lap times."""

import pytest

from avlite_autodrive.lap_tracking import LapProgress


def test_three_laps_exclude_stationary_wait_and_first_partial_interval():
    progress = LapProgress(3)
    for count, elapsed in [(7, 0), (7, 20), (8, 45), (8, 48), (9, 60)]:
        progress.observe(count, elapsed)
    assert not progress.target_reached
    progress.observe(10, 77)
    assert progress.target_reached
    assert progress.summary() == {
        "requested_laps": 3,
        "completed_laps": 3,
        "counter_discontinuity": False,
        "first_lap_elapsed_s": 45,
        "lap_times_s": [15, 17],
        "lap_time_mean_s": 16,
        "lap_time_std_s": 1,
    }


@pytest.mark.parametrize("jump", [0, 4])
def test_counter_reset_or_skip_never_fabricates_lap_times(jump):
    progress = LapProgress(3)
    progress.observe(1, 0)
    progress.observe(2, 10)
    progress.observe(jump, 20)
    assert progress.completed_laps == 1
    assert progress.counter_discontinuity
    progress.observe(jump + 1, 30)
    assert progress.summary()["lap_times_s"] == []
    progress.observe(jump + 2, 41)
    assert progress.summary()["lap_times_s"] == [11]


def test_single_lap_reports_first_crossing_without_fabricated_rolling_time():
    progress = LapProgress(1)
    progress.observe(0, 0)
    progress.observe(1, 8)
    assert progress.target_reached
    assert progress.summary()["first_lap_elapsed_s"] == 8
    assert progress.summary()["lap_times_s"] == []
    assert progress.summary()["lap_time_mean_s"] is None
    assert progress.summary()["lap_time_std_s"] is None


def test_passive_recording_has_no_lap_stop_target():
    progress = LapProgress()
    progress.observe(1, 0)
    progress.observe(2, 8)
    assert progress.completed_laps == 1
    assert not progress.target_reached


@pytest.mark.parametrize("count", [None, float("inf"), float("nan"), -1, 0.5, True, "1"])
def test_invalid_count_cannot_establish_a_baseline(count):
    progress = LapProgress(1)
    progress.observe(count, 0)
    progress.observe(1, 5)
    assert progress.completed_laps == 0
    assert not progress.target_reached


@pytest.mark.parametrize("target", [0, -1, True, 1.5, "3"])
def test_invalid_targets_are_rejected(target):
    with pytest.raises(ValueError):
        LapProgress(target)


def test_backward_time_breaks_timing_continuity():
    progress = LapProgress(3)
    for count, elapsed in [(0, 0), (1, 10), (2, 9), (3, 20), (4, 32)]:
        progress.observe(count, elapsed)
    assert progress.counter_discontinuity
    assert progress.summary()["lap_times_s"] == [12]
