"""Timing tools used to find why the supervisor falls behind its sensors."""

import threading

import pytest

from avlite_roboracer.profiling import StackSampler, TickStats


def test_tick_stats_report_the_busiest_phase_and_the_longest_gap():
    stats = TickStats()
    assert stats.text() == "no control ticks recorded"
    # Tick 1 at t=0: scans 10 ms, control 40 ms. Tick 2 starts 0.3 s later.
    stats.record(0.0, {"scans": 0.010, "supervisor": 0.011, "faults": 0.012,
                       "actions": 0.013, "control": 0.053, "publish": 0.055})
    stats.record(0.3, {"scans": 0.31, "supervisor": 0.311, "faults": 0.312,
                       "actions": 0.313, "control": 0.343, "publish": 0.345})
    assert stats.ticks == 2
    assert stats.gap_max_s == pytest.approx(0.3)
    assert stats.phase_total_s["control"] == pytest.approx(0.07)
    assert stats.tick_max_s == pytest.approx(0.055)
    text = stats.text()
    assert "2 ticks" in text and "longest gap between ticks 300 ms" in text
    assert "most time in 'control'" in text


def test_missing_phases_are_skipped_and_reset_clears_everything():
    stats = TickStats()
    stats.record(1.0, {"scans": 1.1})
    assert stats.phase_total_s["scans"] == pytest.approx(0.1)
    assert stats.tick_max_s == pytest.approx(0.1)
    stats.reset()
    assert stats.ticks == 0 and stats.last_start is None and stats.tick_max_s == 0.0


def busy_function(stop):
    while not stop.is_set():
        sum(range(1000))


def test_stack_sampler_finds_the_function_a_thread_is_busy_in():
    stop = threading.Event()
    worker = threading.Thread(target=busy_function, args=(stop,))
    worker.start()
    try:
        sampler = StackSampler(worker.ident, interval_s=0.005)
        assert sampler.report() == "no samples"
        for _ in range(50):
            sampler.sample()
    finally:
        stop.set()
        worker.join()
    assert sampler.samples == 50
    report = sampler.report()
    assert "busy_function" in report and "samples of the supervisor thread" in report
    sampler.reset()
    assert sampler.samples == 0


def test_stack_sampler_only_counts_while_active():
    sampler = StackSampler(threading.get_ident(), interval_s=0.002)
    sampler.start()
    try:
        threading.Event().wait(0.05)
        assert sampler.samples == 0  # switched off by default
        sampler.active = True
        threading.Event().wait(0.1)
    finally:
        sampler.active = False
        sampler.stop()
    assert sampler.samples > 0
