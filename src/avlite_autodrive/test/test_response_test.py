import copy
from types import SimpleNamespace

import numpy as np
import pytest

from avlite_autodrive.response_test import (
    ResponseExperiment, StraightCoastWindow, measurement_config,
)
from avlite_autodrive.braking_analysis import analyze_response


def test_one_bounded_attempt_per_circuit_and_completion():
    experiment = ResponseExperiment(1.5, 3)
    for trial in range(1, 4):
        now = 10 * trial
        assert not experiment.step(now, 5, 30, 1.5, True)
        assert experiment.step(now + 0.21, 5.3, 30, 1.5, True)
        assert experiment.trial == trial
        assert experiment.step(now + 0.4, 5.5, 30, 0.8, True)
        assert not experiment.step(now + 1.1, 6, 30, 0.8, True)
        assert not experiment.step(now + 2, 7, 30, 1.5, True)
        experiment.step(now + 3, 29, 30, 1.5, False)
        experiment.step(now + 4, 0, 30, 1.5, False)
    assert experiment.phase == experiment.FINISHED
    assert not experiment.step(50, 5, 30, 1.5, True)


@pytest.mark.parametrize("speed,eligible,safe", [
    (.6, True, True), (1.5, False, True), (1.5, True, False),
])
def test_coast_ends_on_low_speed_turn_obstacle_or_invalid_pose(speed, eligible, safe):
    experiment = ResponseExperiment(1.5, 3)
    experiment.step(1, 5, 30, 1.5, True)
    assert experiment.step(1.21, 5.3, 30, 1.5, True)
    assert not experiment.step(1.3, 5.4, 30, speed, eligible, safe)
    assert experiment.trial == 1


def test_interruption_aborts_until_new_controller():
    experiment = ResponseExperiment(1.5, 3)
    experiment.reset()  # initial sensor discovery is allowed
    experiment.step(0, 10, 30, 0, False)
    experiment.reset()
    assert not experiment.step(1, 10, 30, 1.5, True)
    assert experiment.diagnostics()["response_aborted"]


@pytest.mark.parametrize("speed,trials", [
    (0, 3), (3, 3), (float("nan"), 3), (1.5, 1), (1.5, 3.2), (1.5, True),
])
def test_reject_invalid_experiment(speed, trials):
    with pytest.raises(ValueError):
        ResponseExperiment(speed, trials)


def test_runtime_profile_preserves_saved_settings_and_limits_speed():
    config = {"driving_mode": "follow_the_gap", "c30_control": {
        "c32_ego_max_velocity": 20, "c35_cruise_velocity": 20},
        "planning": {"braking_calibrated": True, "map_path": "maps/practice.json"}}
    original = copy.deepcopy(config)
    changed = measurement_config(config, 1.5)
    assert config == original
    assert changed["driving_mode"] == "planned"
    assert changed["planning"]["max_velocity_mps"] == 1.5
    assert not changed["planning"]["braking_calibrated"]


@pytest.mark.parametrize("speed", [0.5, 2.6, float("nan"), float("inf")])
def test_unsupported_speed_rejected(speed):
    with pytest.raises(ValueError):
        measurement_config({}, speed)


def test_no_straight_fails_before_driving():
    path = SimpleNamespace(
        s=np.linspace(0, 20, 201),
        at=lambda s: np.column_stack((3 * np.cos(s / 3), 3 * np.sin(s / 3))))
    with pytest.raises(ValueError, match="No sufficiently long straight"):
        StraightCoastWindow(path, 1.5, np.pi / 6)


def trial_rows():
    return [dict(elapsed_s=trial * 3 + i / 10, speed=2.0 - i / 10,
                 throttle_command=0, throttle=0, steering=0,
                 odom_age_s=0.01, throttle_command_age_s=0.01,
                 throttle_age_s=0.01, steering_age_s=0.01,
                 controller_diagnostics_age_s=0.01,
                 target_velocity_mps=2.5, speed_limit_reason=5,
                 resets=0, collision_count=0, response_trial=trial + 1, response_phase=1)
            for trial in range(3) for i in range(8)]


def test_fragmented_attempt_is_not_three_independent_coasts():
    rows = trial_rows()
    for row in rows:
        row["response_trial"] = 1
    result = analyze_response(rows)
    assert not result["qualified"]
    assert len(result["coast_episodes"]) == 1


def test_stale_experiment_phase_cannot_qualify():
    rows = trial_rows()
    for row in rows:
        row["controller_diagnostics_age_s"] = 0.3
    assert not analyze_response(rows)["qualified"]


def test_ordinary_coast_analysis_accepts_both_response_diagnostic_versions():
    rows = trial_rows()
    legacy = analyze_response(rows)
    for row in rows:
        row["response_trial_id"] = row.pop("response_trial")
        row["response_phase"] = 2
    current = analyze_response(rows)
    assert legacy["qualified"] and current["qualified"]
    assert current["coast_episodes"] == legacy["coast_episodes"]
