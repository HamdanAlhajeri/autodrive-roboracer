import copy
import json
from types import SimpleNamespace

import numpy as np
import pytest

from avlite_autodrive.response_test import CoastExperiment, measurement_config
from avlite_autodrive.response_analysis import analyze
from avlite_autodrive.braking_analysis import analyze_response


def experiment(trials=3):
    path = SimpleNamespace(length=20.0, s=np.linspace(0, 20, 201))
    return CoastExperiment(path, np.zeros(200), 1.5, trials, 0.324, np.pi / 6)


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
    with pytest.raises(ValueError, match="No sufficiently long straight"):
        CoastExperiment(SimpleNamespace(length=20, s=np.linspace(0, 20, 201)),
                        np.ones(200), 1.5, 3, 0.324, np.pi / 6)


def test_coast_is_bounded_and_cannot_repeat_without_a_circuit():
    test = experiment()
    assert test.step(0, 0, 1.5, 0, True, True, 1.5)
    for i in range(1, 17):
        test.step(i * 0.05, i * 0.06, 1.4, 0, True, True, 1.5)
    assert test.phase == 0
    assert not test.step(0.85, 1.02, 1.5, 0, True, True, 1.5)
    assert test.trial == 1


@pytest.mark.parametrize("change", [{"fresh": False}, {"valid": False},
                                   {"steer": 0.1}, {"target": 0.8}, {"speed": 0.7}])
def test_never_starts_coast_in_invalid_turning_slow_or_limited_state(change):
    args = dict(now=1, s=0, speed=1.5, steer=0, valid=True, fresh=True, target=1.5)
    args.update(change)
    assert not experiment().step(**args)


def test_stale_tick_and_reset_jump_end_current_attempt():
    for now, s in ((1.0, 0.1), (0.05, 5)):
        test = experiment()
        assert test.step(0, 0, 1.5, 0, True, True, 1.5)
        assert not test.step(now, s, 1.5, 0, True, True, 1.5)


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


def test_report_qualifies_three_trials_and_explains_speed_limit_without_settings_changes():
    plan = {"settings": {"max_velocity_mps": 20, "braking_calibrated": False,
                         "commissioning_speed_mps": 2.5}, "response_test": {"requested_trials": 3}}
    result = analyze(trial_rows(), plan, {"max_throttle": 0.2, "feedforward": 0.04}, {"clean_run": True})
    assert result["qualified"]
    assert result["suggested_braking_deceleration_mps2"] == pytest.approx(0.8)
    assert len(result["attempts"]) == 3
    assert result["speed_limits"]["effective_planned_ceiling_mps"] == 2.5
    assert result["speed_limits"]["feedforward_only_cap_speed_mps"] == 5.0
    assert result["speed_limits"]["upper_throttle_saturation_samples"] == 0
    assert result["coast_episodes"][0]["distance_m"] > 0
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("summary", [{}, {"clean_run": False}, {"clean_run": True, "incident_detected": True}])
def test_failed_recording_cannot_calibrate(summary):
    result = analyze(trial_rows(), summary=summary)
    assert not result["qualified"]
    assert result["suggested_braking_deceleration_mps2"] is None


def test_request_for_more_trials_requires_all_of_them():
    result = analyze(trial_rows(), {"response_test": {"requested_trials": 4}})
    assert not result["qualified"]
