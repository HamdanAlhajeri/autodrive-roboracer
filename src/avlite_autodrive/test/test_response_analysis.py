import copy
import json
import sys

import pytest

from avlite_autodrive.response_analysis import analyze_response, speed_limit_diagnostics


def capture(interval=0.025):
    rows = []
    for trial in range(1, 4):
        for index in range(9):
            t = trial * 10 + index * interval
            row = {"response_trial_id": trial, "response_phase": 2,
                   "response_data_version": 1, "capture_mode": "odometry",
                   "odom_received_monotonic_s": t, "odom_stamp_ns": int(t * 1e9),
                   "sample_sequence": len(rows) + 1, "elapsed_s": t,
                   "speed": 1.5 - 3 * index * interval,
                   "x": 1.5 * index * interval - 1.5 * (index * interval)**2, "y": 0,
                   "throttle_command": 0, "throttle": 0, "steering": 0,
                   "steering_command": 0}
            row.update({key + "_age_s": 0.01 for key in (
                "odom", "throttle_command", "throttle", "steering", "steering_command",
                "controller_diagnostics")})
            rows.append(row)
    summary = {"status": "completed", "clean_run": True, "stop_reason": "lap_target",
               "actuator_publication_valid": True, "final": {"response_phase": 4},
               "response_capture": {"enabled": True, "samples": len(rows)}}
    return rows, summary


def test_three_source_rate_trials_use_conservative_uncertainty_bound():
    rows, summary = capture()
    result = analyze_response(rows, summary)
    assert result["qualified"]
    assert 2 < result["suggested_braking_deceleration_mps2"] < 2.4
    assert result["accepted_trials"] == 3
    assert result["trials"][0]["fit"]["interval_distance_m"] == pytest.approx(0.24)
    assert result["trials"][0]["fit"]["deceleration_mps2"] == pytest.approx(3)


def test_report_explains_commissioning_and_throttle_limits_without_changing_settings():
    rows, summary = capture()
    plan = {"settings": {"max_velocity_mps": 20, "braking_calibrated": False,
                         "commissioning_speed_mps": 2.5}}
    original = copy.deepcopy(plan)
    result = speed_limit_diagnostics(
        rows, plan, {"max_throttle": 0.2, "feedforward": 0.04}, summary)
    assert result["effective_planned_ceiling_mps"] == 2.5
    assert result["feedforward_only_cap_speed_mps"] == 5.0
    assert result["upper_throttle_saturation_samples"] == 0
    assert plan == original
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("changes", [
    {"status": "interrupted"}, {"stop_reason": "time_limit"}, {"clean_run": False},
    {"actuator_publication_valid": False}, {"incident_detected": True}, {"resets": 1},
    {"response_capture": {"enabled": False}},
])
def test_no_suggestion_for_incomplete_or_unhealthy_run(changes):
    rows, summary = capture()
    result = analyze_response(rows, {**summary, **changes})
    assert not result["qualified"]
    assert result["suggested_braking_deceleration_mps2"] is None
    assert result["run_rejections"]


@pytest.mark.parametrize("field,value", [
    ("throttle_age_s", 0.2), ("controller_diagnostics_age_s", 0.2),
    ("steering", 0.1), ("throttle_command", 0.02), ("throttle", 0.02),
    ("steering_command", 0.1),
])
def test_stale_feedback_powered_or_turning_intervals_are_rejected(field, value):
    rows, summary = capture()
    for row in rows:
        row[field] = value
    result = analyze_response(rows, summary)
    assert not result["qualified"]
    assert result["accepted_trials"] == 0


def test_legacy_10hz_data_and_duplicated_held_samples_cannot_qualify():
    rows, summary = capture(interval=0.1)
    assert not analyze_response(rows, summary)["qualified"]
    rows, summary = capture()
    rows.insert(3, copy.deepcopy(rows[2]))
    result = analyze_response(rows, summary)
    assert not result["qualified"]
    assert any("duplicate" in reason for reason in result["run_rejections"])


def test_fragments_from_one_attempt_do_not_count_as_independent_trials():
    rows, summary = capture()
    for row in rows:
        row["response_trial_id"] = 1
    result = analyze_response(rows, summary)
    assert result["accepted_trials"] == 1
    assert not result["qualified"]
    rows, summary = capture()
    assert not analyze_response(rows, summary, expected_trials=4)["qualified"]


def test_noisy_short_fit_is_rejected():
    rows, summary = capture()
    for i, row in enumerate(rows):
        row["speed"] += 0.09 * (-1 if i % 2 else 1)
    assert analyze_response(rows, summary)["accepted_trials"] == 0


def test_smooth_delivery_with_slow_simulation_does_not_qualify():
    rows, summary = capture()
    for row in rows:
        row["x"] *= 0.39  # observed mismatch in the first live response run
    result = analyze_response(rows, summary)
    assert not result["qualified"]
    assert result["accepted_trials"] == 0
    assert "position travel" in result["trials"][0]["rejections"][0]


def test_powered_acceleration_is_reported_without_qualifying_braking():
    rows, summary = capture()
    for row in rows:
        row.update(response_phase=1, throttle=0.1, throttle_command=0.1,
                   speed=2 - row["speed"])
    result = analyze_response(rows, summary)
    assert not result["qualified"]
    assert len(result["powered_acceleration_intervals"]) == 3
    assert result["powered_acceleration_intervals"][0]["acceleration_mps2"] == pytest.approx(3)


def test_report_cli_writes_graph_and_rejects_missing_configuration(tmp_path, monkeypatch):
    pytest.importorskip("matplotlib")
    from avlite_autodrive.response_analysis import main
    rows, summary = capture()
    (tmp_path / "telemetry.response.jsonl").write_text(
        "\n".join(json.dumps(row) for row in rows) + "\n")
    (tmp_path / "telemetry.summary.json").write_text(json.dumps(summary))
    monkeypatch.setattr(sys, "argv", ["response_analysis", str(tmp_path)])
    main()
    result = json.loads((tmp_path / "response-report.json").read_text())
    assert not result["qualified"]
    assert result["suggested_braking_deceleration_mps2"] is None
    assert result["odometry_receive_timing"]["median_interval_s"] == pytest.approx(0.025)
    assert (tmp_path / "response-report.png").stat().st_size > 1000


def test_truncated_capture_cannot_qualify_even_if_three_fits_survive():
    rows, summary = capture()
    result = analyze_response(rows[:-1], summary)
    assert result["accepted_trials"] == 3
    assert not result["qualified"]
    assert any("sample count" in reason for reason in result["run_rejections"])


def test_partial_json_report_preserves_data_and_cannot_qualify(tmp_path, monkeypatch):
    pytest.importorskip("matplotlib")
    from avlite_autodrive.response_analysis import main
    rows, summary = capture()
    (tmp_path / "telemetry.response.jsonl").write_text(
        "\n".join(json.dumps(row) for row in rows) + '\n{"partial":')
    (tmp_path / "telemetry.summary.json").write_text(json.dumps(summary))
    (tmp_path / "telemetry.planning-config.json").write_text(
        json.dumps({"response_test": {"trials": 3}}))
    monkeypatch.setattr(sys, "argv", ["response_analysis", str(tmp_path)])
    main()
    result = json.loads((tmp_path / "response-report.json").read_text())
    assert not result["qualified"]
    assert any("JSON line" in reason for reason in result["run_rejections"])
    assert result["accepted_trials"] == 3
