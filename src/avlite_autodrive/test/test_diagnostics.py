from avlite_autodrive.diagnostics import CONTROLLER_FIELDS, diagnostic_values


def test_diagnostics_cannot_replace_counters_and_clear_missing_fields():
    result = diagnostic_values(
        '{"target_velocity_mps": 2.5, "lap_count": 99, "steering_saturated": true}',
        CONTROLLER_FIELDS,
    )
    assert result["target_velocity_mps"] == 2.5
    assert result["steering_saturated"] is True
    assert result["lookahead_m"] is None
    assert "lap_count" not in result


def test_preview_diagnostics_preserve_distances_bearings_and_fallback():
    values = diagnostic_values(
        '{"gap_preview_requested_m": 1.5, "gap_preview_selected_m": 1.14, '
        '"gap_preview_fallback": true, "target_bearing_raw_rad": -0.4, '
        '"target_bearing_rad": -0.26, "lookahead_m": 0.6}', CONTROLLER_FIELDS,
    )
    assert values["gap_preview_requested_m"] == 1.5
    assert values["gap_preview_selected_m"] == 1.14
    assert values["lookahead_m"] == 0.6
    assert values["gap_preview_fallback"] is True
    assert values["target_bearing_raw_rad"] == -0.4
    assert values["target_bearing_rad"] == -0.26
    missing = diagnostic_values('{}', CONTROLLER_FIELDS)
    assert missing["gap_preview_requested_m"] is None
    assert missing["gap_preview_fallback"] is None


def test_bad_messages_and_nonfinite_values_are_not_fresh_numeric_measurements():
    assert diagnostic_values('not json', CONTROLLER_FIELDS) is None
    assert diagnostic_values('[]', CONTROLLER_FIELDS) is None
    values = diagnostic_values('{"lookahead_m": NaN, "clearance_m": {}}', CONTROLLER_FIELDS)
    assert values["lookahead_m"] is None
    assert values["clearance_m"] is None
