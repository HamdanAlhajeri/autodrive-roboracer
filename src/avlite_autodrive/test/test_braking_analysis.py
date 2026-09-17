import pytest

from avlite_autodrive.braking_analysis import analyze_response


def episodes():
    rows = []
    for run in range(3):
        for i in range(10):
            rows.append(dict(elapsed_s=run * 2 + i / 10, speed=2.5 - i / 10,
                             throttle_command=0, throttle=0, steering=0,
                             odom_age_s=0.01, throttle_command_age_s=0.01,
                             throttle_age_s=0.01, steering_age_s=0.01,
                             resets=0, collision_count=0))
    return rows


def test_measured_coasting_has_explicit_conservative_margin():
    result = analyze_response(episodes())
    assert result["qualified"]
    assert len(result["coast_episodes"]) == 3
    assert result["suggested_braking_deceleration_mps2"] == pytest.approx(0.8)


@pytest.mark.parametrize("key,value", [("steering", 0.2), ("odom_age_s", 0.3),
                                       ("throttle", 0.1), ("throttle_command", 0.1)])
def test_turning_powered_or_stale_intervals_do_not_calibrate_braking(key, value):
    rows = episodes()
    for row in rows:
        row[key] = value
    assert not analyze_response(rows)["qualified"]


def test_not_enough_independent_coasts_remains_uncalibrated():
    result = analyze_response(episodes()[:10])
    assert not result["qualified"]
    assert result["suggested_braking_deceleration_mps2"] is None
