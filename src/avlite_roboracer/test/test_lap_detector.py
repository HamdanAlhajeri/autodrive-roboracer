import math

import pytest

from avlite_roboracer.geometry import compose, inverse
from avlite_roboracer.lap_detector import LapDetector, LapSettings, scan_similarity
from roboracer_fixtures import circle_pose

START_SCAN = [1.0, 2.0, None, 3.0] * 20
OTHER_SCAN = [2.5, 0.4, 1.0, None] * 20


def drive(detector, angles, scan_for=lambda a: OTHER_SCAN, odom_scale=1.0, jump=None):
    """Drive CCW around a radius-4 circle; odometry follows the same motion."""
    start = circle_pose(0.0)
    results = []
    for a in angles:
        truth = circle_pose(a)
        odom = compose(inverse(start), truth)
        odom = (odom[0] * odom_scale, odom[1] * odom_scale, odom[2])
        estimate = jump(a, truth) if jump else truth
        results.append(detector.update(estimate, odom, scan_for(a)))
    return results


def started(settings=None):
    detector = LapDetector(settings or LapSettings(min_travel_m=15.0))
    detector.start(circle_pose(0.0), (0.0, 0.0, 0.0), START_SCAN)
    return detector


def lap_angles(laps=1.2, step=0.02):
    return [i * step for i in range(1, int(2 * math.pi * laps / step))]


def near_start(a):
    return START_SCAN if abs((a + math.pi) % (2 * math.pi) - math.pi) < 0.2 else OTHER_SCAN


def test_completes_after_one_lap_with_matching_scan():
    detector = started()
    results = drive(detector, lap_angles(), near_start)
    index = results.index("complete")
    assert lap_angles()[index] == pytest.approx(2 * math.pi, abs=0.03)
    assert detector.travel_m == pytest.approx(2 * math.pi * 4, rel=0.01)


def test_localization_correction_across_the_line_is_not_a_finish():
    # Near the end of the lap, the estimate jumps 0.6 m forward across the finish line
    # while odometry moves only a few centimetres.
    detector = started()

    def jump(a, truth):
        if 2 * math.pi - 0.14 <= a < 2 * math.pi + 0.3:
            return compose(truth, (0.6, 0.0, 0.0))
        return truth

    angles = [a for a in lap_angles(1.0) if a < 2 * math.pi - 0.02]
    results = drive(detector, angles, lambda a: START_SCAN, jump=jump)
    assert "complete" not in results
    assert detector.ignored_corrections == 1
    assert "localization correction" in detector.rejected_crossings


def test_crossing_requires_enough_odometry_travel():
    detector = started(LapSettings(min_travel_m=40.0))
    results = drive(detector, lap_angles(1.1), near_start)
    assert "complete" not in results
    assert any("travelled" in r for r in detector.rejected_crossings)


def test_crossing_requires_scan_confirmation():
    detector = started()
    assert "complete" not in drive(detector, lap_angles(1.1))
    assert any("scan match" in r for r in detector.rejected_crossings)


def test_no_finish_without_departing_the_start_area():
    detector = started(LapSettings(min_travel_m=1.0, departure_radius_m=2.0))
    start = circle_pose(0.0)
    # Shuffle back and forth across the line within 1 m of the start.
    for x in (-0.5, 0.3, -0.5, 0.3):
        pose = compose(start, (x, 0.0, 0.0))
        assert detector.update(pose, compose(inverse(start), pose), START_SCAN) == "mapping"
    assert not detector.departed


def test_heading_must_match_start_direction():
    detector = started(LapSettings(min_travel_m=1.0))
    detector.departed = True
    detector.travel_m = 20.0
    start = circle_pose(0.0)
    before = compose(start, (-0.05, 0.0, math.pi / 2))
    after = compose(start, (0.05, 0.0, math.pi / 2))
    detector._previous = (before, compose(inverse(start), before))
    # Crossing sideways (90 degrees off) is rejected; motion is consistent but heading wrong.
    detector.update(after, compose(inverse(start), after), START_SCAN)
    assert detector.state == "mapping"
    assert "heading does not match the start" in detector.rejected_crossings


def test_excess_travel_fails_the_attempt():
    detector = started(LapSettings(min_travel_m=5.0, max_travel_m=10.0))
    results = drive(detector, lap_angles(0.5))
    assert results[-1] == "failed"


def test_scan_similarity():
    assert scan_similarity(START_SCAN, START_SCAN, 0.1) == 1.0
    assert scan_similarity(START_SCAN, OTHER_SCAN, 0.1) < 0.2
    assert scan_similarity([None, None], [None, None], 0.1) == 0.0
    assert scan_similarity([1.0], [1.0, 2.0], 0.1) == 0.0
