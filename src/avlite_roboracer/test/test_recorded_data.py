"""Recorded-data checks and the localization-quality report on synthetic recordings."""

import copy
import json
import math
from types import SimpleNamespace as NS

import pytest

from avlite_roboracer.geometry import TransformBuffer, compose
from avlite_roboracer.localization import LocalizationMonitor, LocalizationThresholds
from avlite_roboracer.quality import localization_report, render_markdown
from avlite_roboracer.records import make_record, read_records, write_records
from avlite_roboracer.recording_check import check_recording
from avlite_roboracer.report import main as report_main
from roboracer_fixtures import commissioned_profile, synthetic_recording


@pytest.fixture(scope="module")
def recording():
    return synthetic_recording(odom_drift=0.02)


def statuses(report):
    return {c["name"]: c["status"] for c in report["checks"]}


def test_clean_recording_passes(recording):
    records, *_ = recording
    report = check_recording(records, commissioned_profile(), require_map=True)
    assert report["status"] == "pass", report["checks"]
    assert statuses(report)["laser_mount_matches_profile"] == "pass"


def test_conflicting_transform_publishers_fail(recording):
    records = copy.deepcopy(recording[0])
    # A second node publishes base_link under another parent, and the mount appears on
    # both /tf and /tf_static.
    records.append({"kind": "tf", "topic": "/tf", "rx_ns": records[50]["rx_ns"],
                    "transforms": [{"parent": "map", "child": "base_link", "stamp_ns": None,
                                    "x": 0, "y": 0, "yaw": 0},
                                   {"parent": "base_link", "child": "laser", "stamp_ns": None,
                                    "x": 0.25, "y": 0, "yaw": 0}]})
    report = check_recording(records, commissioned_profile())
    check = next(c for c in report["checks"] if c["name"] == "transform_ownership")
    assert check["status"] == "fail"
    assert "base_link has parents" in check["detail"] and "laser is on both" in check["detail"]


def test_backwards_stamps_and_gaps_fail(recording):
    records = copy.deepcopy(recording[0])
    scans = [r for r in records if r["kind"] == "scan"]
    scans[10]["stamp_ns"] = scans[9]["stamp_ns"] - 1
    for r in scans[20:26]:
        records.remove(r)
    report = check_recording(records, commissioned_profile())
    check = next(c for c in report["checks"] if c["name"] == "scan_timing")
    assert check["status"] == "fail"
    assert "backwards" in check["detail"] and "exceeds sensor timeout" in check["detail"]


def test_clock_mismatch_is_detected(recording):
    records = copy.deepcopy(recording[0])
    for r in records:
        if r["kind"] == "odom":
            r["stamp_ns"] += 2_000_000_000  # driver stamps from another clock
    report = check_recording(records, commissioned_profile())
    assert statuses(report)["odom_timing"] == "fail"


def test_wrong_laser_mount_fails(recording):
    report = check_recording(recording[0], commissioned_profile(**{"mounts.laser.x_m": 0.1}))
    assert statuses(report)["laser_mount_matches_profile"] == "fail"


def test_odometry_jump_and_inverted_imu(recording):
    records = copy.deepcopy(recording[0])
    odom = [r for r in records if r["kind"] == "odom"]
    for r in odom[100:]:
        r["x"] += 1.0
    for r in records:
        if r["kind"] == "imu":
            r["wz"] = -r["wz"]
    report = check_recording(records, commissioned_profile())
    assert statuses(report)["odometry_self_consistency"] == "fail"
    assert statuses(report)["imu_yaw_rate"] == "fail"


def test_missing_map_transform_fails_when_required():
    records, *_ = synthetic_recording(seconds=2.0)
    records = [r for r in records if not (r["kind"] == "tf"
                                          and r["transforms"][0]["parent"] == "map")]
    assert statuses(check_recording(records, commissioned_profile(),
                                    require_map=True))["tf_map_to_odom"] == "fail"


def test_localization_report_passes_on_accurate_estimates(recording):
    records, grid, truth, t0 = recording
    reference = [(t0 + i / 20, truth(i / 20)) for i in range(0, 121)]
    report = localization_report(records, commissioned_profile(), grid, reference)
    assert report["status"] == "pass", report["assessment"]
    assert report["estimate_source"] == "pose_topic"
    assert report["scan_alignment"]["median"] > 0.95
    assert report["reference_position_error_m"]["max"] < 1e-6
    assert report["processing_latency_s"]["median"] == pytest.approx(0.04)
    assert "# Localization-quality report" in render_markdown(None, report)


def test_localization_report_flags_correction_jumps():
    records, grid, *_ = synthetic_recording(jump_at=3.0)
    report = localization_report(records, commissioned_profile(), grid)
    assert report["status"] == "fail"
    failed = {a["metric"] for a in report["assessment"] if a["status"] == "fail"}
    assert "motion_mismatch_m" in failed
    assert "no reference poses" in " ".join(report["limitations"])


def test_unmeasured_thresholds_are_unassessed_not_passed(recording):
    records, grid, *_ = recording
    profile = commissioned_profile(**{"localization.min_scan_alignment": None})
    report = localization_report(records, profile, grid)
    assert report["status"] == "unassessed"


def test_report_cli_end_to_end(tmp_path, recording):
    records, grid, *_ = recording
    write_records(tmp_path / "records.jsonl", records)
    grid.save(tmp_path / "map.yaml")
    profile = tmp_path / "hardware.yaml"
    import yaml
    profile.write_text(yaml.safe_dump(commissioned_profile().data))
    code = report_main([str(tmp_path / "records.jsonl"), "--profile", str(profile),
                        "--map", str(tmp_path / "map.yaml"), "--require-map-frame",
                        "--output", str(tmp_path / "out")])
    assert code == 0
    assert json.loads((tmp_path / "out" / "localization.json").read_text())["status"] == "pass"
    assert (tmp_path / "out" / "report.md").exists()
    assert len(read_records(tmp_path / "records.jsonl")) == len(records)


def header(sec, frame):
    return NS(stamp=NS(sec=sec, nanosec=5), frame_id=frame)


def test_message_converters():
    quaternion = NS(x=0.0, y=0.0, z=math.sin(0.25), w=math.cos(0.25))
    odom = NS(header=header(3, "odom"), child_frame_id="base_link",
              pose=NS(pose=NS(position=NS(x=1.0, y=2.0), orientation=quaternion),
                      covariance=[0.1] + [0.0] * 6 + [0.2] + [0.0] * 27 + [0.3]),
              twist=NS(twist=NS(linear=NS(x=1.5, y=0.0), angular=NS(z=0.4))))
    record = make_record("odom", "/odom", 10, odom)
    assert record["yaw"] == pytest.approx(0.5) and record["cov"] == [0.1, 0.2, 0.3]
    assert record["stamp_ns"] == 3_000_000_005
    scan = NS(header=header(1, "laser"), angle_min=-1.0, angle_increment=0.5,
              range_min=0.1, range_max=10.0, ranges=[1.0, math.inf, math.nan])
    assert make_record("scan", "/scan", 5, scan)["ranges"] == [1.0, None, None]
    sphere, line = 2, 5
    graph = NS(markers=[NS(id=3, type=sphere, pose=NS(position=NS(x=3.0, y=0.0)),
                           header=header(0, "map")),
                        NS(id=1, type=sphere, pose=NS(position=NS(x=1.0, y=0.0)),
                           header=header(0, "map")),
                        NS(id=9, type=line, pose=NS(position=NS(x=0.0, y=0.0)),
                           header=header(0, "map"))])
    assert make_record("graph", "/g", 1, graph)["nodes"] == [[1, 1.0, 0.0], [3, 3.0, 0.0]]
    status = make_record("status", "/s", 1, NS(data="not json"))
    assert status["data"] is None


def test_transform_buffer_chains_and_refuses_stale_lookups():
    buffer = TransformBuffer(max_extrapolation_s=0.1)
    buffer.add("map", "odom", 0.0, (1.0, 0.0, 0.0))
    buffer.add("map", "odom", 1.0, (2.0, 0.0, 0.0))
    buffer.add("odom", "base_link", 0.5, (0.0, 1.0, math.pi / 2))
    buffer.add("base_link", "laser", 0.0, (0.25, 0.0, 0.0), static=True)
    pose = buffer.lookup("map", "laser", 0.5)
    assert pose == pytest.approx(compose((1.5, 0.0, 0.0), (0.0, 1.25, math.pi / 2)))
    assert buffer.lookup("map", "laser", 0.75) is None  # base_link too old
    assert buffer.lookup("laser", "map", 0.5) is None
    with pytest.raises(ValueError):
        buffer.add("world", "odom", 2.0, (0.0, 0.0, 0.0))


def thresholds():
    return LocalizationThresholds.from_profile(commissioned_profile())


def test_localization_monitor_requires_stable_fresh_estimates():
    monitor = LocalizationMonitor(thresholds())
    assert monitor.failures(0.0) == ["no localization estimate"]
    good_cov = [0.001, 0.001, 0.001]
    for i in range(30):
        t = i * 0.1
        monitor.update(t, (t, 0.0, 0.0), (t, 0.0, 0.0), good_cov, 0.9)
        if t < 2.0:
            assert monitor.failures(t)
    assert monitor.failures(t) == []
    assert monitor.failures(3.5)[0].endswith("old")
    monitor.update(3.0, (3.5, 0.0, 0.0), (3.0, 0.0, 0.0), good_cov, 0.9)
    assert "map motion disagrees with odometry" in monitor.failures(3.0)
    monitor.update(3.1, (3.6, 0.0, 0.0), (3.1, 0.0, 0.0), [None, 0, 0], 0.5)
    failures = monitor.failures(3.1)
    assert "pose covariance unavailable" in failures
    assert "scan does not align with the map" in failures


def test_unmeasured_localization_thresholds_block_monitoring():
    profile = commissioned_profile(**{"localization.max_motion_mismatch_m": None})
    with pytest.raises(ValueError, match="max_motion_mismatch_m"):
        LocalizationThresholds.from_profile(profile)


def test_localization_gap_requires_a_new_stability_window():
    monitor = LocalizationMonitor(thresholds())
    for i in range(30):
        monitor.update(i * 0.1, (0, 0, 0), (0, 0, 0), [0.001] * 3, 0.9)
    assert not monitor.failures(29 * 0.1)
    monitor.update(10.0, (0, 0, 0), (0, 0, 0), [0.001] * 3, 0.9)
    assert "stable for only" in monitor.failures(10.0)[0]


@pytest.mark.parametrize("pose,cov,alignment", [
    ((float("nan"), 0, 0), [0.001] * 3, 0.9),
    ((0, 0, 0), [-0.001] * 3, 0.9),
    ((0, 0, 0), [0.001] * 3, float("nan")),
])
def test_invalid_localization_never_becomes_ready(pose, cov, alignment):
    monitor = LocalizationMonitor(thresholds())
    for i in range(30):
        monitor.update(i * 0.1, pose, (0, 0, 0), cov, alignment)
    assert monitor.failures(29 * 0.1)
