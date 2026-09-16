import json
import math
from types import SimpleNamespace as NS

import pytest

from avlite_autodrive.track_map import TrackMap


def header(stamp_s, frame):
    total_ns = round(stamp_s * 1e9)
    sec, nanosec = divmod(total_ns, 1_000_000_000)
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame)


def odom(stamp=1.0, x=0.0, y=0.0, yaw=0.0):
    return NS(
        header=header(stamp, "world"), child_frame_id="roboracer_1",
        pose=NS(pose=NS(position=NS(x=x, y=y), orientation=NS(
            x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))),
        twist=NS(twist=NS(linear=NS(x=0.0, y=0.0))),
    )


def scan(stamp=1.0, ranges=(1.0,), angle=0.0, increment=math.pi / 2):
    return NS(
        header=header(stamp, "lidar"), ranges=ranges, angle_min=angle,
        angle_increment=increment, range_min=0.1, range_max=10.0,
    )


def pair(track, stamp=1.0, received=1.0, **pose):
    track.add_odometry(odom(stamp, **pose), received)
    track.add_scan(scan(stamp), received)


@pytest.mark.parametrize("scan_first", [False, True])
def test_real_hits_transform_with_mount_and_rotated_pose_in_either_callback_order(scan_first):
    track = TrackMap()
    messages = [
        (track.add_odometry, odom(x=4.0, y=-3.0, yaw=math.pi / 2)),
        (track.add_scan, scan(ranges=[2.0, 1.0])),
    ]
    for add, msg in reversed(messages) if scan_first else messages:
        add(msg, 5.0)
    result = track.snapshot()
    assert result["points"][0] == pytest.approx([4.0, -0.7267])
    assert result["points"][1] == pytest.approx([3.0, -2.7267])
    assert result["scans_received"] == result["scans_used"] == 1
    assert result["source"] == "lidar_hits_with_simulator_ground_truth"
    assert result["frame_id"] == "world"
    json.dumps(result, allow_nan=False)


def test_invalid_and_no_return_ranges_do_not_draw_fabricated_boundaries():
    track = TrackMap()
    track.add_odometry(odom(), 1.0)
    track.add_scan(scan(ranges=[math.inf, math.nan, -2.0, 0.09, 10.0, 10.1, 0.1, 3]), 1.0)
    result = track.snapshot()
    assert result["hit_count"] == result["point_count"] == 2
    assert result["points"][0] == pytest.approx([0.1733, 0.0])
    assert result["points"][1] == pytest.approx([0.2733, -3.0])


@pytest.mark.parametrize("kind,frame", [("odom", "map"), ("scan", "base_link")])
def test_mismatched_frames_are_rejected(kind, frame):
    track = TrackMap()
    message = odom() if kind == "odom" else scan()
    message.header.frame_id = frame
    getattr(track, "add_odometry" if kind == "odom" else "add_scan")(message, 1.0)
    assert track.snapshot()["rejections"][kind + "_frame_mismatch"] == 1
    assert track.snapshot()["points"] == []


def test_wrong_odometry_child_frame_is_rejected():
    track = TrackMap()
    message = odom()
    message.child_frame_id = "lidar"
    track.add_odometry(message, 1.0)
    assert track.snapshot()["rejections"]["odom_frame_mismatch"] == 1


@pytest.mark.parametrize("kind", ["odom", "scan"])
@pytest.mark.parametrize("missing", [False, True])
def test_zero_and_missing_source_stamps_are_not_implicitly_fresh(kind, missing):
    track = TrackMap()
    message = odom(stamp=0.0) if kind == "odom" else scan(stamp=0.0)
    if missing:
        del message.header.stamp
    getattr(track, "add_odometry" if kind == "odom" else "add_scan")(message, 1.0)
    assert track.snapshot()["rejections"][kind + "_invalid_stamp"] == 1


def test_source_skew_limit_and_future_odometry_pairing():
    track = TrackMap()
    track.add_odometry(odom(stamp=1.0), 1.0)
    track.add_scan(scan(stamp=1.01), 1.01)
    assert track.snapshot()["scans_used"] == 0
    track.add_odometry(odom(stamp=1.02), 1.02)
    assert track.snapshot()["scans_used"] == 1
    assert track.snapshot()["synchronization"]["max_used_source_skew_s"] == 0.01
    track.add_scan(scan(stamp=1.20), 1.20)
    track.add_odometry(odom(stamp=1.23), 1.23)
    assert track.snapshot()["scans_used"] == 1
    assert track.snapshot()["rejections"]["scan_unsynchronized"] == 1


@pytest.mark.parametrize("scan_first", [False, True])
def test_old_received_data_is_not_paired_even_with_equal_source_stamps(scan_first):
    track = TrackMap()
    if scan_first:
        track.add_scan(scan(), 1.0)
        track.add_odometry(odom(), 1.101)
    else:
        track.add_odometry(odom(), 1.0)
        track.add_scan(scan(), 1.101)
        track.add_odometry(odom(stamp=1.2), 1.202)
    assert track.snapshot()["points"] == []
    assert track.snapshot()["rejections"]["scan_unsynchronized"] == 1


def test_explicit_reset_retains_hits_and_rejects_pre_reset_pairing():
    track = TrackMap()
    pair(track)
    track.add_scan(scan(stamp=1.2), 1.2)
    track.reset()
    track.add_odometry(odom(stamp=1.2, x=10.0), 1.21)
    assert track.snapshot()["scans_used"] == 1
    pair(track, stamp=1.4, received=1.4, x=10.0)
    result = track.snapshot()
    assert result["points"][0] == pytest.approx([1.2733, 0.0])
    assert result["points"][1] == pytest.approx([11.2733, 0.0])
    assert result["reset_count"] == 1
    assert result["rejections"]["scan_reset_discarded"] == 1
    assert result["rejections"]["odom_nonmonotonic_stamp"] == 1


def test_pose_jump_discards_scan_that_arrives_before_jump_odometry():
    track = TrackMap()
    pair(track)
    track.add_odometry(odom(stamp=1.2), 1.2)
    track.add_scan(scan(stamp=1.21), 1.21)
    assert track.snapshot()["scans_pending"] == 1
    track.add_odometry(odom(stamp=1.21, x=10), 1.22)
    result = track.snapshot()
    assert result["scans_used"] == 1
    assert result["reset_count"] == 1
    assert result["scans_pending"] == 0
    track.add_scan(scan(stamp=1.22), 1.23)
    track.add_odometry(odom(stamp=1.23, x=10), 1.24)
    assert track.snapshot()["points"][-1] == pytest.approx([11.2733, 0.0])


@pytest.mark.parametrize("kind", ["odom", "scan"])
def test_backwards_or_duplicate_source_stamps_are_rejected(kind):
    track = TrackMap()
    pair(track)
    add = track.add_odometry if kind == "odom" else track.add_scan
    make = odom if kind == "odom" else scan
    add(make(stamp=0.9), 1.01)
    add(make(stamp=1.0), 1.02)
    assert track.snapshot()["rejections"][kind + "_nonmonotonic_stamp"] == 2
    assert track.snapshot()["scans_used"] == 1


def test_rate_limit_deduplication_and_maximum_map_size():
    track = TrackMap(max_cells=2)
    pair(track)
    pair(track, stamp=1.02, received=1.02)
    pair(track, stamp=1.1, received=1.1, x=0.001)
    assert track.snapshot()["point_count"] == 1
    pair(track, stamp=1.2, received=1.2, x=0.1)
    pair(track, stamp=1.3, received=1.3, x=0.2)
    result = track.snapshot()
    assert result["scans_used"] == 4
    assert result["hit_count"] == 4
    assert result["point_count"] == 2
    assert result["truncated"]
    assert result["rejections"]["scan_rate_limited"] == 1


def test_pending_scan_buffer_is_bounded_and_snapshot_is_independent():
    track = TrackMap(buffer_size=2)
    for i in range(3):
        track.add_scan(scan(stamp=1 + i / 100), 1 + i / 100)
    assert track.snapshot()["scans_pending"] == 2
    assert track.snapshot()["rejections"]["scan_buffer_full"] == 1
    track.add_odometry(odom(stamp=1.02), 1.02)
    result = track.snapshot()
    result["points"][0][0] = 900
    assert track.snapshot()["points"][0][0] == pytest.approx(1.2733)


def test_empty_no_return_scan_is_finite_and_missing_geometry_is_rejected():
    track = TrackMap()
    track.add_odometry(odom(), 1.0)
    track.add_scan(scan(ranges=[math.inf, math.nan]), 1.0)
    result = track.snapshot()
    assert result["points"] == []
    assert result["scans_used"] == 1
    json.dumps(result, allow_nan=False)
    message = scan(stamp=1.2)
    message.angle_increment = math.nan
    track.add_scan(message, 1.2)
    assert track.snapshot()["rejections"]["scan_invalid_geometry"] == 1
