"""ICP replay of a recorded drive against a saved map, on a synthetic ring track."""

import math

import numpy as np
import pytest

from avlite_roboracer.icp_replay import ReplayError, load_drive, main, replay
from avlite_roboracer.occupancy import FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from avlite_roboracer.records import write_records

pytest.importorskip("avlite")

RES = 0.05
MOUNT = (0.2, 0.0, 0.0)
YAW_SCALE, DIST_SCALE = 0.7, 0.92   # the fake odometry turns and travels too little


def ring_grid():
    """A 10 x 7 m room with an island, and a stub on the outer wall so it is not symmetric."""
    cells = np.full((int(8 / RES), int(11 / RES)), UNKNOWN, dtype=np.int8)

    def box(x0, y0, x1, y1, value):
        cells[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = value

    box(0, 0, 10, 7, FREE)
    for x0, y0, x1, y1 in ((0, 0, 10, 0.1), (0, 6.9, 10, 7), (0, 0, 0.1, 7), (9.9, 0, 10, 7),
                           (3, 2, 6, 5), (7, 0, 7.1, 0.8)):
        box(x0, y0, x1, y1, OCCUPIED)
    return OccupancyGrid(cells, RES, (0.0, 0.0))


def scan_at(grid, pose, beams=180, max_range=8.0):
    x, y, yaw = pose
    sx, sy = x + MOUNT[0] * math.cos(yaw), y + MOUNT[1] * math.sin(yaw)
    angles = np.linspace(-math.pi, math.pi, beams, endpoint=False)
    ranges = []
    for a in angles:
        d, outcome = grid.cast((sx, sy), (math.cos(yaw + a), math.sin(yaw + a)), max_range)
        ranges.append(round(d, 3) if outcome == "occupied" else None)
    return {"kind": "scan", "topic": "/scan", "frame": "laser", "angle_min": -math.pi,
            "angle_increment": 2 * math.pi / beams, "range_min": 0.05, "range_max": max_range,
            "ranges": ranges}


def drive(grid, step=0.15):
    """Records of one lap of the ring's centre line, with odometry that under-reads."""
    corners = [(1.5, 1.5), (8.5, 1.5), (8.5, 5.5), (1.5, 5.5), (1.5, 1.5)]
    poses = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
        n = int(math.hypot(x1 - x0, y1 - y0) / step)
        yaw = math.atan2(y1 - y0, x1 - x0)
        poses += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, yaw) for i in range(n)]
    records, ox, oy, oyaw, t = [], 0.0, 0.0, poses[0][2] * YAW_SCALE, 0
    for i, pose in enumerate(poses):
        if i:
            prev = poses[i - 1]
            dist = math.hypot(pose[0] - prev[0], pose[1] - prev[1]) * DIST_SCALE
            turn = math.atan2(math.sin(pose[2] - prev[2]), math.cos(pose[2] - prev[2]))
            oyaw += turn * YAW_SCALE
            ox, oy = ox + dist * math.cos(oyaw), oy + dist * math.sin(oyaw)
        t += 100_000_000
        records.append({"kind": "odom", "topic": "/odom", "rx_ns": t - 1, "stamp_ns": t - 1,
                        "frame": "odom", "child": "base_link", "x": ox, "y": oy, "yaw": oyaw,
                        "vx": 0.5, "vy": 0.0, "wz": 0.0, "cov": [None, None, None]})
        scan = scan_at(grid, pose)
        scan.update(rx_ns=t, stamp_ns=t)
        records.append(scan)
    return records, poses


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    grid = ring_grid()
    records, poses = drive(grid)
    path = tmp_path_factory.mktemp("icp") / "records.jsonl"
    write_records(path, records)
    scans, odom = load_drive(path)
    return grid, scans, odom, poses


def test_known_start_tracks_the_car_and_reports_what_the_odometry_gets_wrong(run):
    grid, scans, odom, poses = run
    rows, summary = replay(scans, odom, grid, MOUNT, start=poses[0])
    assert summary["scans_replayed"] == len(scans) and summary["scans_skipped_no_odometry"] == 0
    assert summary["alignment_median"] > 0.9
    assert summary["recoveries"] == 0
    last = rows[-1]
    assert math.hypot(last["icp_x"] - poses[-1][0], last["icp_y"] - poses[-1][1]) < 0.3
    # The wheel odometry in this drive is short in distance and clearly short in turning.
    assert summary["odom_to_icp_distance_ratio"] == pytest.approx(DIST_SCALE, abs=0.06)
    assert summary["odom_to_icp_total_turn_ratio"] == pytest.approx(YAW_SCALE, abs=0.1)
    assert summary["icp_pose_jumps_over_0_5m"] == 0


def test_auto_start_finds_the_start_pose_and_says_how_ambiguous_it_was(run):
    grid, scans, odom, poses = run
    _, summary = replay(scans[:30], odom, grid, MOUNT, start=None)
    x, y, _ = summary["start_pose"]
    assert math.hypot(x - poses[0][0], y - poses[0][1]) < 0.4
    assert summary["start_search_alignment"] > 0.8
    assert summary["start_search_runner_up_alignment"] < summary["start_search_alignment"]


def test_scans_without_recent_odometry_are_skipped_and_an_empty_replay_is_an_error(run):
    grid, scans, odom, poses = run
    late = [dict(o, t=o["t"] + 1000.0) for o in odom]
    with pytest.raises(ReplayError, match="no scan had recent odometry"):
        replay(scans, late, grid, MOUNT, start=poses[0])


def test_records_without_scans_or_odometry_are_rejected(tmp_path):
    path = tmp_path / "records.jsonl"
    write_records(path, [{"kind": "status", "topic": "/s", "rx_ns": 1}])
    with pytest.raises(ReplayError, match="no scan records"):
        load_drive(path)


def test_the_command_needs_exactly_one_way_to_find_the_start(tmp_path):
    with pytest.raises(SystemExit):
        main([str(tmp_path / "r.jsonl"), "--map", "m.yaml", "--profile", "p.yaml"])
    with pytest.raises(SystemExit):
        main([str(tmp_path / "r.jsonl"), "--map", "m.yaml", "--profile", "p.yaml",
              "--start", "0", "0", "0", "--auto-start"])
