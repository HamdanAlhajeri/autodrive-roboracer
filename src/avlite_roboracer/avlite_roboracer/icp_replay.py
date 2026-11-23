"""Replay a recorded drive through the car's ICP localizer against a saved map (no ROS, no motors).

    python -m avlite_roboracer.icp_replay RECORDS.jsonl --map MAP.yaml \
        --profile config/roboracer/hardware.yaml [--start X Y YAW | --auto-start] [--output DIR]

RECORDS.jsonl comes from `roboracer_record export` on a recording. MAP.yaml is a SLAM Toolbox
map_saver map (for example the map of a mapping replay). Each recorded scan is aligned to the
map with the same `IcpPose` the race runtime uses, starting from the previous pose moved by
the recorded wheel odometry, so the result shows how ICP would have behaved on this drive.

The start pose is where the car was in the map frame at the first scan. Give it with
--start, or let --auto-start search the whole map for the pose that fits the first scan best.

It reports how well scans fit the map (alignment: share of scan hits within the tolerance of a
wall), how often the pose was lost and recovered, and how far the odometry disagrees with the
ICP pose in distance and in turning. Nothing here sends anything to the car.
"""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

from .icp_pose import IcpPose, IcpSettings, icp_settings
from .occupancy import FREE, OccupancyGrid
from .profile import HardwareProfile
from .records import read_records

MAX_ODOM_GAP_S = 0.2     # a scan with no odometry this close before it is skipped
COARSE_STEP_M = 0.25     # --auto-start: spacing of candidate positions over the free space
COARSE_YAWS = 24
COARSE_TOLERANCE_M = 0.15
COARSE_KEEP = 8          # best coarse candidates refined with ICP


class ReplayError(ValueError):
    """The records, map or arguments cannot be replayed."""


def _seconds(record):
    stamp = record.get("stamp_ns")
    return (stamp if stamp else record["rx_ns"]) / 1e9


def load_drive(records_path):
    """Return (scans, odom) as time-ordered record lists with a 't' in seconds."""
    records = read_records(records_path)
    scans = [dict(r, t=_seconds(r)) for r in records if r["kind"] == "scan"]
    odom = [dict(r, t=_seconds(r)) for r in records if r["kind"] == "odom"]
    if not scans:
        raise ReplayError(f"{records_path}: no scan records")
    if not odom:
        raise ReplayError(f"{records_path}: no odom records")
    scans.sort(key=lambda r: r["t"])
    odom.sort(key=lambda r: r["t"])
    return scans, odom


def scan_message(record):
    """A LaserScan-shaped object from an exported scan record (null ranges become NaN)."""
    ranges = np.array([math.nan if r is None else r for r in record["ranges"]], dtype=float)
    return SimpleNamespace(ranges=ranges, angle_min=record["angle_min"],
                           angle_increment=record["angle_increment"],
                           range_min=record["range_min"], range_max=record["range_max"])


def sensor_frame(scan, mount_pose):
    """The AVLite sensor frame the race runtime builds from a scan and the laser mount."""
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from avlite_autodrive.plugin.planned_controller import AutoDRIVESensorFrame
    from avlite_autodrive.sensors import scan_cloud, scan_hit_mask
    x, y, yaw = mount_pose
    mount = np.eye(4)
    mount[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    mount[:2, 3] = [x, y]
    return AutoDRIVESensorFrame(
        lidar=scan_cloud(scan), lidar_sensor=Lidar(base_to_sensor=mount), frame_id="base_link",
        lidar_hit_mask=scan_hit_mask(scan), lidar_age_s=0.0, odom_age_s=0.0)


def _base_points(scan, mount_pose, limit=100):
    """Scan hits as x/y points in the base frame, thinned to about ``limit`` points."""
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    good = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max)
    pts = np.c_[ranges[good] * np.cos(angles[good]), ranges[good] * np.sin(angles[good])]
    if len(pts) > limit:
        pts = pts[np.linspace(0, len(pts) - 1, limit).astype(int)]
    x, y, yaw = mount_pose
    c, s = math.cos(yaw), math.sin(yaw)
    return pts @ np.array([[c, -s], [s, c]]).T + [x, y]


def find_start(grid, scan, mount_pose, settings):
    """Search the whole map for the pose that fits one scan best.

    Coarse: score every free cell on a 0.25 m grid at 24 headings by the share of hits near a
    wall. Fine: refine the best few with ICP on the same scan and keep the best alignment.
    Return (pose, alignment, runner_up) where runner_up is the best alignment more than 1 m
    away from the winner (close to the winner's value means the answer is ambiguous).
    """
    from scipy.spatial import cKDTree
    points = _base_points(scan, mount_pose)
    if len(points) < 30:
        raise ReplayError("the first scan has too few returns to find the start pose")
    tree = cKDTree(grid.occupied_points())
    rows, cols = np.nonzero(grid.cells == FREE)
    stride = max(1, int(round(COARSE_STEP_M / grid.resolution)))
    keep = (rows % stride == 0) & (cols % stride == 0)
    centres = np.c_[(cols[keep] + 0.5) * grid.resolution + grid.origin[0],
                    (rows[keep] + 0.5) * grid.resolution + grid.origin[1]]
    if not len(centres):
        raise ReplayError("the map has no free cells")
    scored = []
    for k in range(COARSE_YAWS):
        yaw = 2 * math.pi * k / COARSE_YAWS
        c, s = math.cos(yaw), math.sin(yaw)
        rotated = points @ np.array([[c, -s], [s, c]]).T
        for centre in centres:
            dist, _ = tree.query(rotated + centre)
            scored.append((float(np.mean(dist <= COARSE_TOLERANCE_M)), centre[0], centre[1], yaw))
    scored.sort(reverse=True)
    chosen = []  # the best candidates that are not near an already chosen one
    for candidate in scored:
        if all(math.hypot(candidate[1] - c[1], candidate[2] - c[2]) > 0.5 for c in chosen):
            chosen.append(candidate)
        if len(chosen) == COARSE_KEEP:
            break
    refined = []
    for _, x, y, yaw in chosen:
        icp = IcpPose(grid.occupied_points(), (x, y, yaw), settings)
        frame = sensor_frame(scan, mount_pose)
        for step in range(3):
            icp.update(frame, (0.0, 0.0, 0.0), 0.0, now=float(step))
        if icp.alignment is not None:
            refined.append((icp.alignment, icp.pose))
    if not refined:
        raise ReplayError("could not fit the first scan anywhere on the map")
    refined.sort(key=lambda r: -r[0])
    best_alignment, best_pose = refined[0]
    runner = max((a for a, p in refined[1:]
                  if math.hypot(p[0] - best_pose[0], p[1] - best_pose[1]) > 1.0), default=0.0)
    return best_pose, best_alignment, runner


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def replay(scans, odom, grid, mount_pose, start=None, settings=None):
    """Run ICP over the recorded scans. Return (rows, summary) for one drive."""
    settings = settings or IcpSettings()
    odom_times = [o["t"] for o in odom]
    first = scans[0]
    start_info = {}
    if start is None:
        start, fit, runner = find_start(grid, scan_message(first), mount_pose, settings)
        start_info = {"start_search_alignment": round(fit, 3),
                      "start_search_runner_up_alignment": round(runner, 3)}
    icp = IcpPose(grid.occupied_points(), start, settings)
    rows, skipped = [], 0
    for scan in scans:
        i = bisect.bisect_right(odom_times, scan["t"]) - 1
        if i < 0 or scan["t"] - odom_times[i] > MAX_ODOM_GAP_S:
            skipped += 1
            continue
        o = odom[i]
        icp.update(sensor_frame(scan_message(scan), mount_pose), (o["x"], o["y"], o["yaw"]),
                   o.get("vx", 0.0), now=scan["t"])
        pose = icp.pose
        rows.append({"t": scan["t"], "odom_x": o["x"], "odom_y": o["y"], "odom_yaw": o["yaw"],
                     "icp_x": pose[0], "icp_y": pose[1], "icp_yaw": pose[2],
                     "alignment": icp.alignment, "low_scans": icp.low_scans,
                     "recoveries": icp.recoveries, "recovery_tries": icp.recovery_tries})
    if not rows:
        raise ReplayError("no scan had recent odometry; nothing was replayed")
    summary = summarize(rows, skipped, start, settings)
    summary.update(start_info)
    return rows, summary


def _path(xs, ys):
    return float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))


def _turning(yaws):
    steps = [_wrap(b - a) for a, b in zip(yaws, yaws[1:])]
    return float(sum(steps)), float(sum(abs(s) for s in steps))


def summarize(rows, skipped, start, settings):
    t = np.array([r["t"] for r in rows])
    align = np.array([r["alignment"] if r["alignment"] is not None else math.nan for r in rows])
    ok = align[np.isfinite(align)]
    icp_xy = np.array([[r["icp_x"], r["icp_y"]] for r in rows])
    odom_xy = np.array([[r["odom_x"], r["odom_y"]] for r in rows])
    icp_dist, odom_dist = _path(icp_xy[:, 0], icp_xy[:, 1]), _path(odom_xy[:, 0], odom_xy[:, 1])
    icp_net, icp_abs = _turning([r["icp_yaw"] for r in rows])
    odom_net, odom_abs = _turning([r["odom_yaw"] for r in rows])
    below = float(np.mean(ok < settings.lost_alignment)) if len(ok) else math.nan
    ratio = lambda a, b: round(a / b, 3) if b > 1e-6 else None  # noqa: E731
    return {
        "scans_replayed": len(rows), "scans_skipped_no_odometry": skipped,
        "duration_s": round(float(t[-1] - t[0]), 1),
        "start_pose": [round(float(v), 3) for v in start],
        "alignment_mean": round(float(np.mean(ok)), 3) if len(ok) else None,
        "alignment_median": round(float(np.median(ok)), 3) if len(ok) else None,
        "alignment_p5": round(float(np.percentile(ok, 5)), 3) if len(ok) else None,
        "alignment_min": round(float(np.min(ok)), 3) if len(ok) else None,
        "share_of_scans_below_lost_threshold": round(below, 3),
        "lost_threshold": settings.lost_alignment,
        "recoveries": rows[-1]["recoveries"], "recovery_tries": rows[-1]["recovery_tries"],
        "icp_path_m": round(icp_dist, 2), "odom_path_m": round(odom_dist, 2),
        "odom_to_icp_distance_ratio": ratio(odom_dist, icp_dist),
        "icp_net_turn_deg": round(math.degrees(icp_net), 1),
        "odom_net_turn_deg": round(math.degrees(odom_net), 1),
        "odom_to_icp_net_turn_ratio": ratio(odom_net, icp_net),
        "icp_total_turn_deg": round(math.degrees(icp_abs), 1),
        "odom_total_turn_deg": round(math.degrees(odom_abs), 1),
        "odom_to_icp_total_turn_ratio": ratio(odom_abs, icp_abs),
        "icp_start_to_end_m": round(float(np.linalg.norm(icp_xy[-1] - icp_xy[0])), 2),
        "icp_pose_jumps_over_0_5m": int(np.sum(np.hypot(*np.diff(icp_xy, axis=0).T) > 0.5)),
    }


def write_outputs(directory, rows, summary):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / "trace.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                            encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("records", type=Path, help="records.jsonl from `roboracer_record export`")
    parser.add_argument("--map", type=Path, required=True, help="map_saver map.yaml")
    parser.add_argument("--profile", type=Path, required=True,
                        help="hardware profile (laser mount)")
    parser.add_argument("--race", type=Path, default=None,
                        help="race.yaml whose localization block sets the ICP settings")
    parser.add_argument("--start", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                        help="pose of the car in the map frame at the first scan (metres, radians)")
    parser.add_argument("--auto-start", action="store_true",
                        help="search the whole map for the start pose")
    parser.add_argument("--output", type=Path, help="directory for trace.csv and summary.json")
    args = parser.parse_args(argv)
    if (args.start is None) == (not args.auto_start):
        parser.error("give exactly one of --start X Y YAW or --auto-start")
    profile = HardwareProfile.load(args.profile)
    mount = profile.laser_mount
    if mount is None:
        parser.error(f"{args.profile}: mounts.laser is not set")
    settings = IcpSettings()
    if args.race:
        settings = icp_settings(yaml.safe_load(args.race.read_text(encoding="utf-8")))
    try:
        scans, odom = load_drive(args.records)
        grid = OccupancyGrid.load(args.map)
        rows, summary = replay(scans, odom, grid, mount, None if args.auto_start else args.start,
                               settings)
    except ReplayError as exc:
        print(f"icp_replay: {exc}")
        return 1
    if args.output:
        write_outputs(args.output, rows, summary)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
