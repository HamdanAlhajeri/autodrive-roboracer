"""Synthetic tracks, grids, profiles and recordings shared by the hardware tests."""

import copy
import math
from pathlib import Path

import numpy as np
import yaml

from avlite_roboracer.geometry import compose, inverse
from avlite_roboracer.occupancy import FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from avlite_roboracer.profile import HardwareProfile

ROOT = Path(__file__).resolve().parents[3]
PROFILE_PATH = ROOT / "config" / "roboracer" / "hardware.yaml"

MEASURED = {
    "mounts": {"laser": {"x_m": 0.25, "y_m": 0.0, "yaw_rad": 0.0}},
    "vehicle": {"wheelbase_m": 0.33, "width_m": 0.30, "length_m": 0.55,
                "rear_axle_to_front_m": 0.45},
    "steering": {"max_angle_rad": 0.40, "positive_left": True, "center_offset_rad": 0.0},
    "motor": {"max_command_speed_mps": 3.0, "stop_command_speed_mps": 0.0},
    "measurements": {"command_latency_s": 0.08, "braking_deceleration_mps2": 2.0,
                     "max_acceleration_mps2": 1.5, "stop_test_speed_mps": 2.5,
                     "stopping_distance_m": 1.8},
    "limits": {"mapping_speed_mps": 0.8, "racing_speed_mps": 2.0},
    "localization": {"max_position_stddev_m": 0.10, "max_yaw_stddev_rad": 0.08,
                     "min_scan_alignment": 0.7, "scan_alignment_tolerance_m": 0.10,
                     "max_motion_mismatch_m": 0.10, "max_motion_mismatch_rad": 0.08,
                     "stable_duration_s": 2.0},
}


def raw_profile():
    return yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))


def commissioned_data(**overrides):
    data = raw_profile()
    for section, values in MEASURED.items():
        data.setdefault(section, {})
        for key, value in values.items():
            if isinstance(value, dict):
                data[section][key] = {**data[section].get(key, {}), **value}
            else:
                data[section][key] = value
    for dotted, value in overrides.items():
        target = data
        keys = dotted.split(".")
        for key in keys[:-1]:
            target = target[key]
        target[keys[-1]] = value
    return data


def commissioned_profile(**overrides):
    return HardwareProfile(commissioned_data(**overrides))


def annulus_grid(inner=3.0, outer=5.0, resolution=0.05, wall=0.1, frame="map",
                 extra=None):
    """Square grid with circular walls; free between them, unknown elsewhere.

    extra(xc, yc, cells) may edit the cell array given cell-centre coordinates.
    """
    size = int(2 * (outer + 1.5) / resolution)
    origin = (-(outer + 1.5), -(outer + 1.5))
    xs = origin[0] + (np.arange(size) + 0.5) * resolution
    xc, yc = np.meshgrid(xs, xs)
    r = np.hypot(xc, yc)
    cells = np.full((size, size), UNKNOWN, dtype=np.int16)
    cells[(r > inner) & (r < outer)] = FREE
    cells[(np.abs(r - inner) <= wall / 2) | (np.abs(r - outer) <= wall / 2)] = OCCUPIED
    if extra is not None:
        extra(xc, yc, cells)
    return OccupancyGrid(cells, resolution, origin, frame)


def circle_nodes(radius=4.0, count=300, laps=1.05, start_angle=0.0):
    """Pose-graph nodes on a CCW circle, ordered by id, slightly over one lap."""
    angles = start_angle + np.linspace(0, 2 * np.pi * laps, int(count * laps))
    return [[i, radius * math.cos(a), radius * math.sin(a)] for i, a in enumerate(angles)]


def circle_pose(angle, radius=4.0):
    """CCW tangent pose on a circle (heading = angle + 90 degrees)."""
    return (radius * math.cos(angle), radius * math.sin(angle), angle + math.pi / 2)


def ray_scan(grid, pose, mount=(0.25, 0.0, 0.0), beams=180, max_range=8.0):
    """Ray-cast a LaserScan-like record from base_link pose in the grid."""
    sensor = compose(pose, mount)
    angles = -math.pi + np.arange(beams) * (2 * math.pi / beams)
    ranges = []
    for a in angles:
        direction = (math.cos(sensor[2] + a), math.sin(sensor[2] + a))
        distance, outcome = grid.cast(sensor[:2], direction, max_range)
        ranges.append(round(distance, 4) if outcome == "occupied" else None)
    return {"angle_min": -math.pi, "angle_increment": 2 * math.pi / beams,
            "range_min": 0.05, "range_max": max_range, "ranges": ranges}


def synthetic_recording(grid=None, seconds=6.0, speed=0.8, radius=4.0, odom_drift=0.0,
                        with_pose=True, pose_noise=0.0, jump_at=None, imu_sign=1.0,
                        seed=1):
    """Records for a car circling the annulus: scans 10 Hz, odom 50 Hz, tf, optional pose.

    odom_drift adds a slowly growing yaw error to odometry so map->odom is non-trivial;
    jump_at (seconds) injects a 0.5 m localization correction into the map estimate.
    """
    rng = np.random.default_rng(seed)
    grid = grid or annulus_grid()
    omega = speed / radius
    records = []
    t0 = 100.0

    def ns(t):
        return int(round((t0 + t) * 1e9))

    def truth(t):
        return circle_pose(omega * t, radius)

    def odom_pose(t):
        # Odometry frame starts at the true start pose; drift rotates it slowly.
        start = truth(0.0)
        relative = compose(inverse(start), truth(t))
        drift = (0.0, 0.0, odom_drift * t)
        return compose(drift, relative)

    def map_to_odom(t):
        return compose(truth(t), inverse(odom_pose(t)))

    records.append({"kind": "tf_static", "topic": "/tf_static", "rx_ns": ns(0) - 1000,
                    "transforms": [{"parent": "base_link", "child": "laser", "stamp_ns": ns(0),
                                    "x": 0.25, "y": 0.0, "yaw": 0.0}]})
    for i in range(int(seconds * 50) + 1):
        t = i / 50
        o = odom_pose(t)
        records.append({"kind": "odom", "topic": "/odom", "rx_ns": ns(t) + 2_000_000,
                        "frame": "odom", "child": "base_link", "stamp_ns": ns(t),
                        "x": o[0], "y": o[1], "yaw": o[2], "vx": speed, "vy": 0.0,
                        "wz": omega + odom_drift, "cov": [0.01, 0.01, 0.01]})
        records.append({"kind": "tf", "topic": "/tf", "rx_ns": ns(t) + 2_000_000,
                        "transforms": [{"parent": "odom", "child": "base_link",
                                        "stamp_ns": ns(t), "x": o[0], "y": o[1],
                                        "yaw": o[2]}]})
        records.append({"kind": "imu", "topic": "/imu", "rx_ns": ns(t) + 1_000_000,
                        "frame": "imu", "stamp_ns": ns(t), "wz": imu_sign * omega, "ax": 0.0})
    for i in range(int(seconds * 10) + 1):
        t = i / 10
        scan = ray_scan(grid, truth(t))
        records.append({"kind": "scan", "topic": "/scan", "rx_ns": ns(t) + 5_000_000,
                        "frame": "laser", "stamp_ns": ns(t), **scan})
        correction = (0.5, 0.0, 0.0) if jump_at is not None and t >= jump_at else (0, 0, 0)
        m2o = compose(correction, map_to_odom(t))
        records.append({"kind": "tf", "topic": "/tf", "rx_ns": ns(t) + 30_000_000,
                        "transforms": [{"parent": "map", "child": "odom", "stamp_ns": ns(t),
                                        "x": m2o[0], "y": m2o[1], "yaw": m2o[2]}]})
        if with_pose:
            estimate = compose(correction, truth(t))
            noise = rng.normal(0, pose_noise, 3) if pose_noise else (0, 0, 0)
            records.append({"kind": "pose", "topic": "/pose", "rx_ns": ns(t) + 40_000_000,
                            "frame": "map", "stamp_ns": ns(t),
                            "x": estimate[0] + noise[0], "y": estimate[1] + noise[1],
                            "yaw": estimate[2] + noise[2], "cov": [0.0004, 0.0004, 0.0004]})
    records.sort(key=lambda r: r["rx_ns"])
    return records, grid, truth, t0


def deep(data):
    return copy.deepcopy(data)
