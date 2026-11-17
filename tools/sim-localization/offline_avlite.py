"""Replay a recorded mapping run through AVLite's LidarLocalization and score it (no sim).

    python3 offline_avlite.py EXPERIMENT_DIR [--seconds 40] [--every 2] [--set key=value ...]

Reads LiDAR scans from EXPERIMENT_DIR/mapping/bag and the simulator's true pose from
EXPERIMENT_DIR/truth, seeds the localizer at the first true pose (the spawn), feeds it
every Nth scan as the runner would, and prints the position/yaw error over time.
Needs the AVLite image (rosbag2_py, avlite) with src/avlite_autodrive on PYTHONPATH.
"""

import argparse
import bisect
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml


def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    meta = yaml.safe_load((Path(path) / "metadata.yaml").read_text())
    storage = meta["rosbag2_bagfile_information"]["storage_identifier"]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, types[topic])
        yield topic, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg


def main():
    from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from avlite_autodrive.avlite_localization import (
        AutoDRIVELidarLocalization, avlite_localization_config)
    from avlite_autodrive.ros_utils import odom_state
    from avlite_autodrive.sensors import scan_cloud, scan_hit_mask

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("experiment", type=Path)
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--every", type=int, default=2, help="use every Nth scan (35 Hz in)")
    parser.add_argument("--set", nargs="*", default=[], help="localization key=value")
    parser.add_argument("--no-odometry", action="store_true",
                        help="ICP alone, without the wheel-odometry prediction")
    args = parser.parse_args()

    truth = []
    for _, t, msg in read_bag(args.experiment / "truth", ["/autodrive/roboracer_1/odom"]):
        truth.append((t, odom_state(msg)[:3]))
    truth.sort()
    times = [t for t, _ in truth]

    def true_at(t):
        i = min(max(bisect.bisect_left(times, t), 0), len(times) - 1)
        return truth[i][1]

    mount = np.eye(4)
    mount[:3, 3] = [0.2733, 0.0, 0.096]
    sensor = Lidar(base_to_sensor=mount)
    scans, odom = [], []
    for topic, t, m in read_bag(args.experiment / "mapping" / "bag",
                                ["/autodrive/roboracer_1/scan", "/autodrive/roboracer_1/wheel_odom"]):
        (scans if topic.endswith("scan") else odom).append((t, m))
    scans.sort(key=lambda s: s[0])
    odom.sort(key=lambda s: s[0])
    odom_times = [t for t, _ in odom]
    current = {"t": None}

    def odometry_pose():  # latest wheel odometry at or before the current scan
        i = bisect.bisect_right(odom_times, current["t"]) - 1
        return None if i < 0 else odom_state(odom[i][1])[:3]
    start_t = scans[0][0]
    block = {"source": "avlite", "start_pose_world": list(true_at(start_t))}
    for item in args.set:
        key, value = item.split("=", 1)
        block[key] = yaml.safe_load(value)
    settings = avlite_localization_config({"localization": block})
    pm = PerceptionModel(ego_vehicle=EgoState(x=0.0, y=0.0))
    localizer = AutoDRIVELidarLocalization(
        pm, settings, lambda: None, None if args.no_odometry else odometry_pose)
    print("settings:", settings)
    print("  t(s)   err(m)  yaw(rad)  est_x  est_y   true_x true_y")
    for n, (t, msg) in enumerate(scans):
        if t - start_t > args.seconds:
            break
        if n % args.every:
            continue
        current["t"] = t
        frame = SimpleNamespace(lidar=scan_cloud(msg), lidar_hit_mask=scan_hit_mask(msg),
                                lidar_sensor=sensor)
        localizer.localize(pm, frame)
        ego, true = pm.ego_vehicle, true_at(t)
        if n % (args.every * 35) == 0:
            yaw = math.atan2(math.sin(ego.theta - true[2]), math.cos(ego.theta - true[2]))
            print(f"{t - start_t:6.1f} {math.hypot(ego.x - true[0], ego.y - true[1]):8.3f} "
                  f"{abs(yaw):8.3f}  {ego.x:6.2f} {ego.y:6.2f}  {true[0]:6.2f} {true[1]:6.2f}")


if __name__ == "__main__":
    main()
