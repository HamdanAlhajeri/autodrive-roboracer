import math
from pathlib import Path

import pytest

from avlite_autodrive.configuration import load_config
from avlite_autodrive.slam_localization import (
    apply_slam_overrides, localization_config, to_map, to_world,
)

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "config" / "avlite.yaml"


def slam_block(**overrides):
    block = {"source": "slam", "pose_graph": "maps/slam/practice/posegraph",
             "map_yaml": "maps/slam/practice/map.yaml", "start_pose": [1.0, 2.0, 0.5]}
    block.update(overrides)
    return {"localization": block}


def test_repository_config_keeps_ground_truth_by_default():
    assert localization_config(load_config(CONFIG), CONFIG) is None
    assert localization_config({}, CONFIG) is None


def test_slam_config_resolves_paths_next_to_the_config_file():
    settings = localization_config(slam_block(), CONFIG)
    config_dir = CONFIG.parent.resolve()
    assert settings.pose_graph == config_dir / "maps/slam/practice/posegraph"
    assert settings.profile == config_dir / "roboracer/sim_profile.yaml"
    assert settings.start_pose == (1.0, 2.0, 0.5)
    assert settings.map_origin_world is None


@pytest.mark.parametrize("overrides, message", [
    ({"source": "gps"}, "source"),
    ({"pose_graph": ""}, "pose_graph"),
    ({"map_yaml": None}, "map_yaml"),
    ({"start_pose": [0, 0]}, "start_pose"),
    ({"start_pose": [0, 0, math.nan]}, "start_pose"),
    ({"map_origin_world": [0, True, 0]}, "map_origin_world"),
    ({"slam_overrides": ["minimum_travel_distance"]}, "slam_overrides"),
    ({"slam_overrides": {"minimum_travel_distance": [0.0]}}, "slam_overrides"),
])
def test_invalid_slam_settings_are_rejected_before_ros_starts(overrides, message):
    with pytest.raises(ValueError, match=message):
        localization_config(slam_block(**overrides), CONFIG)


def test_map_origin_aligns_slam_poses_with_the_world_frame():
    origin = (3.0, -1.0, math.pi / 2)  # map started at (3, -1) facing +y in the world
    world = to_world((2.0, 0.0, 0.0), origin)  # 2 m ahead of the map origin
    assert world == pytest.approx((3.0, 1.0, math.pi / 2))
    assert to_map(world, origin) == pytest.approx((2.0, 0.0, 0.0))
    assert to_world((1.0, 2.0, 0.3), None) == (1.0, 2.0, 0.3)


def test_localization_matches_scans_while_the_car_is_parked(tmp_path):
    """Zero travel thresholds by default, so SLAM reports a pose before the car moves."""
    settings = localization_config(slam_block(), CONFIG)
    assert {k: v for k, v in settings.slam_overrides if k.startswith("minimum_travel")} == {
        "minimum_travel_distance": 0.0, "minimum_travel_heading": 0.0}
    assert dict(settings.slam_overrides)["correlation_search_space_resolution"] == 0.025
    custom = localization_config(slam_block(slam_overrides={"minimum_time_interval": 0.2}),
                                 CONFIG)
    assert dict(custom.slam_overrides)["minimum_time_interval"] == 0.2
    assert dict(custom.slam_overrides)["minimum_travel_distance"] == 0.0

    import yaml
    from avlite_roboracer.profile import HardwareProfile
    from avlite_roboracer.slam import effective_parameters
    profile = HardwareProfile.load(settings.profile)
    params = effective_parameters(settings.slam_params, profile, "localization",
                                  tmp_path / "slam.yaml", map_file="graph",
                                  start_pose=settings.start_pose)
    apply_slam_overrides(params, custom.slam_overrides)
    written = yaml.safe_load(params.read_text())["slam_toolbox"]["ros__parameters"]
    assert written["minimum_travel_distance"] == 0.0
    assert written["minimum_travel_heading"] == 0.0
    assert written["minimum_time_interval"] == 0.2
    assert written["mode"] == "localization"
    base = yaml.safe_load(settings.slam_params.read_text())["slam_toolbox"]["ros__parameters"]
    assert base["minimum_travel_distance"] == 0.2  # the car's shared file is unchanged


class _Time:
    def __init__(self, ns=0):
        self.nanoseconds = ns

    @classmethod
    def from_msg(cls, msg):
        return cls(msg.ns)

    def __sub__(self, other):
        return _Time(self.nanoseconds - other.nanoseconds)


class _FakeTime:
    Time = _Time


class _Clock:
    def now(self):
        return _Time(10_000_000_000)


class _Node:
    def get_clock(self):
        return _Clock()


def _transform(x, y, yaw, stamp_ns):
    from types import SimpleNamespace as NS
    return NS(header=NS(stamp=NS(ns=stamp_ns)),
              transform=NS(translation=NS(x=x, y=y, z=0.0),
                           rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2),
                                       w=math.cos(yaw / 2))))


class _LaggingTf:
    """map -> odom only exists at an older stamp; odom -> base_link is current."""

    def lookup_transform(self, target, source, time):
        if (target, source) == ("map", "odom"):
            return _transform(1.0, 2.0, math.pi / 2, 9_400_000_000)
        if (target, source) == ("odom", "base_link"):
            return _transform(0.5, 0.0, 0.0, 9_980_000_000)
        raise LookupError("would extrapolate into the future")


def test_map_pose_uses_latest_correction_with_current_odometry():
    from avlite_autodrive.slam_localization import SlamPoseSource
    source = SlamPoseSource.__new__(SlamPoseSource)  # no ROS: only the TF math
    source.frames = {"map": "map", "odom": "odom", "base_link": "base_link"}
    source.tf, source.node, source.rclpy_time = _LaggingTf(), _Node(), _FakeTime
    x, y, yaw, age = source.map_pose(_Time(9_980_000_000))
    assert (x, y) == pytest.approx((1.0, 2.5))
    assert yaw == pytest.approx(math.pi / 2)
    assert age == pytest.approx(0.02)  # odometry's age, not the lagging correction's
    assert source.lookup("map") is None  # the single chained lookup this replaces fails
