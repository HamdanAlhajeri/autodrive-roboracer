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
    assert dict(settings.slam_overrides) == {"minimum_travel_distance": 0.0,
                                             "minimum_travel_heading": 0.0}
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
