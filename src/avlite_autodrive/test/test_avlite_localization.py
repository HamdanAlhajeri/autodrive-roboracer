import math
from types import SimpleNamespace

import numpy as np
import pytest

from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c50_common.c52_world_sensor_datatypes import Lidar
from avlite_autodrive.avlite_localization import (
    AutoDRIVELidarLocalization, avlite_localization_config,
)
from avlite_autodrive.slam_localization import localization_config

SPAWN = (2.0, 1.0, math.pi / 2)
MOUNT = np.eye(4)
MOUNT[:3, 3] = [0.2733, 0.0, 0.096]


def walls():
    """An irregular corridor around the car so ICP has a unique fit."""
    t = np.linspace(-4, 4, 400)
    return np.vstack([np.c_[np.full_like(t, 1.0) + 0.1 * np.sin(2 * t), t + 1.0],
                      np.c_[np.full_like(t, 3.0) + 0.15 * np.cos(3 * t), t + 1.0],
                      np.c_[np.linspace(1.0, 3.0, 100), np.full(100, 5.5)]])


def scan_from(pose, max_range=8.0, no_return_beams=60):
    """Sensor-frame cloud (N x 4) of the walls seen from base pose, plus no-return beams."""
    x, y, yaw = pose
    sx, sy = x + 0.2733 * math.cos(yaw), y + 0.2733 * math.sin(yaw)
    world = walls()
    rel = world - [sx, sy]
    c, s = math.cos(-yaw), math.sin(-yaw)
    local = rel @ np.array([[c, -s], [s, c]]).T
    local = local[np.hypot(*local.T) < max_range]
    angles = np.linspace(-math.pi, math.pi, no_return_beams)  # no echo: put at max range
    phantom = np.c_[max_range * np.cos(angles), max_range * np.sin(angles)]
    points = np.vstack([local, phantom])
    cloud = np.zeros((len(points), 4), dtype=np.float32)
    cloud[:, :2] = points
    mask = np.r_[np.ones(len(local), bool), np.zeros(len(phantom), bool)]
    return SimpleNamespace(lidar=cloud, lidar_hit_mask=mask, lidar_sensor=Lidar(base_to_sensor=MOUNT))


def block(**extra):
    return {"localization": {"source": "avlite", "start_pose_world": list(SPAWN), **extra}}


def test_config_requires_a_spawn_pose_and_leaves_other_sources_alone():
    settings = avlite_localization_config(block())
    assert settings.start_pose_world == SPAWN
    assert (settings.scan_subsample, settings.map_subsample, settings.max_iterations) == (4, 4, 10)
    assert avlite_localization_config({}) is None
    assert avlite_localization_config({"localization": {"source": "slam"}}) is None
    assert localization_config(block(), "config/avlite.yaml") is None  # not a SLAM config


@pytest.mark.parametrize("extra, message", [
    ({"start_pose_world": None}, "start_pose_world"),
    ({"start_pose_world": [0, 0, math.inf]}, "start_pose_world"),
    ({"scan_subsample": 0}, "scan_subsample"),
    ({"max_iterations": True}, "max_iterations"),
    ({"max_correspondence_m": -1}, "max_correspondence_m"),
])
def test_invalid_settings_are_rejected_before_ros_starts(extra, message):
    with pytest.raises(ValueError, match=message):
        avlite_localization_config(block(**extra))


def test_avlite_icp_tracks_the_car_from_the_spawn_pose():
    pm = PerceptionModel(ego_vehicle=EgoState(x=0.0, y=0.0))
    speed = {"value": 2.4}
    localizer = AutoDRIVELidarLocalization(pm, avlite_localization_config(block(
        scan_subsample=1, map_subsample=1, max_iterations=30)), lambda: speed["value"])
    ego = pm.ego_vehicle
    assert (ego.x, ego.y, ego.theta) == pytest.approx(SPAWN)  # seeded before any scan

    localizer.localize(pm, scan_from(SPAWN))  # first scan becomes the reference map
    assert len(localizer._map) == pytest.approx(len(scan_from(SPAWN).lidar) - 60, abs=1)

    moved = (SPAWN[0] + 0.02, SPAWN[1] + 0.3, SPAWN[2] + 0.03)
    localizer.localize(pm, scan_from(moved))
    assert (ego.x, ego.y) == pytest.approx(moved[:2], abs=0.03)
    assert ego.theta == pytest.approx(moved[2], abs=0.01)
    assert ego.velocity == 2.4  # wheel odometry, not ground truth

    speed["value"] = None  # stale odometry leaves the last speed untouched
    localizer.localize(pm, scan_from(moved))
    assert ego.velocity == 2.4

    localizer.reset()  # simulator reset: back to the spawn pose with a fresh map
    assert (ego.x, ego.y, ego.theta) == pytest.approx(SPAWN)
    assert localizer._map is None
