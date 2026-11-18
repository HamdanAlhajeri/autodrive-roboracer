"""AVLite ICP pose for the car: seeding, tracking with odometry, health and config checks."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from avlite_roboracer.icp_pose import IcpPose, icp_settings, localization_source
from avlite_roboracer.localization import LocalizationMonitor, LocalizationThresholds
from roboracer_fixtures import commissioned_profile

avlite = pytest.importorskip("avlite")
from avlite.c50_common.c52_world_sensor_datatypes import Lidar  # noqa: E402

MOUNT_X = 0.25


def walls():
    """An irregular corridor so ICP has a unique fit."""
    t = np.linspace(-4, 4, 400)
    return np.vstack([np.c_[np.full_like(t, 1.0) + 0.1 * np.sin(2 * t), t + 1.0],
                      np.c_[np.full_like(t, 3.0) + 0.15 * np.cos(3 * t), t + 1.0],
                      np.c_[np.linspace(1.0, 3.0, 100), np.full(100, 5.5)]])


def frame(pose, max_range=8.0):
    """A sensor frame of the walls seen from base pose, with a laser mounted MOUNT_X ahead."""
    x, y, yaw = pose
    sx, sy = x + MOUNT_X * math.cos(yaw), y + MOUNT_X * math.sin(yaw)
    rel = walls() - [sx, sy]
    c, s = math.cos(-yaw), math.sin(-yaw)
    local = rel @ np.array([[c, -s], [s, c]]).T
    local = local[np.hypot(*local.T) < max_range]
    cloud = np.zeros((len(local), 4), dtype=np.float32)
    cloud[:, :2] = local
    mount = np.eye(4)
    mount[:3, 3] = [MOUNT_X, 0.0, 0.0]
    return SimpleNamespace(lidar=cloud, lidar_hit_mask=np.ones(len(local), bool),
                           lidar_sensor=Lidar(base_to_sensor=mount))


def test_seeded_pose_is_available_only_after_the_first_scan_and_tracks_the_car():
    start = (2.0, 1.0, math.pi / 2)
    icp = IcpPose(walls(), start)
    assert icp.pose is None and icp.age(0.0) == math.inf
    path = [(2.0, 1.0 + 0.05 * i, math.pi / 2) for i in range(60)]
    for i, pose in enumerate(path):
        icp.update(frame(pose), (0.05 * i, 0.0, 0.0), 1.0, now=float(i))
    x, y, yaw = icp.pose
    assert math.hypot(x - path[-1][0], y - path[-1][1]) < 0.05
    assert abs(yaw - math.pi / 2) < 0.05
    assert icp.alignment > 0.9
    assert icp.age(70.0) == pytest.approx(11.0)


def test_odometry_prediction_keeps_the_pose_ahead_of_a_one_second_scan_gap():
    # Missing scans: the car moved 0.4 m between two updates, and odometry carries ICP over.
    icp = IcpPose(walls(), (2.0, 1.0, math.pi / 2))
    icp.update(frame((2.0, 1.0, math.pi / 2)), (0.0, 0.0, 0.0), 1.0, now=0.0)
    icp.update(frame((2.0, 1.4, math.pi / 2)), (0.4, 0.0, 0.0), 1.0, now=1.0)
    assert icp.pose[1] == pytest.approx(1.4, abs=0.05)


def test_unusable_reference_or_start_is_rejected():
    with pytest.raises(ValueError, match="occupied points"):
        IcpPose(np.zeros((10, 2)), (0, 0, 0))
    with pytest.raises(ValueError, match="start pose"):
        IcpPose(walls(), (0, 0, math.nan))


def test_localization_config_defaults_and_validation():
    assert localization_source({}) == "icp"
    assert localization_source({"localization": {"source": "slam"}}) == "slam"
    with pytest.raises(ValueError, match="source"):
        localization_source({"localization": {"source": "gps"}})
    assert icp_settings({}).max_correspondence_m == 1.0
    assert icp_settings({"localization": {"scan_subsample": 2}}).scan_subsample == 2
    for bad in ({"scan_subsample": 0}, {"max_iterations": True},
                {"max_correspondence_m": -1}, {"alignment_tolerance_m": math.inf}):
        with pytest.raises(ValueError):
            icp_settings({"localization": bad})


def monitor(required):
    return LocalizationMonitor(LocalizationThresholds.from_profile(commissioned_profile()),
                               covariance_required=required)


def test_icp_health_does_not_need_a_covariance_but_slam_still_does():
    pose = (1.0, 2.0, 0.3)
    for required, expect_failure in ((False, False), (True, True)):
        m = monitor(required)
        for step in range(40):
            m.update(step * 0.1, pose, pose, None, alignment=0.95)
        failures = m.failures(3.95)
        assert any("covariance" in f for f in failures) == expect_failure
    healthy = monitor(False)
    for step in range(40):
        healthy.update(step * 0.1, pose, pose, None, alignment=0.95)
    assert healthy.failures(3.95) == []
    healthy.update(4.0, pose, pose, None, alignment=0.2)  # a scan that no longer fits the map
    assert any("does not align" in f for f in healthy.failures(4.0))


def test_race_config_ships_icp_as_the_default_source():
    from avlite_autodrive.configuration import load_config
    from roboracer_fixtures import ROOT
    config = load_config(ROOT / "config" / "roboracer" / "race.yaml")
    assert localization_source(config) == "icp"
    assert icp_settings(config).max_correspondence_m == 1.0


def test_a_lost_pose_is_recovered_by_a_grid_search():
    # A tight match distance cannot pull a pose that is 0.8 m off back onto the walls; the
    # recovery search starts from a grid of poses and finds the fit.
    from avlite_roboracer.icp_pose import IcpSettings
    truth = (2.0, 1.0, math.pi / 2)
    settings = IcpSettings(max_correspondence_m=0.15, lost_scans=2)
    icp = IcpPose(walls(), (2.8, 1.5, math.pi / 2), settings)
    for i in range(6):
        icp.update(frame(truth), (0.0, 0.0, 0.0), 0.0, now=float(i))
    assert icp.recoveries >= 1 and icp.last_recovery is not None
    x, y, yaw = icp.pose
    assert math.hypot(x - truth[0], y - truth[1]) < 0.1
    assert icp.alignment > 0.9


def test_an_icp_step_that_fits_worse_than_the_odometry_prediction_is_undone():
    truth = (2.0, 1.0, math.pi / 2)
    icp = IcpPose(walls(), truth)
    icp.update(frame(truth), (0.0, 0.0, 0.0), 0.0, now=0.0)
    localizer = icp._localizer
    localizer._icp = lambda *args: (args[2] + 1.5, args[3] - 1.0, args[4] + 0.5)  # a wild fit
    icp.update(frame(truth), (0.0, 0.0, 0.0), 0.0, now=1.0)
    x, y, yaw = icp.pose
    assert math.hypot(x - truth[0], y - truth[1]) < 0.05
    assert icp.alignment > 0.9


def test_recovery_can_be_turned_off_and_a_healthy_pose_is_left_alone():
    from avlite_roboracer.icp_pose import IcpSettings
    truth = (2.0, 1.0, math.pi / 2)
    off = IcpPose(walls(), (2.8, 1.5, math.pi / 2),
                  IcpSettings(max_correspondence_m=0.25, lost_scans=2, recovery_radius_m=0.0))
    for i in range(6):
        off.update(frame(truth), (0.0, 0.0, 0.0), 0.0, now=float(i))
    assert off.recoveries == 0 and off.recovery_tries == 0
    healthy = IcpPose(walls(), truth, IcpSettings(lost_scans=2))
    for i in range(6):
        healthy.update(frame(truth), (0.0, 0.0, 0.0), 0.0, now=float(i))
    assert healthy.recovery_tries == 0


def test_recovery_config_validation():
    assert icp_settings({}).lost_scans == 5
    assert icp_settings({"localization": {"recovery_radius_m": 0}}).recovery_radius_m == 0.0
    for bad in ({"lost_scans": 0}, {"lost_alignment": 0}, {"lost_alignment": 1.5},
                {"recovery_radius_m": -1}, {"recovery_accept": True}):
        with pytest.raises(ValueError, match="localization"):
            icp_settings({"localization": bad})
