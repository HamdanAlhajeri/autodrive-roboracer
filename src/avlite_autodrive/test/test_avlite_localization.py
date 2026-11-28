import math
from types import SimpleNamespace

import numpy as np
import pytest

from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c50_common.c52_world_sensor_datatypes import Lidar
from avlite_autodrive.avlite_localization import (
    after_interruption,
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
    return SimpleNamespace(lidar=cloud, lidar_hit_mask=mask,
                           lidar_sensor=Lidar(base_to_sensor=MOUNT))


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


def drive(localizer, poses):
    ego = localizer.perception_model.ego_vehicle
    for pose in poses:
        localizer.localize(sensors=scan_from(pose))
    return ego


def test_prebuilt_map_keeps_the_estimate_on_the_track_instead_of_the_first_scan():
    # The reference is the real walls, so the pose is correct from the first scan and
    # does not drift back toward where the car started.
    settings = avlite_localization_config(block(reference="map_hits"))
    assert settings.reference == "map_hits"
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    localizer = AutoDRIVELidarLocalization(pm, settings, lambda: 1.0, reference_points=walls())
    path = [(2.0, 1.0 + 0.05 * i, math.pi / 2) for i in range(60)]
    ego = drive(localizer, path)
    assert math.hypot(ego.x - path[-1][0], ego.y - path[-1][1]) < 0.05
    assert localizer.alignment is not None and localizer.alignment > 0.9
    assert localizer.hits > 0


def test_prebuilt_map_survives_a_reset_and_reports_poor_alignment_off_the_map():
    settings = avlite_localization_config(block(reference="map_hits"))
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    localizer = AutoDRIVELidarLocalization(pm, settings, lambda: 1.0, reference_points=walls())
    drive(localizer, [(2.0, 2.0, math.pi / 2)] * 5)
    localizer.reset()
    assert (pm.ego_vehicle.x, pm.ego_vehicle.y) == SPAWN[:2]
    assert localizer._map is not None  # seeded straight from the reference, no first scan needed
    wrong = np.array(walls()) + [5.0, 0.0]  # a reference that does not match what the car sees
    other = AutoDRIVELidarLocalization(PerceptionModel(ego_vehicle=EgoState(x=0, y=0)), settings,
                                       lambda: 1.0, reference_points=wrong)
    drive(other, [SPAWN] * 3)
    assert other.alignment < 0.5


def test_unknown_reference_is_rejected():
    with pytest.raises(ValueError, match="reference"):
        avlite_localization_config(block(reference="nonsense"))


def test_recovery_settings_default_off_and_are_validated():
    settings = avlite_localization_config(block())
    assert settings.recovery_radius_m == 0.0 and settings.lost_scans == 5
    assert avlite_localization_config(block(recovery_radius_m=1.5)).recovery_radius_m == 1.5
    for bad in ({"recovery_radius_m": -1}, {"lost_alignment": 2}, {"recovery_accept": True},
                {"lost_scans": 0}, {"recovery_interval_s": math.inf}, {"recovery_budget_s": -1}):
        with pytest.raises(ValueError, match="localization"):
            avlite_localization_config(block(**bad))


def lost_localizer(**extra):
    """A localizer whose estimate is 0.8 m off, with a match distance too tight to fix it."""
    settings = avlite_localization_config(block(
        reference="map_hits", max_correspondence_m=0.15, lost_scans=2, recovery_interval_s=0.0,
        **extra))
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    localizer = AutoDRIVELidarLocalization(pm, settings, lambda: 1.0, reference_points=walls())
    localizer._x, localizer._y = SPAWN[0] + 0.8, SPAWN[1] + 0.5
    return localizer


def test_a_lost_estimate_is_searched_for_and_recovered():
    localizer = lost_localizer(recovery_radius_m=1.5)
    ego = drive(localizer, [SPAWN] * 6)
    assert localizer.recoveries >= 1
    assert math.hypot(ego.x - SPAWN[0], ego.y - SPAWN[1]) < 0.1
    assert localizer.alignment > 0.9


def test_recovery_off_leaves_a_lost_estimate_alone_and_a_healthy_one_is_not_searched():
    off = lost_localizer()
    drive(off, [SPAWN] * 6)
    assert off.recoveries == 0 and off.recovery_tries == 0
    settings = avlite_localization_config(block(reference="map_hits", recovery_radius_m=1.5,
                                                lost_scans=2))
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    healthy = AutoDRIVELidarLocalization(pm, settings, lambda: 1.0, reference_points=walls())
    drive(healthy, [SPAWN] * 6)
    assert healthy.recovery_tries == 0


def test_a_sensor_gap_keeps_the_estimate_but_a_teleport_returns_to_the_spawn_pose():
    settings = avlite_localization_config(block(reference="map_hits"))
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    localizer = AutoDRIVELidarLocalization(pm, settings, lambda: 1.0, reference_points=walls())
    ego = drive(localizer, [(2.0, 1.0 + 0.1 * i, math.pi / 2) for i in range(1, 30)])
    here = (ego.x, ego.y, ego.theta)
    assert here[1] > 3.0
    after_interruption(localizer, teleported=False)  # sensors dropped for a moment
    assert (ego.x, ego.y, ego.theta) == pytest.approx(here)
    assert localizer._map is not None
    after_interruption(localizer, teleported=True)  # the simulator respawned the car
    assert (ego.x, ego.y, ego.theta) == pytest.approx(SPAWN)


def test_a_wild_odometry_step_after_a_sensor_gap_does_not_move_the_estimate():
    # During a sensor gap the wheel odometry can report a big, wrong jump. The first update
    # afterwards keeps whichever of the old and the odometry-moved pose fits the scan better.
    settings = avlite_localization_config(block(reference="map_hits"))
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    odom = {"pose": (0.0, 0.0, 0.0)}
    localizer = AutoDRIVELidarLocalization(pm, settings, lambda: 1.0, reference_points=walls(),
                                           odometry_pose=lambda: odom["pose"])
    ego = drive(localizer, [SPAWN] * 3)
    here = (ego.x, ego.y)
    after_interruption(localizer, teleported=False)
    odom["pose"] = (6.0, 0.0, 0.0)  # the odometry claims the car moved 6 m during the gap
    localizer.localize(sensors=scan_from(SPAWN))
    assert math.hypot(ego.x - here[0], ego.y - here[1]) < 0.1
    assert localizer.alignment > 0.9


def test_a_search_with_a_time_budget_tries_the_nearest_candidates_first_and_stops():
    localizer = lost_localizer(recovery_radius_m=3.0)
    scan = scan_from(SPAWN)
    calls = []
    real_icp = localizer._icp
    localizer._icp = lambda *args: calls.append(args) or real_icp(*args)
    localizer.recover(scan, radius_m=3.0, budget_s=0.0)
    # the first candidate always runs, then the spent budget stops the search
    assert len(calls) == 1
    assert (calls[0][2], calls[0][3]) == (localizer._x, localizer._y)  # nearest first: no offset
    calls.clear()
    localizer.recover(scan, radius_m=1.0)  # no budget: the whole grid, 5 x 5 x 3 starts
    assert len(calls) == 75


def odometry_localizer(limit):
    settings = avlite_localization_config(
        block(reference="map_hits", odometry_max_accel_mps2=limit))
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    clock = {"t": 0.0}
    odom = {"x": 0.0}
    localizer = AutoDRIVELidarLocalization(pm, settings, lambda: 1.0, reference_points=walls(),
                                           odometry_pose=lambda: (odom["x"], 0.0, 0.0),
                                           clock=lambda: clock["t"])
    return localizer, clock, odom


def advance(localizer, clock, odom, forward_m, dt=0.05):
    clock["t"] += dt
    odom["x"] += forward_m
    before = (localizer._x, localizer._y)
    localizer._predict()
    return math.hypot(localizer._x - before[0], localizer._y - before[1])


def test_wheel_slip_at_a_launch_cannot_advance_the_estimate_faster_than_the_car_accelerates():
    # The wheels spin and odometry reports 1 m per 50 ms step from a standstill (20 m/s at once).
    limited, clock, odom = odometry_localizer(3.3)
    advance(limited, clock, odom, 0.0)  # first call only records the starting odometry
    moved = sum(advance(limited, clock, odom, 1.0) for _ in range(20))  # one second of slip
    assert moved <= 0.5 * 3.3 * 1.0 ** 2 * 1.1  # at most the distance of a 3.3 m/s^2 launch
    assert limited.odometry_clipped == 20
    free, clock, odom = odometry_localizer(0.0)
    advance(free, clock, odom, 0.0)
    assert sum(advance(free, clock, odom, 1.0) for _ in range(20)) == pytest.approx(20.0)
    assert free.odometry_clipped == 0


def test_honest_odometry_during_a_normal_launch_and_at_speed_is_not_clipped():
    localizer, clock, odom = odometry_localizer(3.3)
    advance(localizer, clock, odom, 0.0)
    speed = 0.0
    for _ in range(60):  # 3 m/s^2 up to 5 m/s, then steady
        speed = min(5.0, speed + 3.0 * 0.05)
        advance(localizer, clock, odom, speed * 0.05)
    assert localizer.odometry_clipped == 0
    assert advance(localizer, clock, odom, 0.0) == 0.0  # standing still moves nothing


def test_a_respawn_or_sensor_gap_does_not_let_the_acceleration_limit_widen():
    localizer, clock, odom = odometry_localizer(3.3)
    advance(localizer, clock, odom, 0.0)
    for _ in range(40):
        advance(localizer, clock, odom, 0.2)  # reaching 4 m/s
    localizer.reset()  # respawned at rest
    advance(localizer, clock, odom, 0.0)
    assert advance(localizer, clock, odom, 1.0) < 0.05  # the speed was reset to zero, not 4 m/s
    after_gap, clock2, odom2 = odometry_localizer(3.3)
    advance(after_gap, clock2, odom2, 0.0)
    after_gap.resume()
    # The time across a gap is unknown, so that one step is not limited here (the first-update check
    # after a gap judges it against the scan); the steps after it are limited again.
    clock2["t"] += 5.0
    assert advance(after_gap, clock2, odom2, 10.0) == pytest.approx(10.0)
    assert advance(after_gap, clock2, odom2, 10.0) < 0.1


def test_odometry_limit_setting_is_validated():
    assert avlite_localization_config(block()).odometry_max_accel_mps2 == 0.0
    for bad in (-1, 99, True):
        with pytest.raises(ValueError, match="localization"):
            avlite_localization_config(block(odometry_max_accel_mps2=bad))


def test_pose_lead_moves_only_the_published_pose_forward_by_speed_times_lead():
    settings = avlite_localization_config(block(reference="map_hits", pose_lead_s=0.05))
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    localizer = AutoDRIVELidarLocalization(pm, settings, lambda: 4.0, reference_points=walls())
    ego = drive(localizer, [SPAWN] * 3)
    heading = (math.cos(localizer._theta), math.sin(localizer._theta))
    assert (ego.x - localizer._x, ego.y - localizer._y) == pytest.approx(
        (4.0 * 0.05 * heading[0], 4.0 * 0.05 * heading[1]), abs=1e-9)
    # ICP estimate is untouched
    assert math.hypot(localizer._x - SPAWN[0], localizer._y - SPAWN[1]) < 0.05
    drive(localizer, [SPAWN] * 3)  # a second round does not stack the lead
    assert math.hypot(ego.x - localizer._x, ego.y - localizer._y) == pytest.approx(0.2, abs=1e-6)
    off = AutoDRIVELidarLocalization(PerceptionModel(ego_vehicle=EgoState(x=0, y=0)),
                                     avlite_localization_config(block(reference="map_hits")),
                                     lambda: 4.0, reference_points=walls())
    drive(off, [SPAWN] * 3)
    ego = off.perception_model.ego_vehicle
    assert (ego.x, ego.y) == pytest.approx((off._x, off._y))
    for bad in (-1, 1, True):
        with pytest.raises(ValueError, match="localization"):
            avlite_localization_config(block(pose_lead_s=bad))


def test_the_acceleration_limit_can_apply_only_during_the_launch():
    settings = avlite_localization_config(block(
        reference="map_hits", odometry_max_accel_mps2=3.3, odometry_limit_below_mps=3.0))
    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    clock, odom = {"t": 0.0}, {"x": 0.0}
    localizer = AutoDRIVELidarLocalization(pm, settings, lambda: 1.0, reference_points=walls(),
                                           odometry_pose=lambda: (odom["x"], 0.0, 0.0),
                                           clock=lambda: clock["t"])
    advance(localizer, clock, odom, 0.0)
    slip = sum(advance(localizer, clock, odom, 0.2) for _ in range(10))  # wheel slip: 4 m/s at once
    assert slip < 1.0 and localizer.odometry_clipped == 10
    for _ in range(40):  # reach 5 m/s honestly, then cruise with a noisy speed spike
        advance(localizer, clock, odom, 0.25)
    clipped = localizer.odometry_clipped
    # a spike at cruise passes through
    assert advance(localizer, clock, odom, 0.40) == pytest.approx(0.40)
    assert localizer.odometry_clipped == clipped
