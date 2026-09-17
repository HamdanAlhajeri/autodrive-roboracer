"""Real pinned AVLite planning, geometry, periodic control and stopping regressions."""

import copy
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
from avlite.c20_planning.c26_local_path_planners import ReferencePathPlanner
from avlite.c30_control.c35_pure_pursuit import PurePursuitController
from avlite.c30_control.c39_settings import ControlSettings
from avlite.c50_common.c52_world_sensor_datatypes import Lidar, SensorFrame

from avlite_autodrive.configuration import load_config
from avlite_autodrive.plugin.planned_controller import (
    AutoDRIVEPlannedController, AutoDRIVESensorFrame,
)
from avlite_autodrive.race_map import ClosedPath, build_map, clean_lap, validate_map
from avlite_autodrive.race_planning import PlanningConfig, prepare_plan
from avlite_autodrive.sensors import scan_cloud, scan_hit_mask


def circle_map(radius=3.0, width=1.4):
    angles = np.arange(0, 2 * np.pi, 0.025)
    directions = np.column_stack([np.cos(angles), np.sin(angles)])
    return {"LeftBound": ((radius - width / 2) * directions).tolist(),
            "RightBound": ((radius + width / 2) * directions).tolist(),
            "ReferencePoint": [0, 0], "frame_id": "world", "units": "m", "closed": True}


@pytest.fixture
def profile(monkeypatch):
    control = {
        "c32_ego_distance_front_axle": 0.324, "c32_ego_max_velocity": 5.0,
        "c32_ego_max_acceleration": 1.0, "c32_ego_min_acceleration": -2.0,
        "c32_ego_max_steering": math.pi / 6, "c32_ego_min_steering": -math.pi / 6,
        "c35_cruise_velocity": 5.0, "c35_lookahead_speed_gain": 0.4,
        "c35_min_lookahead": 0.6, "c35_max_lookahead": 1.8,
        "c35_valpha": 1.0, "c35_vbeta": 0.0, "c35_vgamma": 0.0,
    }
    for key, value in control.items():
        monkeypatch.setattr(ControlSettings, key, value)
    return {"c30_control": control, "planning": {"braking_calibrated": True}}


@pytest.fixture
def prepared(tmp_path, profile):
    filename = tmp_path / "track.json"
    filename.write_text(json.dumps(circle_map()))
    profile["planning"]["map_path"] = str(filename)
    return prepare_plan(profile, tmp_path / "avlite.yaml")


def state_at(prepared, s, speed=1.0):
    xy = prepared.path.at(s)
    tangent = prepared.path.at(s + 0.01) - xy
    return EgoState(x=float(xy[0]), y=float(xy[1]),
                    theta=float(np.arctan2(tangent[1], tangent[0])), velocity=speed)


def clear_scan():
    scan = SimpleNamespace(ranges=[10.0] * 64, range_min=0.06, range_max=10.0,
                           angle_min=-np.pi, angle_increment=2 * np.pi / 64)
    return AutoDRIVESensorFrame(lidar=scan_cloud(scan), lidar_hit_mask=scan_hit_mask(scan),
                                lidar_age_s=0.01, odom_age_s=0.01)


def test_periodic_interpolation_and_projection():
    p = ClosedPath([[0, 0], [1, 0], [1, 1], [0, 1]])
    np.testing.assert_allclose(p.at(3.5), [0, 0.5])
    np.testing.assert_allclose(p.at(4.5), [0.5, 0])
    assert p.project([0, 0.5])[0] == pytest.approx(3.5)
    assert p.velocity_at(3.5, [2, 2, 1, 1]) == pytest.approx(math.sqrt(2.5))


@pytest.mark.parametrize("change", [
    lambda d: d.update(LeftBound=d["RightBound"], RightBound=d["LeftBound"]),
    lambda d: d["LeftBound"].pop(),
    lambda d: d.update(closed=False),
    lambda d: d.update(frame_id="lidar"),
    lambda d: d.update(ReferencePoint=[float("nan"), 0]),
    lambda d: d.update(LeftBound=d["LeftBound"][::12], RightBound=d["RightBound"][::12]),
])
def test_invalid_maps_rejected(change):
    data = circle_map()
    change(data)
    with pytest.raises(ValueError):
        validate_map(data)


def test_narrow_and_crossing_maps_rejected():
    with pytest.raises(ValueError, match="narrow"):
        validate_map(circle_map(width=0.5))
    data = circle_map()
    data["LeftBound"][40:80] = list(reversed(data["LeftBound"][40:80]))
    with pytest.raises(ValueError):
        validate_map(data)


def mapping_samples():
    angles = np.linspace(0, 6 * np.pi, 600, endpoint=False)
    return [dict(x=3 * np.cos(a), y=3 * np.sin(a), lap_count=i // 200,
                 collision_count=0, resets=0, odom_age_s=0.01, lap_count_age_s=0.01,
                 collision_count_age_s=0.01) for i, a in enumerate(angles)]


def test_offline_map_from_recorded_lap_and_hits():
    data = circle_map()
    outline = {"version": 1, "frame_id": "world", "units": "m",
               "points": data["LeftBound"] + data["RightBound"]}
    result = build_map(mapping_samples(), outline)
    validate_map(result)
    assert result["ReferencePoint"] == [0, 0]
    assert result["spacing_m"] == 0.1
    assert len(result["LeftBound"]) > 180
    outline["points"] = data["LeftBound"]
    with pytest.raises(ValueError):
        build_map(mapping_samples(), outline)


@pytest.mark.parametrize("failure", ["partial", "collision", "reset", "stale", "jump"])
def test_mapping_requires_complete_clean_fresh_lap(failure):
    rows = mapping_samples()
    if failure == "partial":
        rows = rows[:300]
    elif failure == "collision":
        rows[230]["collision_count"] = 1
    elif failure == "reset":
        rows[230]["resets"] = 1
    elif failure == "stale":
        rows[230]["odom_age_s"] = 0.6
    else:
        rows[230]["x"] = 100
    with pytest.raises(ValueError):
        clean_lap(rows)


def test_upstream_plan_limits_include_closing_segment(prepared):
    assert prepared.planner.RESAMPLE_STEP == 0.1
    v = np.asarray(prepared.global_plan.velocity)
    k = np.asarray(prepared.artifact["curvature"])
    ds = prepared.path.ds
    assert len(v) > 150
    assert max(v) <= 5.0
    assert np.all(v**2 * k <= 3.0 + 1e-5)
    dv2 = np.roll(v, -1)**2 - v**2
    assert np.all(dv2 <= 2 * ds + 1e-5)
    assert np.all(-dv2 <= 3 * ds + 1e-5)


def test_pure_pursuit_wraps_finish_line_and_starts_mid_lap(prepared):
    controller = AutoDRIVEPlannedController(prepared)
    assert isinstance(controller, PurePursuitController)
    local = ReferencePathPlanner(prepared.global_plan, PerceptionModel())
    outputs = []
    for s in (prepared.path.length - 0.1, 0.05, prepared.path.length / 2):
        ego = state_at(prepared, s)
        local.replan()
        cmd = controller.control(ego, local.get_local_plan(), sensors=clear_scan())
        assert 0.04 < cmd.steer < 0.2
        assert cmd.acceleration > 0
        assert controller.diagnostics["plan_valid"] == 1
        outputs.append(cmd.steer)
    assert abs(outputs[0] - outputs[1]) < 0.02
    controller.cte_v_sum = 30
    controller.reset()
    assert controller.cte_v_sum == 0
    assert controller._s is None


def test_no_return_is_not_an_obstacle_and_corrupt_beams_are_not_clear():
    scan = SimpleNamespace(ranges=[math.inf, 10.0, 1.0, math.nan, -math.inf, 0.0],
                           range_min=0.06, range_max=10.0, angle_min=0, angle_increment=0.1)
    np.testing.assert_array_equal(scan_hit_mask(scan), [False, False, True, True, True, True])
    assert np.isfinite(scan_cloud(scan)).all()


def test_obstacle_on_curved_path_and_sensor_mount(prepared):
    controller = AutoDRIVEPlannedController(prepared)
    ego = state_at(prepared, 0, speed=2.5)
    frame = clear_scan()
    obstacle = prepared.path.at(0.8)
    # Convert world hit into a translated/rotated sensor frame.
    c, sn = np.cos(ego.theta), np.sin(ego.theta)
    rot = np.array([[c, -sn], [sn, c]])
    body = (obstacle - [ego.x, ego.y]) @ rot
    mount = np.eye(4)
    mount[:3, 3] = [0.2733, 0, 0.096]
    frame.lidar_sensor = Lidar(base_to_sensor=mount)
    frame.lidar[0, :2] = body - mount[:2, 3]
    frame.lidar_hit_mask[0] = True
    cmd = controller.control(ego, prepared.global_plan, sensors=frame)
    assert cmd.acceleration <= -1.5
    assert controller.diagnostics["speed_limit_reason"] == 2
    assert controller.diagnostics["target_velocity_mps"] < 1.0
    assert controller.diagnostics["clearance_m"] < 0.8


@pytest.mark.parametrize("failure", ["scan", "age", "pose", "heading", "plan"])
def test_bad_inputs_request_deceleration_and_reset(prepared, failure):
    controller = AutoDRIVEPlannedController(prepared)
    ego, frame, plan = state_at(prepared, 1), clear_scan(), prepared.global_plan
    controller.control(ego, plan, sensors=frame)
    if failure == "scan":
        frame.lidar = None
    elif failure == "age":
        frame.odom_age_s = 0.51
    elif failure == "pose":
        ego.x = 100
    elif failure == "heading":
        ego.theta += math.pi
    else:
        plan = None
    cmd = controller.control(ego, plan, sensors=frame)
    assert cmd.acceleration == -2.0
    assert controller.diagnostics["target_velocity_mps"] == 0
    assert controller._s is None
    json.dumps(controller.diagnostics, allow_nan=False)


def test_commissioning_gate_and_shared_speed(profile, tmp_path):
    profile["planning"]["braking_calibrated"] = False
    assert PlanningConfig.from_config(profile).speed_limit == 2.5
    profile["planning"]["braking_calibrated"] = True
    assert PlanningConfig.from_config(profile).speed_limit == 5.0
    profile["planning"]["max_velocity_mps"] = 10
    with pytest.raises(ValueError, match="shared"):
        PlanningConfig.from_config(profile)
    (tmp_path / "driving.yaml").write_text("speed_mps: 4.0\nmax_throttle: 0.2\n")
    (tmp_path / "avlite.yaml").write_text(
        'shared_settings: driving.yaml\nc30_control:\n  c32_ego_max_velocity: ${speed_mps}\n'
        'planning:\n  max_velocity_mps: ${speed_mps}\n')
    config = load_config(tmp_path / "avlite.yaml")
    assert config["planning"]["max_velocity_mps"] == 4.0
    assert config["c30_control"]["c32_ego_max_velocity"] == 4.0


def test_saved_artifact_contains_actual_map_and_settings(prepared, tmp_path):
    filename = tmp_path / "plan.json"
    prepared.save(filename)
    data = json.loads(filename.read_text())
    assert data["map"]["LeftBound"]
    assert len(data["map_sha256"]) == 64
    assert data["resolved_config"]["c30_control"]["c32_ego_max_velocity"] == 5.0
    assert len(data["distance_m"]) == len(data["velocity"])
    from avlite_autodrive.plot_recording import plot_planned_report
    row = {"elapsed_s": 0, "x": 3, "y": 0, "speed": 1, "path_progress_m": 0,
           "path_deviation_m": 0, "target_velocity_mps": 2, "speed_limit_reason": 1}
    plot_planned_report([row], copy.deepcopy(data), tmp_path / "report.png")
    assert (tmp_path / "report.png").stat().st_size > 1000


def test_raw_obstacle_inside_track_rejects_global_plan(profile, tmp_path):
    data = circle_map()
    # Block the entire corridor with observed points, keeping its walls valid.
    data["observed_hits"] = [[x, 0.0] for x in np.arange(2.3, 3.71, 0.02)]
    path = tmp_path / "blocked.json"
    path.write_text(json.dumps(data))
    profile["planning"]["map_path"] = str(path)
    with pytest.raises(ValueError, match="LiDAR obstacle"):
        prepare_plan(profile, tmp_path / "avlite.yaml")


def test_missing_hit_classification_fails_closed(prepared):
    controller = AutoDRIVEPlannedController(prepared)
    frame = SensorFrame(lidar=clear_scan().lidar)
    cmd = controller.control(state_at(prepared, 0), prepared.global_plan, sensors=frame)
    assert cmd.acceleration == -2.0
    assert controller.diagnostics["speed_limit_reason"] == 4
