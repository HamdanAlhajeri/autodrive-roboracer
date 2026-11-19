"""IROS 2026 replica: spec checks, map conversion and a full commissioning plan."""

import json
from pathlib import Path

import numpy as np
import pytest

from avlite_autodrive.configuration import load_config
from avlite_autodrive.race_map import ClosedPath, validate_map
from avlite_autodrive.race_planning import prepare_plan
from avlite_autodrive.sketch_track import (
    build_sketch_map, monotone_pairing, road_polygons, validate_physical_clearance,
)


ROOT = Path(__file__).resolve().parents[3]
TRACK = ROOT / "assets/tracks/iros2026"


@pytest.fixture(scope="module")
def metadata():
    return json.loads((TRACK / "track_metadata.json").read_text())


@pytest.fixture(scope="module")
def iros(metadata):
    return build_sketch_map(metadata, spacing=metadata["map_spacing_m"])


def test_replica_records_rules_it_meets(metadata):
    checks = metadata["spec_checks"]
    assert checks["minimum_width_ge_1_m"] and checks["most_sections_ge_1_5_m"]
    assert checks["pipe_height_ge_0_30_m"]
    assert metadata["barrier_height_in_unity_m"] >= 0.30
    assert 1.5 < metadata["road_width_m"]["min"] < metadata["road_width_m"]["median"]
    # The published drawing is about 44 x 24 m at its stated 0.05 m/pixel. Keep the
    # official scale and record that it exceeds the 43 x 23 m written limit.
    assert not checks["extent_within_43_x_23_m"]
    assert 43 < metadata["extent_m"][0] < 45 and 23 < metadata["extent_m"][1] < 24.5


def test_obj_has_the_three_objects_the_unity_builder_expects():
    names = [line.split()[1] for line in (TRACK / "iros2026_track.obj").open()
             if line.startswith("o ")]
    assert sorted(names) == ["Inner_Barrier", "Outer_Barrier", "Track_Surface"]


def test_map_stays_inside_physical_road(metadata, iros):
    left, right, corridor = validate_map(iros)
    _, _, physical = road_polygons(metadata)
    assert physical.buffer(1e-7).covers(corridor)
    assert len(left) == len(right) > 3000
    assert iros["source"] == "iros2026_official_map"
    assert iros["reference_seed"] == "smoothed_midline"


def test_committed_map_matches_assets(iros):
    committed = json.loads((ROOT / "config/maps/iros2026.json").read_text())
    for key in ("LeftBound", "RightBound", "ReferencePath"):
        np.testing.assert_allclose(committed[key], iros[key], atol=1e-9)


def test_monotone_pairing_keeps_pairs_radial():
    angle = np.linspace(0, 2 * np.pi, 400, endpoint=False)
    inner = ClosedPath(np.column_stack([np.cos(angle), np.sin(angle)]))
    # Uneven outer sampling would break fixed-fraction pairing; matching must not care.
    warped = angle + 0.4 * np.sin(angle)
    outer = ClosedPath(3 * np.column_stack([np.cos(warped), np.sin(warped)]))
    left, right = monotone_pairing(inner, outer, 0.05)
    cross = np.abs(left[:, 0] * right[:, 1] - left[:, 1] * right[:, 0]) / 3
    assert np.max(cross) < 0.05


def test_commissioning_plan_clears_original_pipes(metadata, iros, tmp_path):
    config_path = ROOT / "config/avlite.iros2026.yaml"
    config = load_config(config_path)
    filename = tmp_path / "iros2026.json"
    filename.write_text(json.dumps(iros))
    config["planning"]["map_path"] = str(filename)
    prepared = prepare_plan(config, config_path)
    validate_physical_clearance(prepared, metadata)
    assert 190 < prepared.path.length < 210
    assert max(prepared.global_plan.velocity) <= 2.5


def lidar_scan(segments, pose, mount=0.2733, beams=1081, max_range=10.0):
    """Hokuyo-like 270 degree scan of the pipe surfaces from a rear-axle pose."""
    from types import SimpleNamespace
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    x, y, yaw = pose
    sx, sy = x + mount * np.cos(yaw), y + mount * np.sin(yaw)
    angles = np.radians(np.linspace(-135, 135, beams))
    d = np.c_[np.cos(yaw + angles), np.sin(yaw + angles)]
    s = segments[np.hypot(segments[:, 0] - sx, segments[:, 1] - sy) < max_range + 1]
    p, e = s[:, :2] - [sx, sy], s[:, 2:] - s[:, :2]
    den = d[:, 0, None] * e[None, :, 1] - d[:, 1, None] * e[None, :, 0]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (p[None, :, 0] * e[None, :, 1] - p[None, :, 1] * e[None, :, 0]) / den
        u = (p[None, :, 0] * d[:, 1, None] - p[None, :, 1] * d[:, 0, None]) / den
    r = np.where((t > 0) & (u >= 0) & (u <= 1), t, np.inf).min(axis=1)
    hit = r < max_range
    r = np.where(hit, r, max_range)
    cloud = np.zeros((beams, 4), dtype=np.float32)
    cloud[:, :2] = np.c_[r * np.cos(angles), r * np.sin(angles)]
    transform = np.eye(4)
    transform[:3, 3] = [mount, 0.0, 0.096]
    return SimpleNamespace(lidar=cloud, lidar_hit_mask=hit,
                           lidar_sensor=Lidar(base_to_sensor=transform))


def test_icp_profile_starts_at_the_spawn_and_tracks_the_car(metadata, iros):
    from avlite.c10_perception.c11_perception_model import EgoState, PerceptionModel
    from avlite_autodrive.avlite_localization import (
        AutoDRIVELidarLocalization, avlite_localization_config,
    )
    config = load_config(ROOT / "config/avlite.iros2026.yaml")
    report = json.loads((ROOT / "config/maps/iros2026.validation.json").read_text())
    np.testing.assert_allclose(config["localization"]["start_pose_world"],
                               report["spawn_world_pose"], atol=1e-4)
    assert config["localization"]["reference"] == "map_hits"
    hits = np.array(iros["observed_hits"])
    _, _, physical = road_polygons(metadata)
    edge = [physical.exterior, *physical.interiors]
    rings = [np.array(r.coords) for r in edge]
    segments = np.vstack([np.c_[c[:-1], c[1:]] for c in rings])
    assert len(hits) > 10000

    # Drive the first 40 m of the commissioning line at 2.5 m/s and 10 Hz with wheel
    # odometry reading 2% long (slip). ICP against the synthetic map must hold the pose.
    path = ClosedPath(np.array(report_path(config))[:, :2])
    s = np.arange(0, 40, 0.25)
    xy = path.at(s)
    heading = np.unwrap(np.arctan2(*(path.at(s + 0.05) - xy).T[::-1]))
    truth = np.c_[xy, heading]
    odom = [0.0, 0.0, 0.0]

    pm = PerceptionModel(ego_vehicle=EgoState(x=0, y=0))
    localizer = AutoDRIVELidarLocalization(
        pm, avlite_localization_config(config), lambda: 2.5,
        odometry_pose=lambda: tuple(odom), reference_points=hits)
    errors = []
    for k, pose in enumerate(truth):
        if k:
            dx, dy = (pose[:2] - truth[k - 1][:2]) * 1.02
            c, s_ = np.cos(truth[k - 1][2]), np.sin(truth[k - 1][2])
            fx, fy = c * dx + s_ * dy, -s_ * dx + c * dy
            yaw = odom[2]
            odom = [odom[0] + np.cos(yaw) * fx - np.sin(yaw) * fy,
                    odom[1] + np.sin(yaw) * fx + np.cos(yaw) * fy,
                    yaw + pose[2] - truth[k - 1][2]]
        localizer.localize(sensors=lidar_scan(segments, pose))
        errors.append(np.hypot(pm.ego_vehicle.x - pose[0], pm.ego_vehicle.y - pose[1]))
        assert localizer.alignment > 0.7
    assert max(errors) < 0.3 and localizer.recoveries == 0


def report_path(config):
    """The commissioning line, prepared from the profile."""
    return prepare_plan(config, ROOT / "config/avlite.iros2026.yaml").artifact["path"]
