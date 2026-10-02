"""Asset coordinates, physical walls and the real concave-track planning regression."""

import copy
import json
from pathlib import Path

import numpy as np
import pytest

from avlite_autodrive.configuration import load_config
from avlite_autodrive.race_map import validate_map
from avlite_autodrive.race_planning import PlanningConfig, prepare_plan, reference_seed
from avlite_autodrive.sketch_track import (
    build_sketch_map, road_polygons, unity_xz_to_world, validate_physical_clearance,
)


ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def metadata():
    return json.loads((ROOT / "assets/tracks/sketch_track/track_metadata.json").read_text())


@pytest.fixture(scope="module")
def sketch(metadata):
    return build_sketch_map(metadata)


def test_unity_coordinates_match_gps_axes():
    np.testing.assert_allclose(
        unity_xz_to_world([[1, 2], [3, 4], [-1, -3]]), [[2, -1], [4, -3], [-3, 1]])


@pytest.mark.parametrize("change", [
    lambda m: m.update(units="cm"),
    lambda m: m.update(up_axis="Z"),
    lambda m: m.update(barrier_diameter_m=float("nan")),
    lambda m: m.update(barrier_diameter_m=-1),
    lambda m: m.update(barrier_diameter_m=100),
    lambda m: m["track_boundaries"].update(inner_xz=m["track_boundaries"]["outer_xz"]),
])
def test_invalid_asset_geometry_rejected(metadata, change):
    modified = copy.deepcopy(metadata)
    change(modified)
    with pytest.raises(ValueError):
        build_sketch_map(modified)


def test_map_excludes_barrier_volume_and_preserves_scale(metadata, sketch):
    left, right, corridor = validate_map(sketch)
    _, _, physical = road_polygons(metadata)
    assert physical.buffer(1e-7).covers(corridor)
    assert len(left) == len(right) > 800
    assert 29 < np.ptp(right[:, 0]) < 30
    assert 16 < np.ptp(right[:, 1]) < 17
    assert sketch["barrier_radius_m"] == pytest.approx(0.0762)


@pytest.mark.parametrize("failure", ["reversed", "outside", "incomplete"])
def test_bad_reference_route_rejected(sketch, failure):
    data = copy.deepcopy(sketch)
    if failure == "reversed":
        data["ReferencePath"].reverse()
    elif failure == "outside":
        data["ReferencePath"][0] = [100, 100]
    else:
        data["ReferencePath"] = data["ReferencePath"][:30]
    with pytest.raises(ValueError, match="ReferencePath"):
        reference_seed(data, *validate_map(data))


@pytest.mark.parametrize("value", [0, -1, True, float("inf"), "0.25"])
def test_invalid_step_limit_rejected(value):
    config = load_config(ROOT / "config/avlite.sketch.yaml")
    config["planning"]["optimization_step_limit_m"] = value
    with pytest.raises(ValueError, match="optimization_step_limit_m"):
        PlanningConfig.from_config(config)


def test_real_track_plan_clearance_and_periodic_speed_limits(metadata, sketch, tmp_path):
    config_path = ROOT / "config/avlite.sketch.yaml"
    config = load_config(config_path)
    filename = tmp_path / "sketch.json"
    filename.write_text(json.dumps(sketch))
    config["planning"]["map_path"] = str(filename)
    prepared = prepare_plan(config, config_path)
    validate_physical_clearance(prepared, metadata)
    assert 45 < prepared.path.length < 55
    assert max(prepared.global_plan.velocity) <= 2.5
    v = np.asarray(prepared.global_plan.velocity)
    dv2 = np.roll(v, -1)**2 - v**2
    assert np.all(dv2 <= 2 * prepared.settings.acceleration_mps2 * prepared.path.ds + 1e-5)
    assert np.all(-dv2 <= 2 * prepared.settings.braking_deceleration_mps2 * prepared.path.ds + 1e-5)
    # Replanning must reset the one-use seed flag, rather than silently revert
    # to the boundary midpoint that produces a folded route on this asset.
    repeated = prepared.planner.plan()
    np.testing.assert_allclose(repeated.path, prepared.path.points, atol=1e-10)


def test_practice_profile_keeps_optional_features_disabled():
    profile = load_config(ROOT / "config/avlite.yaml")
    assert PlanningConfig.from_config(profile).optimization_step_limit_m is None
    data = json.loads((ROOT / "config/maps/practice.json").read_text())
    assert reference_seed(data, *validate_map(data)) is None
