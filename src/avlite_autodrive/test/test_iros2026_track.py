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
