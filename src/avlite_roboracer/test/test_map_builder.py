import numpy as np
import pytest

from avlite_autodrive.race_map import validate_map
from avlite_roboracer.map_builder import MapRejected, build_hardware_map, lap_route
from avlite_roboracer.occupancy import FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from roboracer_fixtures import annulus_grid, circle_nodes


def build(grid=None, nodes=None, radius=0.15, allowance=0.05):
    return build_hardware_map(grid or annulus_grid(), nodes or circle_nodes(), radius, allowance)


def test_builds_paired_boundaries_in_the_map_frame():
    result = build()
    assert result["frame_id"] == "map"
    left = np.linalg.norm(result["LeftBound"], axis=1)
    right = np.linalg.norm(result["RightBound"], axis=1)
    # Driving CCW, the inner wall (r=3) is on the left and the outer wall (r=5) on the right.
    assert np.allclose(left, 3.05, atol=0.08)
    assert np.allclose(right, 4.95, atol=0.08)
    assert result["track_width_m"]["min"] == pytest.approx(1.9, abs=0.1)
    route = np.linalg.norm(result["ReferencePath"], axis=1)
    assert np.allclose(route, 4.0, atol=0.02)
    assert len(result["observed_hits"]) > 100


def test_shared_validation_keeps_world_default_and_accepts_declared_frame():
    result = build()
    with pytest.raises(ValueError, match="world coordinates"):
        validate_map(result, 0.15, 0.05)
    validate_map(result, 0.15, 0.05, frame_id="map")
    simulator = dict(result, frame_id="world")
    with pytest.raises(ValueError, match="map coordinates"):
        validate_map(simulator, 0.15, 0.05, frame_id="map")
    unlabeled = {k: v for k, v in result.items() if k != "frame_id"}
    with pytest.raises(ValueError):
        validate_map(unlabeled, 0.15, 0.05, frame_id="map")


def test_route_is_trimmed_to_one_lap():
    route = lap_route(circle_nodes(laps=1.3))
    assert np.linalg.norm(route[-1] - route[0]) < 0.1
    assert len(route) < 300 * 1.05


def test_route_that_never_returns_is_rejected():
    with pytest.raises(MapRejected, match="does not close"):
        build(nodes=circle_nodes(laps=0.8))


def test_missing_wall_section_is_rejected():
    def open_outer_wall(xc, yc, cells):
        angle = np.arctan2(yc, xc)
        r = np.hypot(xc, yc)
        gap = (np.abs(angle - 1.0) < 0.15) & (r > 4.5)
        cells[gap] = UNKNOWN
        cells[gap & (r < 4.9)] = FREE

    with pytest.raises(MapRejected, match="Unsupported right boundary gap"):
        build(annulus_grid(extra=open_outer_wall))


def test_short_wall_dropout_is_interpolated():
    def pinhole(xc, yc, cells):
        angle = np.arctan2(yc, xc)
        r = np.hypot(xc, yc)
        cells[(np.abs(angle - 1.0) < 0.03) & (np.abs(r - 5.0) < 0.1)] = FREE

    result = build(annulus_grid(extra=pinhole))
    assert np.allclose(np.linalg.norm(result["RightBound"], axis=1), 4.95, atol=0.1)


def test_side_bay_is_ambiguous():
    def bay(xc, yc, cells):
        angle = np.arctan2(yc, xc)
        r = np.hypot(xc, yc)
        sector = np.abs(angle - 1.0) < 0.2
        cells[sector & (r > 4.9) & (r < 6.0)] = FREE
        cells[sector & (np.abs(r - 6.0) <= 0.05)] = OCCUPIED
        edge = (np.abs(np.abs(angle - 1.0) - 0.2) < 0.012) & (r >= 4.95) & (r <= 6.0)
        cells[edge] = OCCUPIED

    with pytest.raises(MapRejected, match="Ambiguous right boundary"):
        build(annulus_grid(outer=5.0, extra=bay))


def test_insufficient_vehicle_clearance():
    with pytest.raises(MapRejected, match="Insufficient vehicle clearance"):
        build(radius=0.9, allowance=0.1)


def test_route_disagreeing_with_map_is_rejected():
    shifted = [[i, x * 1.3, y * 1.3] for i, x, y in circle_nodes()]
    with pytest.raises(MapRejected, match="disagrees with the finalized map"):
        build(nodes=shifted)


def test_occupancy_roundtrip_and_ray_casting(tmp_path):
    grid = annulus_grid()
    grid.save(tmp_path / "map.yaml")
    loaded = OccupancyGrid.load(tmp_path / "map.yaml")
    assert np.array_equal(loaded.cells, grid.cells)
    assert loaded.origin == pytest.approx(grid.origin)
    distance, outcome = loaded.cast((4.0, 0.0), (1.0, 0.0), 2.5)
    assert outcome == "occupied" and distance == pytest.approx(0.95, abs=0.05)
    assert loaded.cast((4.0, 0.0), (0.0, 1.0), 0.5) == (0.5, "max_range")
    assert loaded.cast((0.0, 0.0), (1.0, 0.0), 2.0)[1] == "unknown"
    assert loaded.cast((6.4, 0.0), (1.0, 0.0), 2.0)[1] in ("unknown", "outside")
    assert loaded.distance_to_occupied([[4.0, 0.0]])[0] == pytest.approx(0.95, abs=0.06)
    assert loaded.values([[100.0, 0.0]])[0] == UNKNOWN
