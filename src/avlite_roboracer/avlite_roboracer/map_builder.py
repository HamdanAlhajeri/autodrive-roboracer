"""Hardware RaceMap from a finalized SLAM map and the corrected driven route (no ROS).

Unlike avlite_autodrive.race_map.build_map, this needs no simulator telemetry or finish
counters: the route comes from SLAM Toolbox's optimized pose graph and the walls from the
saved occupancy grid, both in the map frame. Every rejection names where it happened.
"""

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import gaussian_filter1d, median_filter

from avlite_autodrive.race_map import ClosedPath, validate_map, xy_array


class MapRejected(ValueError):
    """The map cannot support racing; the car must remain stopped and remap."""


@dataclass(frozen=True)
class BoundarySettings:
    spacing_m: float = 0.1
    max_half_width_m: float = 2.5
    max_gap_m: float = 0.5
    max_width_jump_m: float = 0.3
    max_route_closure_m: float = 1.0
    min_route_wall_clearance_m: float = 0.1
    hit_corridor_margin_m: float = 0.2


def lap_route(nodes, start_xy=None):
    """Order pose-graph node positions and trim them to one lap.

    nodes are [id, x, y] in increasing id order (time order). The lap starts at the first
    node and ends at the last node before the route returns closest to that start after
    leaving it. Raises MapRejected if the route never comes back.
    """
    points = xy_array([[n[1], n[2]] for n in sorted(nodes)], "pose-graph route", minimum=10)
    start = points[0] if start_xy is None else np.asarray(start_xy, dtype=float)
    distance = np.linalg.norm(points - start, axis=1)
    far = np.flatnonzero(distance > 2.0)
    if not len(far):
        raise MapRejected("Route never left the start area")
    tail = np.arange(far[0], len(points))
    end = tail[np.argmin(distance[tail])]
    return points[:end + 1]


def resample_route(points, settings):
    """Close, resample and lightly smooth the route at the configured spacing."""
    points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-4]]
    closure = float(np.linalg.norm(points[-1] - points[0]))
    if closure > settings.max_route_closure_m:
        raise MapRejected(f"Corrected route does not close: ends {closure:.2f} m from its start")
    if closure < 0.02:
        points = points[:-1]
    path = ClosedPath(points)
    stations = np.arange(0, path.length, settings.spacing_m)
    smooth = gaussian_filter1d(path.at(stations), 2.0, axis=0, mode="wrap")
    return ClosedPath(smooth)


def check_route_agrees(grid, route, settings):
    """Require every route point to lie in free space away from mapped walls.

    A route that crosses walls or unknown space means the corrected poses and finalized
    map disagree (for example an unfinished loop closure).
    """
    values = grid.values(route.points)
    clearance = grid.distance_to_occupied(route.points)
    bad = np.flatnonzero((values != 0) | (clearance < settings.min_route_wall_clearance_m))
    if len(bad):
        s = route.s[bad[0]]
        raise MapRejected(f"Corrected route disagrees with the finalized map at s={s:.1f} m "
                          f"({len(bad)} of {len(route.points)} samples)")


def _widths(grid, route, normals, sign, settings):
    """Cast along one side's normals; return distances (nan where the wall is unsupported)."""
    widths = np.full(len(route.points), np.nan)
    for i, point in enumerate(route.points):
        distance, outcome = grid.cast(point, sign * normals[i], settings.max_half_width_m)
        if outcome == "occupied":
            widths[i] = distance
    return widths


def _fill_gaps(widths, route, side, settings):
    """Interpolate short unsupported stretches; reject long ones with their location."""
    valid = np.flatnonzero(np.isfinite(widths))
    if len(valid) < 3:
        raise MapRejected(f"{side} boundary is missing")
    s = route.s[:-1]
    gaps = np.diff(np.r_[s[valid], s[valid[0]] + route.length])
    worst = int(np.argmax(gaps))
    if gaps[worst] > settings.max_gap_m:
        raise MapRejected(f"Unsupported {side} boundary gap of {gaps[worst]:.2f} m "
                          f"after s={s[valid[worst]]:.1f} m")
    return np.interp(s, s[valid], widths[valid], period=route.length)


def _check_ambiguity(widths, route, side, settings):
    """Reject abrupt wall-distance changes that a median filter cannot explain.

    Such jumps indicate side openings, branches or overlapping sections where the wall
    pairing is ambiguous.
    """
    filtered = median_filter(widths, size=5, mode="wrap")
    jumps = np.abs(np.diff(np.r_[filtered, filtered[0]]))
    worst = int(np.argmax(jumps))
    if jumps[worst] > settings.max_width_jump_m:
        raise MapRejected(f"Ambiguous {side} boundary at s={route.s[worst]:.1f} m "
                          f"(wall distance changes by {jumps[worst]:.2f} m)")
    return filtered


def build_hardware_map(grid, nodes, vehicle_radius_m, tracking_allowance_m,
                       settings=None, start_xy=None):
    """Return a validated RaceMap dict in the grid's frame, or raise MapRejected.

    Steps: trim the corrected route to one lap, check it agrees with the map, find paired
    left/right walls along its normals, reject missing, ambiguous or too-narrow sections,
    then run the shared RaceMap validation with the map's declared frame.
    """
    settings = settings or BoundarySettings()
    route = resample_route(lap_route(nodes, start_xy), settings)
    check_route_agrees(grid, route, settings)
    tangents = np.roll(route.points, -1, axis=0) - np.roll(route.points, 1, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    bounds, widths = [], {}
    for sign, side in ((1, "left"), (-1, "right")):
        raw = _widths(grid, route, normals, sign, settings)
        filled = _check_ambiguity(_fill_gaps(raw, route, side, settings), route, side, settings)
        widths[side] = filled
        bounds.append(gaussian_filter1d(route.points + sign * filled[:, None] * normals,
                                        1.5, axis=0, mode="wrap"))
    total = widths["left"] + widths["right"]
    required = 2 * (vehicle_radius_m + tracking_allowance_m)
    narrow = np.flatnonzero(total <= required)
    if len(narrow):
        raise MapRejected(f"Insufficient vehicle clearance at s={route.s[narrow[0]]:.1f} m: "
                          f"width {total[narrow[0]]:.2f} m, need more than {required:.2f} m")
    hits = grid.occupied_points()
    if len(hits):
        # Keep hits near the corridor: the planner sweeps the vehicle against them.
        reach = settings.max_half_width_m + settings.hit_corridor_margin_m
        near = np.zeros(len(hits), dtype=bool)
        for point in route.points[::5]:
            near |= np.sum((hits - point) ** 2, axis=1) <= reach ** 2
        hits = hits[near]
    result = {
        "LeftBound": bounds[0].tolist(), "RightBound": bounds[1].tolist(),
        "ReferencePath": route.points.tolist(),
        "ReferencePoint": [0.0, 0.0], "frame_id": grid.frame_id, "units": "m", "closed": True,
        "source": "slam_toolbox_pose_graph_and_occupancy_grid",
        "spacing_m": settings.spacing_m, "recorded_path": route.points.tolist(),
        "observed_hits": hits.tolist(),
        "track_width_m": {"min": float(total.min()), "max": float(total.max())},
    }
    try:
        validate_map(result, vehicle_radius_m, tracking_allowance_m, frame_id=grid.frame_id)
    except ValueError as exc:
        raise MapRejected(str(exc)) from exc
    return result
