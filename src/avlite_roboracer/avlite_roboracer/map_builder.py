"""Hardware RaceMap from a finalized SLAM map and the corrected driven route (no ROS).

Unlike avlite_autodrive.race_map.build_map, this needs no simulator telemetry or finish
counters: the route comes from SLAM Toolbox's optimized pose graph and the walls from the
saved occupancy grid, both in the map frame. Every rejection names where it happened.

As a command it turns a SLAM Toolbox mapping replay into a simulator race map:

    python -m avlite_roboracer.map_builder REPLAY_DIR --profile config/avlite.yaml \
        --name practice-slam [--output-dir config/maps]

REPLAY_DIR is a `roboracer_record replay --mode mapping` output (map.yaml, map.pgm and
its bag). Run `roboracer_record export` on it first to read the route from
records.jsonl without ROS; otherwise the bag is read directly. It writes NAME.json,
NAME.plan.json and NAME.png, like `avlite.ps1 map`, and refuses existing names.
"""

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import gaussian_filter1d, median_filter
import yaml

from avlite_autodrive.race_map import ClosedPath, validate_map, xy_array

from .occupancy import OccupancyGrid


START_AREA_M = 2.0  # within this of the start counts as being back at the start


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
    # Wall-offset smoothing (in route samples) tried in order until both boundary rings are
    # simple. Map drift between laps leaves doubled wall segments whose offset lines cross
    # themselves at the first value; a smoother offset removes the loops.
    smoothing_sigmas: tuple = (1.5, 3.0, 6.0, 10.0)


def lap_route(nodes, start_xy=None):
    """Order pose-graph node positions and trim them to one lap.

    nodes are [id, x, y] in increasing id order (time order). The lap starts at the first
    node and ends at the closest node of the FIRST return to the start after leaving it, so
    a multi-lap mapping run (mapping.laps) still gives exactly one lap. Raises MapRejected
    if the route never leaves the start area; a route that never comes back is rejected
    later, as one that does not close.
    """
    points = xy_array([[n[1], n[2]] for n in sorted(nodes)], "pose-graph route", minimum=10)
    start = points[0] if start_xy is None else np.asarray(start_xy, dtype=float)
    distance = np.linalg.norm(points - start, axis=1)
    far = np.flatnonzero(distance > START_AREA_M)
    if not len(far):
        raise MapRejected("Route never left the start area")
    tail = np.arange(far[0], len(points))
    returned = tail[distance[tail] <= START_AREA_M]
    if not len(returned):
        end = tail[np.argmin(distance[tail])]  # never came back: reported as not closing
    else:
        first = returned[0]  # stay within this first pass; later laps would double the route
        left = np.flatnonzero(distance[first:] > START_AREA_M)
        passage = np.arange(first, first + (left[0] if len(left) else len(points) - first))
        end = passage[np.argmin(distance[passage])]
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
        x, y = route.points[worst]
        raise MapRejected(f"Ambiguous {side} boundary at s={route.s[worst]:.1f} m, map x="
                          f"{x:.2f} y={y:.2f} (wall distance changes by {jumps[worst]:.2f} m, "
                          f"limit {settings.max_width_jump_m} m)")
    return filtered


def _rings_are_simple(bounds):
    from shapely.geometry import Polygon
    return all(Polygon(b).is_valid and Polygon(b).area > 0 for b in bounds)


def offset_boundaries(points, normals, widths, settings):
    """Left and right boundary rings and the smoothing used, from wall offsets along normals.

    Try each smoothing in settings.smoothing_sigmas and keep the lightest one whose rings are
    both simple (non-crossing). Raise MapRejected when even the smoothest still crosses.
    """
    for sigma in settings.smoothing_sigmas:
        bounds = [gaussian_filter1d(points + sign * widths[side][:, None] * normals, sigma,
                                    axis=0, mode="wrap")
                  for sign, side in ((1, "left"), (-1, "right"))]
        if _rings_are_simple(bounds):
            return bounds, sigma
    raise MapRejected("Track boundaries cross themselves even after smoothing the wall offsets "
                      f"up to sigma {settings.smoothing_sigmas[-1]}; remap with a cleaner lap")


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
    widths = {}
    for sign, side in ((1, "left"), (-1, "right")):
        raw = _widths(grid, route, normals, sign, settings)
        widths[side] = _check_ambiguity(_fill_gaps(raw, route, side, settings), route, side,
                                        settings)
    bounds, sigma = offset_boundaries(route.points, normals, widths, settings)
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
        "boundary_smoothing_sigma": sigma,
    }
    try:
        validate_map(result, vehicle_radius_m, tracking_allowance_m, frame_id=grid.frame_id)
    except ValueError as exc:
        raise MapRejected(str(exc)) from exc
    return result


# ------------------------------------------------------------- replay command
GRAPH_TOPIC = "/slam_toolbox/graph_visualization"


def _graph_from_records(path):
    """Return the last non-empty pose-graph record in an exported records.jsonl."""
    from .records import read_records
    graphs = [r for r in read_records(path) if r["kind"] == "graph" and r.get("nodes")]
    if not graphs:
        raise MapRejected(f"{path}: no pose-graph records; was the replay in mapping mode?")
    return graphs[-1]


def _graph_from_bag(bag, topic):
    """Read the last pose-graph message on topic from a replay's rosbag2 (needs ROS)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from visualization_msgs.msg import MarkerArray

    from .records import graph_record

    meta = yaml.safe_load((Path(bag) / "metadata.yaml").read_text(encoding="utf-8"))
    storage = meta["rosbag2_bagfile_information"]["storage_identifier"]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    last = None
    while reader.has_next():
        _, data, _ = reader.read_next()
        record = graph_record(deserialize_message(data, MarkerArray))
        if record["nodes"]:
            last = record
    if last is None:
        raise MapRejected(f"{bag}: no {topic} messages with pose-graph nodes")
    return last


def load_replay_graph(replay_dir, topic=GRAPH_TOPIC):
    """Return (nodes, frame, source) for the final optimized pose graph of a replay.

    The last graph message holds the positions corrected by loop closure. Prefer the
    exported records.jsonl; fall back to reading the bag directly.
    """
    replay_dir = Path(replay_dir)
    records = replay_dir / "records.jsonl"
    if records.exists():
        graph, source = _graph_from_records(records), "records.jsonl"
    elif (replay_dir / "bag").is_dir():
        graph, source = _graph_from_bag(replay_dir / "bag", topic), f"bag:{topic}"
    else:
        raise MapRejected(f"{replay_dir}: neither records.jsonl nor bag/ found")
    return graph["nodes"], graph.get("frame"), source


def _outputs(output_dir, name):
    """Return the .json/.plan.json/.png paths, refusing names that already exist."""
    if not name or Path(name).name != name or name.endswith(".json"):
        raise ValueError(f"--name must be a bare map name, not {name!r}")
    base = Path(output_dir) / name
    paths = [base.parent / (base.name + suffix) for suffix in (".json", ".plan.json", ".png")]
    existing = [str(p) for p in paths if p.exists()]
    if existing:
        raise FileExistsError(f"Refusing to overwrite {', '.join(existing)}")
    return paths


def build_replay_map(replay_dir, config_path, name, output_dir="config/maps",
                     topic=GRAPH_TOPIC, plot=True):
    """Build, validate and write a race map and plan from a SLAM mapping replay.

    The AVLite config supplies the vehicle radius and tracking allowance for the boundary
    checks and the limits for racing-line validation (prepare_plan), the same two checks
    Jetson finalization runs. The map keeps SLAM's frame (normally "map"), so racing on it
    needs planning.frame_id set to that frame. Return the written paths.
    """
    from avlite_autodrive.configuration import load_config
    from avlite_autodrive.race_planning import prepare_plan

    replay_dir = Path(replay_dir)
    map_path, plan_path, png_path = _outputs(output_dir, name)
    map_yaml = replay_dir / "map.yaml"
    if not map_yaml.exists():
        raise MapRejected(f"{map_yaml} not found; was the mapping replay's map saved?")
    nodes, frame, source = load_replay_graph(replay_dir, topic)
    grid = OccupancyGrid.load(map_yaml, frame or "map")
    config = load_config(config_path)
    planning = config.setdefault("planning", {})
    racemap = build_hardware_map(grid, nodes, planning.get("vehicle_radius_m", 0.24),
                                 planning.get("tracking_allowance_m", 0.05))
    image = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))["image"]
    racemap.update({
        "ground_truth": False, "source_replay": str(replay_dir),
        "source_pose_graph": source, "source_pose_graph_nodes": len(nodes),
        "source_map_sha256": hashlib.sha256((replay_dir / image).read_bytes()).hexdigest(),
    })
    planning.update(map_path=str(map_path.resolve()), frame_id=grid.frame_id)
    map_path.parent.mkdir(parents=True, exist_ok=True)
    map_path.write_text(json.dumps(racemap, indent=2, allow_nan=False) + "\n")
    try:
        prepared = prepare_plan(config, config_path)
    except Exception:
        map_path.unlink()  # never leave a map whose racing line failed validation
        raise
    prepared.save(plan_path)
    written = [map_path, plan_path]
    if plot:
        try:
            from avlite_autodrive.race_map import plot_map
            plot_map(racemap, png_path, prepared.artifact)
            written.append(png_path)
        except ImportError as exc:
            print(f"Skipping preview: {exc}", file=sys.stderr)
    return written


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("replay", type=Path, help="replay --mode mapping output directory")
    parser.add_argument("--profile", type=Path, required=True,
                        help="AVLite config YAML (vehicle envelope and planning limits)")
    parser.add_argument("--name", required=True, help="map name, e.g. practice-slam")
    parser.add_argument("--output-dir", type=Path, default=Path("config/maps"))
    parser.add_argument("--graph-topic", default=GRAPH_TOPIC)
    parser.add_argument("--no-plot", action="store_true", help="skip the PNG preview")
    args = parser.parse_args(argv)
    try:
        written = build_replay_map(args.replay, args.profile, args.name, args.output_dir,
                                   args.graph_topic, plot=not args.no_plot)
    except (MapRejected, ValueError, OSError) as exc:
        print(f"Map rejected: {exc}", file=sys.stderr)
        return 1
    for path in written:
        print(f"Wrote {path}")
    print("SLAM-built map (not simulator ground truth): race on it with planning.map_path "
          "set to the .json and planning.frame_id: map.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
