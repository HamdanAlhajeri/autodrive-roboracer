"""Convert the supplied Unity XZ track metadata into an offline AVLite map."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import minimize_scalar
from shapely.geometry import Polygon
import yaml

from .configuration import load_config
from .race_map import ClosedPath, footprint, plot_map, validate_map, xy_array
from .race_planning import prepare_plan


def unity_xz_to_world(points):
    """Convert ground-plane Unity (X, Z) pairs into ROS world (x, y) metres.

    AutoDRIVE uses x = Unity Z and y = -Unity X. Keeping this convention matches the
    simulator's GPS/odometry instead of rotating the map later.
    """
    points = xy_array(points, "Unity boundary")
    return np.column_stack((points[:, 1], -points[:, 0]))


def ccw_points(polygon):
    """Return polygon vertices in counterclockwise order without a duplicate closing point."""
    points = np.asarray(polygon.exterior.coords)[:-1]
    if not polygon.exterior.is_ccw:
        points = points[::-1]
    return points


def road_polygons(metadata, extra_inset=0.0):
    """Convert track metadata into the outer edge, inner island and drivable road polygon.

    Move both boundaries away from the barrier centrelines by the tube radius plus any extra
    inset. Reject geometry that no longer leaves a continuous road.
    """
    if metadata.get("units") != "metres" or metadata.get("up_axis") != "Y":
        raise ValueError("Track metadata must use metres with Unity Y up")
    diameter = metadata.get("barrier_diameter_m")
    if (isinstance(diameter, bool) or not isinstance(diameter, (int, float))
            or not np.isfinite(diameter) or diameter <= 0):
        raise ValueError("barrier_diameter_m must be finite and positive")
    radius = diameter / 2 + extra_inset
    boundaries = metadata["track_boundaries"]
    outer = Polygon(unity_xz_to_world(boundaries["outer_xz"]))
    inner = Polygon(unity_xz_to_world(boundaries["inner_xz"]))
    if not outer.is_valid or not inner.is_valid or not outer.contains(inner):
        raise ValueError("Source boundaries must be simple and nested")
    outer, inner = outer.buffer(-radius), inner.buffer(radius)
    if (outer.geom_type != "Polygon" or inner.geom_type != "Polygon"
            or outer.is_empty or inner.is_empty or not outer.contains(inner)
            or len(outer.interiors) or len(inner.interiors)):
        raise ValueError("Barriers leave no continuous closed road")
    return outer, inner, Polygon(outer.exterior.coords, [inner.exterior.coords])


def build_sketch_map(metadata, spacing=0.1):
    """Create paired AVLite boundaries and a starting route from the supplied track geometry.

    Align samples from the unequal inner/outer loops, then form a smooth seed around the
    inner island. Validate that the exported corridor stays inside the physical road before
    returning the map dictionary.
    """
    if not np.isfinite(spacing) or not 0.02 <= spacing <= 0.1:
        raise ValueError("Map spacing must be between 0.02 and 0.1 m")
    # Include the entire tube radius, plus 1 cm for polyline approximation.
    outer, inner, _ = road_polygons(metadata, extra_inset=0.01)
    paths = []
    for polygon in (inner, outer):
        points = ccw_points(polygon)
        points = np.roll(points, -np.argmin(points[:, 0]), axis=0)
        paths.append(ClosedPath(points))
    count = int(np.ceil(max(p.length for p in paths) / spacing))
    fractions = np.arange(count) / count
    left = paths[0].at(fractions * paths[0].length)

    def cost(phase):
        """Score how well inner and outer samples line up at a proposed phase offset.

        The phase is a fraction of the outer perimeter. Lower mean squared cross-track
        distance gives a better pairing for the optimizer.
        """
        right = paths[1].at((fractions + phase) * paths[1].length)
        return float(np.mean(np.sum((left - right)**2, axis=1)))

    phase = min(np.arange(512) / 512, key=cost)
    phase = minimize_scalar(cost, bounds=(phase - 1 / 512, phase + 1 / 512),
                            method="bounded").x
    right = paths[1].at((fractions + phase) * paths[1].length)

    # The unequal perimeters make an index midpoint a poor optimizer seed at
    # the concave inner bend. Start with a smooth route 1.2 m from the island.
    seed_polygon = Polygon(left).buffer(1.2)
    if seed_polygon.geom_type != "Polygon" or len(seed_polygon.interiors):
        raise ValueError("Cannot form a single reference route around the island")
    seed = ClosedPath(ccw_points(seed_polygon))
    seed = gaussian_filter1d(seed.at(np.arange(0, seed.length, spacing)),
                             0.3 / spacing, axis=0, mode="wrap")
    data = {
        "LeftBound": left.tolist(), "RightBound": right.tolist(),
        "ReferencePoint": [0.0, 0.0], "ReferencePath": seed.tolist(),
        "frame_id": "world", "units": "m", "closed": True,
        "source": "sketch_track_metadata", "spacing_m": spacing,
        "unity_transform": "identity; world_xy = (unity_z, -unity_x)",
        "barrier_radius_m": metadata["barrier_diameter_m"] / 2,
        "boundary_approximation_allowance_m": 0.01,
        "reference_inner_clearance_m": 1.2, "unity_runtime_tested": False,
    }
    _, _, mapped = validate_map(data)
    _, _, physical = road_polygons(metadata)
    if not physical.buffer(1e-7).covers(mapped):
        raise ValueError("Sampled map includes space occupied by a physical barrier")
    return data


def validate_physical_clearance(prepared, metadata):
    """Check the planned vehicle footprint against the original barrier geometry.

    Sample every 0.025 m and raise an error at the first contact. This catches collisions
    that a simplified or resampled map could miss.
    """
    _, _, physical = road_polygons(metadata)
    physical = physical.buffer(1e-7)
    for distance in np.arange(0, prepared.path.length, 0.025):
        xy = prepared.path.at(distance)
        heading = prepared.path.at(distance + 0.01) - xy
        body = footprint(xy, np.arctan2(heading[1], heading[0]),
                         prepared.settings.wheelbase_m, prepared.settings.envelope_radius)
        if not physical.covers(body):
            raise ValueError(f"Planned vehicle touches the original barriers at {distance:.2f} m")


def main():
    """Generate the sketch map, separate commissioning profile, preview and validation report.

    Keep the active input YAML unchanged, retain references to the shared driving settings,
    and validate both the planned map and physical barrier clearance. Report a rear-axle
    spawn pose for the Unity scene builder.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile-output", type=Path, required=True)
    args = parser.parse_args()
    if args.config.resolve() == args.profile_output.resolve():
        parser.error("Profile output must differ from the active input configuration")
    if args.profile_output.resolve().parent != args.config.resolve().parent:
        parser.error("Keep the generated profile beside the input so shared settings resolve")
    metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
    data = build_sketch_map(metadata)
    data["source_metadata_sha256"] = hashlib.sha256(args.metadata.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    # Separate commissioning profile; shared speed/throttle still come from
    # driving.yaml. The uncalibrated planner caps this preview at 2.5 m/s.
    profile = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    profile["driving_mode"] = "planned"
    profile.setdefault("planning", {}).update(
        map_path=str(args.output.resolve().relative_to(args.profile_output.resolve().parent)),
        sample_spacing_m=0.1, acceleration_mps2=1.0, lateral_acceleration_mps2=3.0,
        braking_deceleration_mps2=1.5, braking_calibrated=False,
        commissioning_speed_mps=2.5, optimization_step_limit_m=0.25,
        curvature_weight=0.75, optimization_iterations=3,
    )
    profile["c30_control"].update(c32_ego_max_acceleration=1.0, c32_ego_min_acceleration=-1.5)
    args.profile_output.write_text(
        "# Sketch track commissioning. Use only with the matching custom Unity build.\n"
        "# Before driving, set driving.yaml to speed_mps: 2.5 and max_throttle: 0.2.\n"
        + yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")
    prepared = prepare_plan(load_config(args.profile_output), args.profile_output)
    validate_physical_clearance(prepared, metadata)
    prepared.save(args.output.with_suffix(".plan.json"))
    plot_map(data, args.output.with_suffix(".png"), prepared.artifact)
    xy = prepared.path.at(0)
    tangent = prepared.path.at(0.1) - xy
    report = {
        "status": "offline_validated_not_driven", "length_m": prepared.path.length,
        "min_planned_speed_mps": min(prepared.artifact["velocity"]),
        "max_planned_speed_mps": max(prepared.artifact["velocity"]),
        "map_sha256": prepared.artifact["map_sha256"],
        "rear_axle_spawn_unity_xz_m": [-float(xy[1]), float(xy[0])],
        "spawn_unity_yaw_degrees": -float(np.degrees(np.arctan2(tangent[1], tangent[0]))),
        "spawn_note": "Align the rear axle, not the vehicle root; settle wheels at Y=0.",
        "required_optimizer_step_limit_m": 0.25,
        "checks": ["simple nested map", "barrier radius and sampling allowance",
                   "complete reference and planned laps", "steering limit",
                   "lateral acceleration", "acceleration/braking including closing segment",
                   "0.324 m axle with 0.29 m envelope radius sampled every 0.025 m",
                   "original physical barrier clearance"],
    }
    args.output.with_suffix(".validation.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
