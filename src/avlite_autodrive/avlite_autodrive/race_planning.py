"""Small-car adapter around the pinned AVLite GlobalRacePlanner."""

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import LineString, Polygon
from avlite.c10_perception.c11_perception_model import RaceMap
from avlite.c20_planning.c25_global_race_planners import GlobalRacePlanner

from .race_map import ClosedPath, footprint, validate_map


@dataclass(frozen=True)
class PlanningConfig:
    map_path: str = "maps/practice.json"
    sample_spacing_m: float = 0.1
    max_velocity_mps: float = 2.5
    acceleration_mps2: float = 1.0
    lateral_acceleration_mps2: float = 3.0
    braking_deceleration_mps2: float = 1.5
    braking_calibrated: bool = False
    commissioning_speed_mps: float = 2.5
    reaction_time_s: float = 0.25
    clearance_margin_m: float = 0.15
    vehicle_radius_m: float = 0.24
    tracking_allowance_m: float = 0.05
    wheelbase_m: float = 0.324
    curvature_weight: float = 0.75
    optimization_iterations: int = 3
    optimization_step_limit_m: float | None = None
    frame_id: str = "world"

    @classmethod
    def from_config(cls, config):
        """Read the planning section and check it against the resolved controller settings.

        Reject invalid numbers and mismatched speed, acceleration or wheelbase limits before
        planning. Small-car sampling and uncalibrated commissioning limits are checked here
        too.
        """
        values = dict(config.get("planning", {}))
        control = config["c30_control"]
        values.setdefault("max_velocity_mps", control["c32_ego_max_velocity"])
        settings = cls(**values)
        for key, value in asdict(settings).items():
            if key in ("map_path", "braking_calibrated", "frame_id"):
                continue
            if key == "optimization_step_limit_m" and value is None:
                continue
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"planning.{key} must be finite and positive")
        if not isinstance(settings.map_path, str) or not settings.map_path.strip():
            raise ValueError("planning.map_path is required")
        if not isinstance(settings.frame_id, str) or not settings.frame_id.strip():
            raise ValueError("planning.frame_id must be a frame name")
        if not isinstance(settings.braking_calibrated, bool):
            raise ValueError("planning.braking_calibrated must be true or false")
        if settings.curvature_weight > 1 or not isinstance(settings.optimization_iterations, int):
            raise ValueError("Invalid optimization weight or iteration count")
        if settings.sample_spacing_m > 0.1:
            raise ValueError("Small-car plan spacing must be at most 0.1 m")
        if settings.commissioning_speed_mps > 2.5:
            raise ValueError("Uncalibrated commissioning speed cannot exceed 2.5 m/s")
        if settings.max_velocity_mps != control["c32_ego_max_velocity"]:
            raise ValueError("Planner speed ceiling must match the shared control speed")
        if (settings.acceleration_mps2 > control["c32_ego_max_acceleration"]
                or settings.braking_deceleration_mps2 > -control["c32_ego_min_acceleration"]):
            raise ValueError("Planner acceleration/braking exceeds control limits")
        if settings.wheelbase_m != control["c32_ego_distance_front_axle"]:
            raise ValueError("Planning and control wheelbase must match")
        return settings

    @property
    def speed_limit(self):
        """Return the effective ceiling in m/s after applying any commissioning cap.

        A larger shared speed setting cannot bypass the lower cap while braking remains
        uncalibrated.
        """
        return (self.max_velocity_mps if self.braking_calibrated else
                min(self.max_velocity_mps, self.commissioning_speed_mps))

    @property
    def envelope_radius(self):
        """Add the tracking allowance to the physical vehicle radius for clearance checks."""
        return self.vehicle_radius_m + self.tracking_allowance_m


class AutoDRIVERacePlanner(GlobalRacePlanner):
    """Preserve upstream optimization; change spacing for the RoboRacer scale."""

    RESAMPLE_STEP = 0.1

    def __init__(self, race_map, settings, reference_path=None):
        """Configure AVLite's race optimizer for this small vehicle and track.

        Pass the effective speed and acceleration limits upstream, use finer waypoint
        spacing, and leave room for the axle sweep. The optional reference route helps
        initialize concave tracks.
        """
        self.RESAMPLE_STEP = settings.sample_spacing_m
        self.reference_path = reference_path
        self.step_limit = settings.optimization_step_limit_m
        self._seed_pending = False
        # Give the optimizer some front-axle sweep allowance. Full capsule
        # containment is independently validated; this inset is not a guarantee.
        margin = settings.envelope_radius + settings.wheelbase_m / 4
        super().__init__(
            map=race_map, max_velocity=settings.speed_limit,
            max_lateral_accel=settings.lateral_acceleration_mps2,
            max_longitudinal_accel=settings.acceleration_mps2,
            max_braking_decel=settings.braking_deceleration_mps2,
            curvature_weight=settings.curvature_weight,
            optimization_iterations=settings.optimization_iterations, margin=margin,
        )

    def plan(self, *args, **kwargs):
        """Run the upstream planner with a fresh optional seed for this planning call.

        Clear the seed flag even if optimization fails, so a later call cannot reuse partial
        state.
        """
        self._seed_pending = self.reference_path is not None
        try:
            return super().plan(*args, **kwargs)
        finally:
            self._seed_pending = False

    def _resample(self, points, step, closed):
        # The pinned upstream planner first resamples the boundary midpoint,
        # then each optimization result. Replace only that initial reference.
        """Substitute the supplied seed only for the optimizer's first reference path.

        All later resampling uses the newly optimized path and the upstream interpolation
        algorithm.
        """
        if self._seed_pending:
            points = self.reference_path
            self._seed_pending = False
        return super()._resample(points, step, closed)

    def _solve_offsets(self, ref, normals, lb, ub, weight, closed):
        """Optionally limit each optimization step before calling the upstream solver.

        Intersect the allowed lateral shifts with the track boundaries. Small steps reduce
        folded paths in concave sections; an empty allowed interval means the reference
        needs revision.
        """
        if self.step_limit is not None:
            # Large lateral steps can fold normals in a wide, concave corridor.
            # Intersect the trust region with the physical boundary constraints.
            lb = np.maximum(lb, -self.step_limit)
            ub = np.minimum(ub, self.step_limit)
            if np.any(lb >= ub):
                raise ValueError("Reference route leaves the optimizer step region; revise the map")
        return super()._solve_offsets(ref, normals, lb, ub, weight, closed)


def reference_seed(data, left, right, corridor):
    """Validate an optional initial route and return its x/y points.

    The route must make a complete loop around the inner island, stay inside the corridor,
    and follow the boundary direction. Return None when the map provides no custom seed.
    """
    if "ReferencePath" not in data:
        return None
    seed = ClosedPath(data["ReferencePath"])
    ring = Polygon(seed.points)
    closed_line = LineString(np.vstack([seed.points, seed.points[0]]))
    inner = min((Polygon(left), Polygon(right)), key=lambda p: p.area)
    if (not ring.is_valid or not ring.contains(inner)
            or not corridor.buffer(1e-8).covers(closed_line)
            or np.max(seed.ds) > 0.5):
        raise ValueError("ReferencePath must be a complete, simple route inside the corridor")
    midpoint_ring = Polygon((left + right) / 2)
    if ring.exterior.is_ccw != midpoint_ring.exterior.is_ccw:
        raise ValueError("ReferencePath direction disagrees with the map boundaries")
    return seed.points


@dataclass
class PreparedPlan:
    planner: AutoDRIVERacePlanner
    global_plan: object
    path: ClosedPath
    corridor: object
    settings: PlanningConfig
    artifact: dict

    def save(self, filename):
        """Write the validated plan, embedded map and resolved settings to a JSON artifact."""
        Path(filename).write_text(json.dumps(self.artifact, indent=2, allow_nan=False) + "\n")


def prepare_plan(config, config_path):
    """Generate a global racing line and reject plans the configured car cannot follow.

    Resolve the map relative to the YAML file, validate its geometry, then check speed,
    steering, lateral acceleration and acceleration/braking across every segment, including
    the lap boundary. Sweep the vehicle footprint between waypoints to check walls and
    recorded hits.

    Return a PreparedPlan containing the live planner objects and a reproducible JSON
    artifact. This work runs before driving, outside the 20 Hz control loop.
    """
    settings = PlanningConfig.from_config(config)
    source = Path(settings.map_path)
    if not source.is_absolute():
        source = Path(config_path).resolve().parent / source
    data = json.loads(source.read_text(encoding="utf-8"))
    left, right, corridor = validate_map(
        data, settings.vehicle_radius_m, settings.tracking_allowance_m, settings.frame_id)
    # Close both polylines explicitly: upstream nearest-boundary searches use LineString.
    race_map = RaceMap(source_path=str(source),
                       left_bound=np.vstack([left, left[0]]),
                       right_bound=np.vstack([right, right[0]]),
                       _reference_point=tuple(data["ReferencePoint"]))
    planner = AutoDRIVERacePlanner(race_map, settings, reference_seed(data, left, right, corridor))
    plan = planner.plan()
    path = ClosedPath(plan.path)
    path_ring = Polygon(path.points)
    inner = min((Polygon(left), Polygon(right)), key=lambda p: p.area)
    if not path_ring.is_valid or not path_ring.contains(inner):
        raise ValueError("Planned route must be simple and complete a lap around the island")
    velocity = np.asarray(plan.velocity)
    curvature = planner._curvature(path.points, True)
    if (velocity.shape != (len(path.points),) or not np.isfinite(velocity).all()
            or np.any(velocity < 0) or np.max(velocity) > settings.speed_limit + 1e-6):
        raise ValueError("Planner returned an invalid velocity profile")
    control = config["c30_control"]
    max_curvature = math.tan(min(abs(control["c32_ego_min_steering"]),
                                 control["c32_ego_max_steering"])) / settings.wheelbase_m
    if np.max(curvature) > max_curvature + 1e-6:
        raise ValueError(f"Planned curvature {np.max(curvature):.3f} exceeds vehicle limit "
                         f"{max_curvature:.3f}; revise the map")
    if np.any(velocity**2 * curvature > settings.lateral_acceleration_mps2 + 1e-5):
        raise ValueError("Planner violated lateral acceleration limits")
    # v_next^2 - v_now^2 = 2*a*distance. Rolling the array also checks braking
    # from the last waypoint back to the first, where the next lap begins.
    dv2 = np.roll(velocity, -1)**2 - velocity**2
    if (np.any(dv2 > 2 * settings.acceleration_mps2 * path.ds + 1e-5)
            or np.any(-dv2 > 2 * settings.braking_deceleration_mps2 * path.ds + 1e-5)):
        raise ValueError("Planner violated acceleration/braking limits at a path segment")
    # Check intermediate poses too; valid waypoints alone can cut through a wall.
    hits = np.asarray(data.get("observed_hits", []), dtype=float)
    hit_tree = cKDTree(hits) if len(hits) else None
    corridor_tolerant = corridor.buffer(1e-7)
    for s in np.arange(0, path.length, 0.025):
        xy = path.at(s)
        direction = path.at(s + 0.025) - xy
        body = footprint(xy, np.arctan2(direction[1], direction[0]),
                         settings.wheelbase_m, settings.envelope_radius)
        if not corridor_tolerant.covers(body):
            raise ValueError(f"Planned vehicle envelope leaves mapped corridor at s={s:.2f} m")
        # Check raw hits at every swept pose too: projecting an obstacle only to
        # its nearest rear-axle waypoint can miss contact with the front overhang.
        if hit_tree is not None:
            nearby = hit_tree.query_ball_point(xy, settings.envelope_radius + settings.wheelbase_m)
            offset = hits[nearby] - xy
            unit = direction / np.linalg.norm(direction)
            along = np.clip(offset @ unit, 0, settings.wheelbase_m)
            distance = np.linalg.norm(offset - along[:, None] * unit, axis=1)
            if np.any(distance < settings.envelope_radius):
                raise ValueError("Planned envelope intersects a recorded LiDAR obstacle")
    canonical_map = json.dumps(data, sort_keys=True, allow_nan=False)
    artifact = {
        "version": 1, "frame_id": settings.frame_id, "units": "m", "closed": True,
        "planner": "AutoDRIVERacePlanner(GlobalRacePlanner)",
        "map_sha256": hashlib.sha256(canonical_map.encode()).hexdigest(),
        "map": data, "settings": asdict(settings), "resolved_config": config,
        "path": path.points.tolist(), "velocity": velocity.tolist(),
        "curvature": curvature.tolist(), "distance_m": path.s[:-1].tolist(),
        "length_m": path.length,
    }
    return PreparedPlan(planner, plan, path, corridor, settings, artifact)
