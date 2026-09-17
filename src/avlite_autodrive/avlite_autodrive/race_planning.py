"""Small-car adapter around the pinned AVLite GlobalRacePlanner."""

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
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

    @classmethod
    def from_config(cls, config):
        values = dict(config.get("planning", {}))
        control = config["c30_control"]
        values.setdefault("max_velocity_mps", control["c32_ego_max_velocity"])
        settings = cls(**values)
        for key, value in asdict(settings).items():
            if key in ("map_path", "braking_calibrated"):
                continue
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"planning.{key} must be finite and positive")
        if not isinstance(settings.map_path, str) or not settings.map_path.strip():
            raise ValueError("planning.map_path is required")
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
        return (self.max_velocity_mps if self.braking_calibrated else
                min(self.max_velocity_mps, self.commissioning_speed_mps))

    @property
    def envelope_radius(self):
        return self.vehicle_radius_m + self.tracking_allowance_m


class AutoDRIVERacePlanner(GlobalRacePlanner):
    """Preserve upstream optimization; change spacing for the RoboRacer scale."""

    RESAMPLE_STEP = 0.1

    def __init__(self, race_map, settings):
        self.RESAMPLE_STEP = settings.sample_spacing_m
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


@dataclass
class PreparedPlan:
    planner: AutoDRIVERacePlanner
    global_plan: object
    path: ClosedPath
    corridor: object
    settings: PlanningConfig
    artifact: dict

    def save(self, filename):
        Path(filename).write_text(json.dumps(self.artifact, indent=2, allow_nan=False) + "\n")


def prepare_plan(config, config_path):
    settings = PlanningConfig.from_config(config)
    source = Path(settings.map_path)
    if not source.is_absolute():
        source = Path(config_path).resolve().parent / source
    data = json.loads(source.read_text(encoding="utf-8"))
    left, right, corridor = validate_map(
        data, settings.vehicle_radius_m, settings.tracking_allowance_m)
    # Close both polylines explicitly: upstream nearest-boundary searches use LineString.
    race_map = RaceMap(source_path=str(source),
                       left_bound=np.vstack([left, left[0]]),
                       right_bound=np.vstack([right, right[0]]),
                       _reference_point=tuple(data["ReferencePoint"]))
    planner = AutoDRIVERacePlanner(race_map, settings)
    plan = planner.plan()
    path = ClosedPath(plan.path)
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
        "version": 1, "frame_id": "world", "units": "m", "closed": True,
        "planner": "AutoDRIVERacePlanner(GlobalRacePlanner)",
        "map_sha256": hashlib.sha256(canonical_map.encode()).hexdigest(),
        "map": data, "settings": asdict(settings), "resolved_config": config,
        "path": path.points.tolist(), "velocity": velocity.tolist(),
        "curvature": curvature.tolist(), "distance_m": path.s[:-1].tolist(),
        "length_m": path.length,
    }
    return PreparedPlan(planner, plan, path, corridor, settings, artifact)
