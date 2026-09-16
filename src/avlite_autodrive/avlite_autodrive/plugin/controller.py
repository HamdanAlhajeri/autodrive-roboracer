"""Dense-scan gap selection extending AVLite's upstream Pure Pursuit + speed PID."""

import math
from collections.abc import Mapping

import numpy as np

from avlite.c30_control.c35_pure_pursuit import FollowTheGapController
from avlite.c30_control.c31_control_model import ControlCommand


class AutoDRIVEFollowTheGap(FollowTheGapController):
    # Seed assumptions for simulator experiments, NOT measured vehicle limits.
    RACING_DEFAULTS = {
        "lateral_acceleration_mps2": 3.0,
        "braking_deceleration_mps2": 1.5,
        "reaction_time_s": 0.25,
        "clearance_margin_m": 0.15,
    }

    def __init__(self, *args, racing=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.racing = None
        if racing is not None:
            if not isinstance(racing, Mapping):
                raise ValueError("racing must be a settings mapping")
            unknown = set(racing) - (self.RACING_DEFAULTS.keys() | {"gap_preview_min_m"})
            if unknown:
                raise ValueError(f"Unknown racing setting(s): {sorted(unknown)}")
            self.racing = dict(self.RACING_DEFAULTS, **racing)
            for key, value in self.racing.items():
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f"racing.{key} must be a positive finite number")
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"racing.{key} must be a positive finite number")
                self.racing[key] = float(value)
            if "gap_preview_min_m" in self.racing:
                bounds = (self.min_lookahead, self.max_lookahead)
                if any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    or value <= 0
                    for value in bounds
                ) or self.min_lookahead > self.max_lookahead:
                    raise ValueError(
                        "Gap preview requires positive finite ordered lookahead bounds"
                    )
                if not self.min_lookahead <= self.racing["gap_preview_min_m"] <= self.max_lookahead:
                    raise ValueError("racing.gap_preview_min_m must be within the lookahead bounds")
            if self.racing["braking_deceleration_mps2"] > -self.ego_min_acceleration:
                raise ValueError(
                    "Racing braking assumption exceeds the configured deceleration limit"
                )
        self._clear_diagnostics()
        self.bearing = 0.0
        self.blocked = True

    def _clear_diagnostics(self):
        self.diagnostics = {
            "target_velocity_mps": 0.0,
            "lookahead_m": 0.0,
            "gap_preview_requested_m": 0.0,
            "gap_preview_selected_m": 0.0,
            "gap_preview_fallback": False,
            "target_bearing_raw_rad": 0.0,
            "target_bearing_rad": 0.0,
            "clearance_m": 0.0,
            "curvature_1pm": 0.0,
            "curvature_speed_limit_mps": float(self.cruise_velocity),
            "clearance_speed_limit_mps": float(self.cruise_velocity),
            "steering_saturated": False,
            "acceleration_saturated": False,
        }

    def reset(self):
        super().reset()
        self.bearing = 0.0
        self.blocked = True
        self._clear_diagnostics()

    def _corridor_clearances(self, points, candidates):
        """Distance a disk can travel along each ray, capped by observed range.

        This is a local stopping corridor, not a collision-free curved trajectory.
        Nearby observed rays bound the sensing horizon; absent returns in a
        direction cannot imply unlimited free space. The bridge already converts
        valid no-return beams to range_max, and corrupt beams to close obstacles.
        """
        along = points[:, 0, None] * np.cos(candidates) + points[:, 1, None] * np.sin(candidates)
        across = abs(
            -points[:, 0, None] * np.sin(candidates) + points[:, 1, None] * np.cos(candidates)
        )
        radius = self.bubble_radius
        in_corridor = (along > 0) & (across < radius)
        first_contact = along - np.sqrt(np.maximum(radius**2 - across**2, 0.0))
        collision_distance = np.min(np.where(in_corridor, first_contact, np.inf), axis=0)

        bearings = np.arctan2(points[:, 1], points[:, 0])
        ranges = np.hypot(points[:, 0], points[:, 1])
        # Dense AutoDRIVE scans normally have many samples in this angular band.
        supported = abs(bearings[:, None] - candidates) <= np.deg2rad(1.5)
        horizon = np.min(np.where(supported, ranges[:, None], np.inf), axis=0)
        horizon = np.where(np.isfinite(horizon), np.maximum(horizon - radius, 0.0), 0.0)
        return np.maximum(np.minimum(collision_distance, horizon), 0.0)

    def _select_opening(self, free, candidates, preferred):
        indices = np.flatnonzero(free)
        groups = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
        groups = [group for group in groups if len(group) >= 5]
        if not groups:
            return None
        # Prefer wide openings, breaking near ties toward the previous target.
        return max(groups, key=lambda g: len(g) - 12 * abs(candidates[g[len(g) // 2]] - preferred))

    def largest_gap_target(self, ego_pts, ld, preferred_bearing=None):
        self._clear_diagnostics()
        # Gap selection can inspect farther ahead without weakening tight-turn
        # steering by lengthening the Pure Pursuit target radius as well.
        requested = float(ld)
        if self.racing is not None and "gap_preview_min_m" in self.racing:
            requested = min(max(requested, self.racing["gap_preview_min_m"]), self.max_lookahead)
        self.diagnostics["gap_preview_requested_m"] = float(requested)
        points = np.asarray(ego_pts, dtype=float)
        if points.ndim != 2 or points.shape[1] < 2:
            self.blocked = True
            return None
        points = points[:, :2]
        points = points[np.isfinite(points).all(axis=1)]
        candidates = np.linspace(-np.deg2rad(80), np.deg2rad(80), 161)
        if len(points) < 16:
            self.blocked = True
            return None
        preferred = self.bearing if preferred_bearing is None else preferred_bearing
        if self.racing is None:
            # Preserve the established baseline unless racing settings are supplied.
            radius = self.bubble_radius
            along = (
                points[:, 0, None] * np.cos(candidates) + points[:, 1, None] * np.sin(candidates)
            )
            across = abs(
                -points[:, 0, None] * np.sin(candidates) + points[:, 1, None] * np.cos(candidates)
            )
            blocked = np.any((along > 0) & (along < ld + radius) & (across < radius), axis=0)
            group = self._select_opening(~blocked, candidates, preferred)
        else:
            clearances = self._corridor_clearances(points, candidates)
            group = None
            # A long preview ray can fail at a navigable tight bend. Try shorter
            # horizons, retaining the clearance/curvature speed constraints.
            for horizon in np.linspace(requested, min(ld, self.min_lookahead), 6):
                group = self._select_opening(clearances >= horizon, candidates, preferred)
                if group is not None:
                    self.diagnostics["gap_preview_selected_m"] = float(horizon)
                    self.diagnostics["gap_preview_fallback"] = bool(horizon < requested - 1e-9)
                    ld = min(ld, float(horizon))
                    break
        if group is None:
            self.blocked = True
            return None
        raw = float(candidates[group[len(group) // 2]])
        bearing = 0.65 * raw + 0.35 * self.bearing
        # Smoothing must never interpolate out of the selected safe opening.
        self.bearing = float(np.clip(bearing, candidates[group[0]], candidates[group[-1]]))
        self.blocked = False
        if self.racing is None:
            self.diagnostics["gap_preview_selected_m"] = float(ld)
        self.diagnostics["target_bearing_raw_rad"] = raw
        self.diagnostics["target_bearing_rad"] = self.bearing
        self.diagnostics["lookahead_m"] = float(ld)
        self.diagnostics["curvature_1pm"] = float(2.0 * np.sin(self.bearing) / max(ld, 1e-3))
        if self.racing is not None:
            # Include the current heading: steering cannot instantly redirect
            # the car, so a long clear selected ray must not hide a wall ahead.
            clearance = float(
                np.min(self._corridor_clearances(points, np.array([0.0, self.bearing])))
            )
            self.diagnostics["clearance_m"] = clearance
            distance = max(clearance - self.racing["clearance_margin_m"], 0.0)
            braking = self.racing["braking_deceleration_mps2"]
            reaction = self.racing["reaction_time_s"]
            # Solve d = v*t_reaction + v^2/(2*a_brake) for v.
            # Rationalize the root to avoid cancellation for short distances.
            root = math.hypot(braking * reaction, math.sqrt(2 * braking * distance))
            clearance_limit = 2 * braking * distance / (root + braking * reaction)
            curvature = max(abs(self.diagnostics["curvature_1pm"]), abs(2 * math.sin(raw) / ld))
            curvature_limit = math.sqrt(
                self.racing["lateral_acceleration_mps2"] / max(curvature, 1e-9)
            )
            self.diagnostics["curvature_speed_limit_mps"] = float(
                min(self.cruise_velocity, curvature_limit)
            )
            self.diagnostics["clearance_speed_limit_mps"] = float(
                min(self.cruise_velocity, clearance_limit)
            )
        return ld * np.cos(self.bearing), ld * np.sin(self.bearing)

    def steer_to_ego_target(self, target_ego_xy, ld):
        # Upstream retains its original Ld after gap selection. A shortened
        # fallback target needs its actual radius in the bicycle-model equation.
        actual_ld = math.hypot(*target_ego_xy) if self.racing is not None else ld
        return super().steer_to_ego_target(target_ego_xy, actual_ld)

    def velocity_pid(self, ego, target_velocity):
        demand = max(0.0, min(target_velocity, self.cruise_velocity))
        if self.racing is None:
            demand *= max(0.35, np.cos(self.bearing) ** 2)
        else:
            demand = min(
                demand,
                self.diagnostics["curvature_speed_limit_mps"],
                self.diagnostics["clearance_speed_limit_mps"],
            )
        self.diagnostics["target_velocity_mps"] = float(demand)
        acceleration = super().velocity_pid(ego, demand)
        if self.racing is not None and ego.velocity > demand + 0.05:
            # A P-only speed loop otherwise approaches a falling speed limit
            # with a persistent lag. Apply the assumed deceleration promptly
            # whenever outside the envelope; actual response must be measured.
            acceleration = min(acceleration, -self.racing["braking_deceleration_mps2"])
        return float(acceleration)

    def control(self, ego, plan=None, control_dt=None, perception_model=None, sensors=None):
        self.blocked = True
        self._clear_diagnostics()
        cmd = super().control(ego, plan, control_dt, perception_model, sensors)
        if self.blocked:
            self.cte_v_sum = 0.0
            self.cte_velocity = 0.0
            cmd = ControlCommand(steer=0.0, acceleration=self.ego_min_acceleration)
        self.diagnostics["steering_saturated"] = bool(
            cmd.steer <= self.ego_min_steering + 1e-6 or cmd.steer >= self.ego_max_steering - 1e-6
        )
        self.diagnostics["acceleration_saturated"] = bool(
            cmd.acceleration <= self.ego_min_acceleration + 1e-6
            or cmd.acceleration >= self.ego_max_acceleration - 1e-6
        )
        self.cmd = cmd
        return cmd
