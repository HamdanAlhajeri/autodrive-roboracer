"""AVLite Pure Pursuit with periodic racing and a LiDAR stopping envelope."""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree
from avlite.c30_control.c31_control_model import ControlCommand
from avlite.c30_control.c35_pure_pursuit import PurePursuitController
from avlite.c50_common.c51_capabilities import StackCapability, WorldCapability
from avlite.c50_common.c52_world_sensor_datatypes import SensorFrame

from avlite_autodrive.race_map import footprint


# Numeric codes preserve the existing telemetry allowlist's numeric-only contract.
LIMIT_REASONS = {
    0: "speed ceiling", 1: "planned corner/braking", 2: "obstacle stopping",
    3: "invalid pose/corridor", 4: "missing/stale scan", 5: "commissioning ceiling",
    6: "invalid plan",
}


def swept_hit_distance(hits, xy, directions, distance, wheelbase, radius):
    """Find the first sampled distance where the vehicle footprint touches a LiDAR hit.

    xy and directions describe rear-axle poses along the path; distance gives their travel
    distances in metres. Model the car as an axle segment surrounded by a radius. A spatial
    index removes distant points before exact checks. Return infinity if no sampled
    footprint is blocked.
    """
    if not len(hits):
        return math.inf
    # Every capsule fits in this ball about its axle midpoint. Filtering by the
    # ball therefore cannot discard a hit on either end cap or the axle segment.
    centers = xy + 0.5 * wheelbase * directions
    nearby = cKDTree(hits).query_ball_point(centers, 0.5 * wheelbase + radius + 1e-9)
    pose_ids = np.repeat(np.arange(len(xy)), [len(group) for group in nearby])
    if not len(pose_ids):
        return math.inf
    hit_ids = np.concatenate(nearby).astype(np.intp)
    offset = hits[hit_ids] - xy[pose_ids]
    unit = directions[pose_ids]
    along = np.clip(np.sum(offset * unit, axis=1), 0, wheelbase)
    separation = offset - along[:, None] * unit
    blocked = np.sum(separation**2, axis=1) <= radius**2
    return float(np.min(distance[pose_ids[blocked]])) if np.any(blocked) else math.inf


@dataclass
class AutoDRIVESensorFrame(SensorFrame):
    lidar_hit_mask: object = None
    lidar_age_s: float = math.inf
    odom_age_s: float = math.inf


class AutoDRIVEPlannedController(PurePursuitController):
    world_requirements = frozenset({WorldCapability.LIDAR_2D})
    stack_requirements = frozenset({StackCapability.LOCAL_PLAN, StackCapability.LOCALIZATION})

    def __init__(self, prepared, **kwargs):
        """Attach the validated map, closed path and planning limits to AVLite Pure Pursuit."""
        super().__init__(**kwargs)
        self.prepared = prepared
        self.path = prepared.path
        self.settings = prepared.settings
        self.reset()

    def reset(self):
        """Clear path progress, the speed target and upstream controller memory.

        Mark the plan unconfirmed until a later control update passes its checks.
        """
        super().reset()
        self._s = None
        self._target_speed = 0.0
        self.diagnostics = {"planned_mode": 1, "target_velocity_mps": 0.0,
                            "speed_limit_reason": 6, "plan_valid": 0}

    def stop_command(self, reason):
        """Return straight steering and a deceleration request after a failed check.

        The numeric reason is saved for telemetry. Reset upstream control memory and path
        progress so recovery does not reuse an old tracking state.
        """
        super().reset()
        self._s = None
        self.diagnostics.update(target_velocity_mps=0.0, speed_limit_reason=reason,
                                plan_valid=0, acceleration_saturated=True)
        self.cmd = ControlCommand(steer=0.0, acceleration=self.ego_min_acceleration)
        return self.cmd

    def find_path_lookahead(self, ego, ld):
        """Locate the car on the loop and return a lookahead point in the car frame.

        ld is a distance in metres. ClosedPath.at() wraps across the finish line, so the
        target can lie in the next lap without clamping to the last waypoint.
        """
        s, cte, index = self.path.project([ego.x, ego.y])
        self._s = s
        self.tj.current_wp = index
        self.cte_steer = cte
        target = self.path.at(s + ld) - [ego.x, ego.y]
        c, sn = math.cos(ego.theta), math.sin(ego.theta)
        self.diagnostics.update(path_progress_m=s, path_deviation_m=cte, lookahead_m=ld)
        return (float(c * target[0] + sn * target[1]),
                float(-sn * target[0] + c * target[1]))

    def steer_to_ego_target(self, target_ego_xy, ld):
        """Calculate Pure Pursuit steering using the actual distance to the local target."""
        return super().steer_to_ego_target(target_ego_xy, math.hypot(*target_ego_xy))

    def velocity_pid(self, ego, target_velocity):
        # The closed-path profile is interpolated at the actual position, including
        # the seam. Upstream's nearest open-path waypoint value is intentionally unused.
        """Track the speed target already limited by the closed-path and obstacle checks.

        Ignore the upstream waypoint target in favour of our interpolated value. When
        measured speed is above it, request at least the configured deceleration.
        """
        target = self._target_speed
        acc = super().velocity_pid(ego, target)
        if ego.velocity > target + 0.05:
            acc = min(acc, -self.settings.braking_deceleration_mps2)
        return float(acc)

    def obstacle_distance(self, ego, sensors, steer, s):
        """Check how far the car can travel before a measured LiDAR hit enters its footprint.

        Transform hits into world coordinates, then sample the curved reference path and the
        car's current steering arc during reaction time. Ignore valid no-return beams.
        Return clearance in metres, or infinity when these checks find no hit.
        """
        mask = np.asarray(sensors.lidar_hit_mask, dtype=bool)
        hits = sensors.lidar_sensor.to_map(sensors.lidar[mask], ego)[:, :2]
        if len(hits) == 0:
            return math.inf
        st = self.settings
        speed = max(ego.velocity, st.speed_limit)
        horizon = min(self.path.length, speed * st.reaction_time_s
                      + speed**2 / (2 * st.braking_deceleration_mps2) + 1.0)
        distance = np.arange(0, horizon + 0.025, 0.05)
        xy = self.path.at(s + distance)
        directions = self.path.at(s + distance + 0.025) - xy
        directions /= np.linalg.norm(directions, axis=1)[:, None]
        # Include the actual steering arc during reaction, rather than teleporting
        # a displaced car onto the reference line for obstacle checks.
        arc_distance = distance[distance <= max(ego.velocity, 0) * st.reaction_time_s + 0.1]
        curvature = math.tan(steer) / st.wheelbase_m
        theta = ego.theta + curvature * arc_distance
        if abs(curvature) < 1e-6:
            arc = np.array([ego.x, ego.y]) + arc_distance[:, None] * [
                math.cos(ego.theta), math.sin(ego.theta)]
        else:
            arc = np.column_stack((ego.x + (np.sin(theta) - math.sin(ego.theta)) / curvature,
                                   ego.y - (np.cos(theta) - math.cos(ego.theta)) / curvature))
        xy = np.vstack([xy, arc])
        directions = np.vstack([directions, np.column_stack([np.cos(theta), np.sin(theta)])])
        distance = np.r_[distance, arc_distance]
        # Distance of each hit to each rear-to-front-axle segment. Add half the
        # sampling step to the disk radius so discrete poses don't miss a contact.
        return swept_hit_distance(hits, xy, directions, distance, st.wheelbase_m,
                                  st.envelope_radius + 0.025)

    def control(self, ego, plan=None, control_dt=None, perception_model=None, sensors=None):
        """Validate inputs, follow the closed path and choose the final speed target.

        First require a matching plan, fresh sensors and a correctly oriented pose inside
        the track. Then take the lowest of the effective speed ceiling, upcoming planned
        speed and obstacle stopping limit. Pure Pursuit produces steering and acceleration,
        while diagnostics record why speed was limited.
        """
        self.diagnostics = {"planned_mode": 1, "plan_valid": 1,
                            "planned_speed_mps": 0.0, "target_velocity_mps": 0.0,
                            "speed_limit_reason": 0, "obstacle_speed_limit_mps": None,
                            "clearance_m": None}
        if (plan is None or not plan.path or len(plan.path) != len(self.path.points)
                or not np.allclose(plan.path, self.path.points)):
            return self.stop_command(6)
        if (sensors is None or sensors.lidar is None
                or getattr(sensors, "lidar_hit_mask", None) is None
                or len(sensors.lidar) < 16
                or np.shape(sensors.lidar_hit_mask) != (len(sensors.lidar),)
                or not np.isfinite(sensors.lidar).all()
                or not 0 <= sensors.lidar_age_s <= 0.5 or not 0 <= sensors.odom_age_s <= 0.5):
            return self.stop_command(4)
        if not all(math.isfinite(v) for v in (ego.x, ego.y, ego.theta, ego.velocity)):
            return self.stop_command(3)
        st = self.settings
        s, cte, index = self.path.project([ego.x, ego.y])
        forward = self.path.tangents[index]
        aligned = np.dot(forward, [math.cos(ego.theta), math.sin(ego.theta)]) > 0.5
        if (not aligned or not self.prepared.corridor.covers(
                footprint([ego.x, ego.y], ego.theta, st.wheelbase_m, st.envelope_radius))):
            return self.stop_command(3)
        ld = self.effective_lookahead(ego.velocity)
        delta = self.path.at(s + ld) - [ego.x, ego.y]
        c, sn = math.cos(ego.theta), math.sin(ego.theta)
        target = (c * delta[0] + sn * delta[1], -sn * delta[0] + c * delta[1])
        steer = self.steer_to_ego_target(target, ld)
        clearance = self.obstacle_distance(ego, sensors, steer, s)
        planned = float(self.path.velocity_at(s, self.prepared.global_plan.velocity))
        preview = np.arange(0, max(ego.velocity, 0) * st.reaction_time_s + 0.05, 0.05)
        profile_limit = float(np.min(self.path.velocity_at(
            s + preview, self.prepared.global_plan.velocity)))
        obstacle_limit = st.speed_limit
        if math.isfinite(clearance):
            # Solve stopping distance = v*reaction_time + v^2/(2*braking)
            # for v. The margin leaves some distance before the detected hit.
            distance = max(0, clearance - st.clearance_margin_m)
            braking = st.braking_deceleration_mps2
            bt = braking * st.reaction_time_s
            obstacle_limit = min(st.speed_limit, 2 * braking * distance / (
                math.hypot(bt, math.sqrt(2 * braking * distance)) + bt))
        self._target_speed = min(st.speed_limit, profile_limit, obstacle_limit)
        reason = 5 if not st.braking_calibrated and st.max_velocity_mps > st.speed_limit else 0
        if profile_limit < st.speed_limit - 1e-5:
            reason = 1
        if obstacle_limit < min(profile_limit, st.speed_limit) - 1e-5:
            reason = 2
        self.diagnostics.update(
            planned_speed_mps=planned, target_velocity_mps=self._target_speed,
            speed_limit_reason=reason, obstacle_speed_limit_mps=obstacle_limit,
            clearance_m=clearance if math.isfinite(clearance) else None,
            path_progress_m=s, path_deviation_m=cte,
        )
        cmd = super().control(ego, plan, control_dt, perception_model, sensors)
        self.diagnostics.update(
            steering_saturated=abs(cmd.steer) >= self.ego_max_steering - 1e-6,
            acceleration_saturated=(cmd.acceleration <= self.ego_min_acceleration + 1e-6
                                    or cmd.acceleration >= self.ego_max_acceleration - 1e-6),
        )
        return cmd
