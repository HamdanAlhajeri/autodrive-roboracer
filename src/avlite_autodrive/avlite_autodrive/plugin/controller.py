"""Dense-scan gap selection extending AVLite's upstream Pure Pursuit + speed PID."""

import numpy as np

from avlite.c30_control.c35_pure_pursuit import FollowTheGapController
from avlite.c30_control.c31_control_model import ControlCommand


class AutoDRIVEFollowTheGap(FollowTheGapController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.bearing = 0.0
        self.blocked = True

    def reset(self):
        super().reset()
        self.bearing = 0.0
        self.blocked = True

    def largest_gap_target(self, ego_pts, ld, preferred_bearing=None):
        points = np.asarray(ego_pts)
        points = points[np.isfinite(points).all(axis=1)]
        candidates = np.linspace(-np.deg2rad(80), np.deg2rad(80), 161)
        if len(points) < 16:
            self.blocked = True
            return None
        # Sweep a disk through each candidate ray. Dense uniform angular scans
        # do not have useful gaps BETWEEN adjacent bearings; range/clearance is
        # what determines whether the vehicle can actually pass.
        radius = self.bubble_radius
        along = points[:, 0, None] * np.cos(candidates) + points[:, 1, None] * np.sin(candidates)
        across = abs(
            -points[:, 0, None] * np.sin(candidates) + points[:, 1, None] * np.cos(candidates)
        )
        blocked = np.any((along > 0) & (along < ld + radius) & (across < radius), axis=0)
        free = ~blocked
        groups = np.split(np.flatnonzero(free), np.where(np.diff(np.flatnonzero(free)) > 1)[0] + 1)
        groups = [g for g in groups if len(g) >= 5]
        if not groups:
            self.blocked = True
            return None
        preferred = self.bearing if preferred_bearing is None else preferred_bearing
        # Prefer wide openings, breaking near ties toward the previous target.
        group = max(groups, key=lambda g: len(g) - 12 * abs(candidates[g[len(g) // 2]] - preferred))
        raw = float(candidates[group[len(group) // 2]])
        bearing = 0.65 * raw + 0.35 * self.bearing
        # Smoothing must never interpolate out of the selected safe opening.
        self.bearing = float(np.clip(bearing, candidates[group[0]], candidates[group[-1]]))
        self.blocked = False
        return ld * np.cos(self.bearing), ld * np.sin(self.bearing)

    def velocity_pid(self, ego, target_velocity):
        # Retain AVLite's PID; lower its speed demand before tight steering.
        demand = min(target_velocity, self.cruise_velocity) * max(0.35, np.cos(self.bearing) ** 2)
        return super().velocity_pid(ego, demand)

    def control(self, ego, plan=None, control_dt=None, perception_model=None, sensors=None):
        self.blocked = True
        cmd = super().control(ego, plan, control_dt, perception_model, sensors)
        if self.blocked:
            self.cte_v_sum = 0.0
            return ControlCommand(steer=0.0, acceleration=self.ego_min_acceleration)
        return cmd
