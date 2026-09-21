"""Temporary low-speed planned controller for deliberate throttle-off measurements."""

import math
import time

import numpy as np
from avlite.c30_control.c31_control_model import ControlCommand

from .planned_controller import AutoDRIVEPlannedController
from ..response_test import ResponseExperiment


class AutoDRIVEResponseController(AutoDRIVEPlannedController):
    def __init__(self, prepared, speed, trials, clock=time.monotonic):
        super().__init__(prepared)
        self.experiment = ResponseExperiment(speed, trials)
        self.clock = clock

    def reset(self):
        super().reset()
        if hasattr(self, "experiment"):
            self.experiment.reset()

    def control(self, ego, plan=None, control_dt=None, perception_model=None, sensors=None):
        cmd = super().control(ego, plan, control_dt, perception_model, sensors)
        diag, experiment = self.diagnostics, self.experiment
        safe = bool(diag.get("plan_valid"))
        s = diag.get("path_progress_m", 0.0)
        # Require enough nearly straight reference path for the full coast window.
        preview = s + np.linspace(0, max(1.0, experiment.speed * 0.8 + 0.5), 12)
        tangents = self.path.at(preview + 0.05) - self.path.at(preview)
        angles = np.unwrap(np.arctan2(tangents[:, 1], tangents[:, 0]))
        eligible = (safe and abs(cmd.steer) <= 0.05 * self.ego_max_steering
                    and abs(diag.get("path_deviation_m", math.inf)) <= 0.05
                    and diag.get("target_velocity_mps", 0) >= 0.95 * experiment.speed
                    and np.ptp(angles) <= 0.06
                    and sensors is not None
                    and 0 <= sensors.lidar_age_s <= 0.15
                    and 0 <= sensors.odom_age_s <= 0.15)
        coast = experiment.step(self.clock(), s, self.path.length, ego.velocity, eligible, safe)
        if coast or experiment.phase == experiment.ABORTED:
            # Negative acceleration requests zero throttle at the existing actuator.
            # It does not claim a known braking force.
            cmd = ControlCommand(steer=cmd.steer, acceleration=self.ego_min_acceleration)
            self.cmd = cmd
        self.diagnostics.update(experiment.diagnostics())
        return cmd
