"""Temporary low-speed planned controller for deliberate throttle-off measurements."""

import math
import time

from avlite.c30_control.c31_control_model import ControlCommand

from .planned_controller import AutoDRIVEPlannedController
from ..response_test import ResponseExperiment, StraightCoastWindow


class AutoDRIVEResponseController(AutoDRIVEPlannedController):
    def __init__(self, prepared, speed, trials, clock=time.monotonic):
        """Add a timed coasting experiment to the normal planned controller.

        The injectable monotonic clock allows repeatable tests without waiting in real time.
        """
        if speed > prepared.settings.speed_limit + 1e-6:
            raise ValueError("Response target exceeds the effective commissioning ceiling")
        super().__init__(prepared)
        self.experiment = ResponseExperiment(speed, trials)
        self.coast_window = StraightCoastWindow(
            prepared.path, speed, self.ego_max_steering)
        self.clock = clock
        self.last_now = self.last_s = None

    def reset(self):
        """Reset path tracking and invalidate an experiment that had already started."""
        super().reset()
        if hasattr(self, "experiment"):
            self.experiment.reset()
        self.last_now = self.last_s = None

    def control(self, ego, plan=None, control_dt=None, perception_model=None, sensors=None):
        """Follow the racing line while checking whether a coast measurement can begin.

        Require fresh sensors, small steering and path error, and a nearly straight route
        ahead. During a coast or abort, retain the planned steering but request zero
        throttle through the actuator's deceleration branch.
        """
        cmd = super().control(ego, plan, control_dt, perception_model, sensors)
        diag, experiment = self.diagnostics, self.experiment
        safe = bool(diag.get("plan_valid"))
        s = diag.get("path_progress_m", 0.0)
        now = self.clock()
        if self.last_now is not None and not 0 < now - self.last_now <= 0.15:
            safe = False
        if self.last_s is not None:
            delta = ((s - self.last_s + self.path.length / 2) % self.path.length
                     - self.path.length / 2)
            if not 0 <= delta <= 0.75:
                safe = False
        self.last_now, self.last_s = now, s
        eligible = (safe and abs(cmd.steer) <= self.coast_window.steer_limit
                    and self.coast_window.straight_ahead(s)
                    and diag.get("speed_limit_reason") not in (2, 3, 4, 6)
                    and abs(diag.get("path_deviation_m", math.inf)) <= 0.05
                    and diag.get("target_velocity_mps", 0) >= 0.95 * experiment.speed
                    and sensors is not None
                    and 0 <= sensors.lidar_age_s <= 0.15
                    and 0 <= sensors.odom_age_s <= 0.15)
        coast = experiment.step(now, s, self.path.length, ego.velocity, eligible, safe)
        if coast or experiment.phase == experiment.ABORTED:
            # Negative acceleration requests zero throttle at the existing actuator.
            # It does not claim a known braking force.
            cmd = ControlCommand(steer=cmd.steer, acceleration=self.ego_min_acceleration)
            self.cmd = cmd
        self.diagnostics.update(experiment.diagnostics())
        self.diagnostics["response_elapsed_s"] = (
            now - experiment.coast_since if coast else 0.0)
        return cmd


# Preserve the original Python entry point while using one experiment implementation.
ResponseController = AutoDRIVEResponseController
