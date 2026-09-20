"""Keep planned steering and guards while collecting straight coast attempts."""

import time

from avlite.c30_control.c31_control_model import ControlCommand

from avlite_autodrive.response_test import CoastExperiment
from .planned_controller import AutoDRIVEPlannedController


class ResponseController(AutoDRIVEPlannedController):
    def __init__(self, prepared, speed, trials):
        if speed > prepared.settings.speed_limit + 1e-6:
            raise ValueError("Response target exceeds the effective commissioning ceiling")
        super().__init__(prepared)
        self.experiment = CoastExperiment(
            prepared.path, prepared.artifact["curvature"], speed, trials,
            prepared.settings.wheelbase_m, self.ego_max_steering)

    def reset(self):
        super().reset()
        experiment = getattr(self, "experiment", None)
        if experiment is not None:
            experiment.phase = 2 if experiment.trial >= experiment.trials else 0
            experiment.started = experiment.last_now = experiment.last_s = None
            experiment.last_attempt_distance = experiment.distance

    def control(self, ego, plan=None, control_dt=None, perception_model=None, sensors=None):
        cmd = super().control(ego, plan, control_dt, perception_model, sensors)
        now = time.monotonic()
        diag = self.diagnostics
        fresh = (sensors is not None and 0 <= sensors.odom_age_s <= 0.15
                 and 0 <= sensors.lidar_age_s <= 0.15)
        coast = self.experiment.step(
            now, diag.get("path_progress_m", self.experiment.last_s or 0.0),
            ego.velocity, cmd.steer, bool(diag.get("plan_valid"))
            and diag.get("speed_limit_reason") not in (2, 3, 4, 6), fresh,
            diag.get("target_velocity_mps", 0.0))
        if coast:
            # This requests zero forward throttle, not a known physical brake force.
            cmd = ControlCommand(steer=cmd.steer, acceleration=min(cmd.acceleration, -0.2))
            self.cmd = cmd
        diag.update(self.experiment.diagnostics(now))
        return cmd
