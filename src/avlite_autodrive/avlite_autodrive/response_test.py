"""Bounded throttle-off experiments; no ROS dependencies or configuration writes."""

import copy
import math

import numpy as np


def measurement_config(config, speed):
    if not math.isfinite(speed) or not 1.0 <= speed <= 2.5:
        raise ValueError("Response tests require a target from 1.0 to 2.5 m/s")
    result = copy.deepcopy(config)
    result["driving_mode"] = "planned"
    control = result["c30_control"]
    for key in ("c32_ego_max_velocity", "c35_cruise_velocity"):
        control[key] = speed
    result.setdefault("planning", {}).update(
        max_velocity_mps=speed, braking_calibrated=False)
    return result


class CoastExperiment:
    """At most one attempt per circuit, without increasing the normal command.

    A trial is an attempt, not a qualified measurement. The offline analyzer
    decides whether fresh, straight, zero-throttle feedback supports a fit.
    """

    def __init__(self, path, curvature, speed, trials, wheelbase, max_steer):
        if not 1 <= trials <= 20 or not isinstance(trials, int):
            raise ValueError("Response trials must be an integer from 1 to 20")
        self.path = path
        self.speed, self.trials = speed, trials
        self.coast_seconds = 0.8
        self.steer_limit = 0.05 * max_steer  # Same normalized straightness as analyzer.
        self.curvature_limit = math.tan(self.steer_limit) / wheelbase
        self.curvature = np.asarray(curvature)
        # Reserve extra distance for reaction and leave 0.5 m before the bend.
        self.preview_distance = speed * (self.coast_seconds + 0.25) + 0.5
        if not any(self.straight_ahead(s) for s in path.s[:-1]):
            raise ValueError("No sufficiently long straight for this response test")
        self.trial = 0
        self.phase = 0  # 0 ordinary planned driving, 1 coast, 2 all attempts finished.
        self.started = None
        self.last_s = None
        self.distance = 0.0
        self.last_attempt_distance = -math.inf
        self.last_now = None

    def straight_ahead(self, s):
        positions = (s + np.arange(0, self.preview_distance + 0.05, 0.05)) % self.path.length
        indices = np.searchsorted(self.path.s, positions, side="right") - 1
        return bool(np.all(np.abs(self.curvature[indices]) <= self.curvature_limit))

    def step(self, now, s, speed, steer, valid, fresh, target):
        if self.last_now is not None and not 0 < now - self.last_now <= 0.15:
            fresh = False
        self.last_now = now
        if self.last_s is not None:
            delta = (s - self.last_s + self.path.length / 2) % self.path.length - self.path.length / 2
            if not 0 <= delta <= 0.75:
                fresh = False
            else:
                self.distance += delta
        self.last_s = s
        eligible = (valid and fresh and abs(steer) <= self.steer_limit
                    and self.straight_ahead(s))
        if self.phase == 1:
            if not eligible or now - self.started >= self.coast_seconds or speed <= 0.6:
                self.phase = 2 if self.trial >= self.trials else 0
                self.started = None
        elif (self.phase == 0 and eligible and speed >= self.speed - 0.08
              and target >= self.speed - 0.01
              and self.distance - self.last_attempt_distance >= self.path.length * 0.9):
            self.trial += 1
            self.phase = 1
            self.started = now
            self.last_attempt_distance = self.distance
        return self.phase == 1

    def diagnostics(self, now):
        return {"response_phase": self.phase, "response_trial": self.trial,
                "response_target_speed_mps": self.speed,
                "response_elapsed_s": now - self.started if self.started is not None else 0.0}

    def metadata(self):
        return {"target_speed_mps": self.speed, "requested_trials": self.trials,
                "coast_seconds": self.coast_seconds,
                "straight_steering_limit_rad": self.steer_limit,
                "preview_distance_m": self.preview_distance,
                "note": "Attempt counts are not calibration acceptance; inspect response-report.json."}
