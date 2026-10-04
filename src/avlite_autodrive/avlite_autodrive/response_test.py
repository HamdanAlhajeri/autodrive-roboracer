"""Bounded, one-coast-per-circuit response experiment, independent of ROS."""

import copy
import math

import numpy as np


def measurement_config(config, speed):
    """Build a temporary response profile without changing the saved settings."""
    if not math.isfinite(speed) or not 1.0 <= speed <= 2.5:
        raise ValueError("Response tests require a target from 1.0 to 2.5 m/s")
    result = copy.deepcopy(config)
    result["driving_mode"] = "planned"
    for key in ("c32_ego_max_velocity", "c35_cruise_velocity"):
        result["c30_control"][key] = speed
    result.setdefault("planning", {}).update(
        max_velocity_mps=speed, braking_calibrated=False)
    return result


class StraightCoastWindow:
    """Use the same straight-road check before startup and during each coast."""

    def __init__(self, path, speed, max_steer):
        self.path = path
        self.steer_limit = 0.05 * max_steer
        self.preview_distance = max(1.0, speed * 0.8 + 0.5)
        if not any(self.straight_ahead(s) for s in path.s[:-1]):
            raise ValueError("No sufficiently long straight for this response test")

    def straight_ahead(self, s):
        preview = s + np.linspace(0, self.preview_distance, 12)
        tangents = self.path.at(preview + 0.05) - self.path.at(preview)
        angles = np.unwrap(np.arctan2(tangents[:, 1], tangents[:, 0]))
        return bool(np.ptp(angles) <= 0.06)


class ResponseExperiment:
    CRUISE, COAST, RECOVER, FINISHED, ABORTED = range(1, 6)

    def __init__(self, speed, trials):
        """Validate the low-speed test request and initialize its phase and lap counters.

        The test supports 1.0-2.5 m/s and at least three independent coast attempts.
        """
        if not math.isfinite(speed) or not 1 <= speed <= 2.5:
            raise ValueError("Response speed must be between 1 and 2.5 m/s")
        if isinstance(trials, bool) or not isinstance(trials, int) or not 3 <= trials <= 20:
            raise ValueError("Response trials must be an integer between 3 and 20")
        self.speed, self.trials = speed, trials
        self.phase, self.trial = self.CRUISE, 0
        self.circuit, self.last_trial_circuit = 0, -1
        self.previous_progress = None
        self.stable_since = self.coast_since = None

    def reset(self):
        # Once moving/testing, an interruption invalidates the whole experiment.
        """Mark a started experiment as aborted after a reset or interruption.

        A reset before the first progress sample leaves it ready to begin.
        """
        if self.previous_progress is not None:
            self.phase = self.ABORTED

    def step(self, now, progress, length, speed, eligible, safe=True):
        """Advance the experiment by one sample and return whether throttle should be removed.

        Use monotonic seconds, progress around the loop in metres, and measured speed in
        m/s. Start at most one coast per circuit after speed has settled. End a coast after
        0.8 seconds or when eligibility is lost, and abort on an unsafe sample.
        """
        if self.phase == self.ABORTED:
            return False
        if not safe:
            self.phase = self.ABORTED
            return False
        if (self.previous_progress is not None
                and self.previous_progress > 0.8 * length and progress < 0.2 * length):
            self.circuit += 1
        self.previous_progress = progress
        if self.phase == self.COAST:
            if eligible and speed > 0.6 and now - self.coast_since < 0.8:
                return True
            self.phase = self.FINISHED if self.trial >= self.trials else self.RECOVER
            self.stable_since = None
        if self.phase == self.FINISHED:
            return False
        if self.last_trial_circuit == self.circuit:
            return False
        self.phase = self.CRUISE
        if not eligible or not 0.9 * self.speed <= speed <= 1.1 * self.speed:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        if now - self.stable_since < 0.2:
            return False
        self.trial += 1
        self.last_trial_circuit = self.circuit
        self.coast_since = now
        self.phase = self.COAST
        return True

    def diagnostics(self):
        """Expose the current phase and trial counters as numeric telemetry fields."""
        return {"response_phase": self.phase, "response_trial_id": self.trial,
                "response_trial": self.trial, "response_target_speed_mps": self.speed,
                "response_trials_requested": self.trials,
                "response_aborted": self.phase == self.ABORTED}

    def metadata(self):
        return {"target_speed_mps": self.speed, "requested_trials": self.trials,
                "coast_seconds": 0.8,
                "note": "Attempt counts are not calibration acceptance; "
                        "inspect response-report.json."}
