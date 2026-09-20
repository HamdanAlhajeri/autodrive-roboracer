"""Estimate straight-line throttle-off response from recorded telemetry, without driving."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .plot_recording import read_samples


def analyze_response(rows):
    """Require three independent, fresh, straight coast intervals of >=0.3 s.

    Fits measured velocity against wall time. Turn-induced slowing and stale
    samples must not be mistaken for braking authority. This is experimental
    evidence, not a guarantee of grip or hardware braking performance.
    """
    episodes, current = [], []

    def finish():
        if len(current) >= 4:
            t = np.array([r["elapsed_s"] for r in current])
            v = np.array([r["speed"] for r in current])
            duration = t[-1] - t[0]
            if duration >= 0.3 - 1e-8 and v[0] - v[-1] >= 0.15:
                slope, intercept = np.polyfit(t - t[0], v, 1)
                residual = float(np.sqrt(np.mean((v - (intercept + slope * (t - t[0])))**2)))
                if slope < 0 and residual <= 0.1:
                    episodes.append({"start_s": float(t[0]), "duration_s": float(duration),
                                     "start_speed_mps": float(v[0]),
                                     "end_speed_mps": float(v[-1]),
                                     "deceleration_mps2": float(-slope),
                                     "fit_rms_mps": residual,
                                     "distance_m": float(np.sum(np.diff(t) * (v[:-1] + v[1:]) / 2)),
                                     "response_trial": current[0].get("response_trial")})
        current.clear()

    def number(row, name):
        value = row.get(name)
        return value if isinstance(value, (int, float)) and math.isfinite(value) else math.nan

    for row in rows:
        fresh = all(0 <= number(row, key) <= 0.15 for key in (
            "odom_age_s", "throttle_command_age_s", "throttle_age_s", "steering_age_s"))
        coast = (fresh and number(row, "speed") > 0.5
                 and 0 <= number(row, "throttle_command") <= 0.005
                 and 0 <= number(row, "throttle") <= 0.005
                 and abs(number(row, "steering")) <= 0.05)
        if row.get("response_phase") is not None:
            coast = (coast and row["response_phase"] == 1 and number(row, "response_trial") > 0
                     and 0 <= number(row, "controller_diagnostics_age_s") <= 0.15)
        if current:
            dt = number(row, "elapsed_s") - current[-1]["elapsed_s"]
            if (not 0 < dt <= 0.2 or row.get("resets") != current[-1].get("resets")
                    or row.get("collision_count") != current[-1].get("collision_count")
                    or row.get("response_trial") != current[-1].get("response_trial")):
                finish()
        if coast:
            current.append(row)
        else:
            finish()
    finish()
    # A sensor interruption must not turn one physical coast into two independent trials.
    selected = {}
    for episode in episodes:
        trial = episode["response_trial"]
        if trial is not None and (trial not in selected
                                  or episode["duration_s"] > selected[trial]["duration_s"]):
            selected[trial] = episode
    episodes = [e for e in episodes if e["response_trial"] is None] + list(selected.values())
    enough = len(episodes) >= 3
    conservative = 0.8 * min(e["deceleration_mps2"] for e in episodes) if enough else None
    return {
        "qualified": enough, "coast_episodes": episodes,
        "suggested_braking_deceleration_mps2": conservative,
        "method": "80% of the minimum fitted deceleration across >=3 straight coast intervals",
        "next_step": ("Review the intervals/graphs and use no more than this value in planning."
                      if enough else "Collect three fresh straight throttle-off intervals; "
                      "keep braking_calibrated false and the 2.5 m/s commissioning cap."),
        "limitations": "Simulator response over the recorded speed range only; "
                       "does not automatically change configuration or certify higher speeds.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    episodes = []
    for filename in args.recordings:
        result = analyze_response(read_samples(filename))
        episodes.extend(dict(episode, recording=str(filename))
                        for episode in result["coast_episodes"])
    result["coast_episodes"] = episodes
    result["qualified"] = len(episodes) >= 3
    result["suggested_braking_deceleration_mps2"] = (
        0.8 * min(e["deceleration_mps2"] for e in episodes) if result["qualified"] else None)
    result["next_step"] = ("Review qualified intervals before setting braking_calibrated true."
                           if result["qualified"] else "Insufficient straight coast data; "
                           "keep braking_calibrated false.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
