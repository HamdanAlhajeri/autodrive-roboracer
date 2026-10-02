"""Estimate straight-line throttle-off response from recorded telemetry, without driving."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .plot_recording import read_samples


def analyze_response(rows):
    """Estimate throttle-off deceleration from ordinary recorded coast intervals.

    Require fresh data, near-straight steering and at least 0.3 seconds of slowing per
    interval. Three accepted intervals allow a conservative suggestion. This older analysis
    uses sampled elapsed time and does not establish braking at untested speeds.
    """
    episodes, current = [], []

    def finish():
        """Fit the current coast interval if it has enough samples and slowing, then clear it.

        Accepted fits have a negative speed slope and a small residual error.
        """
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
                                     "fit_rms_mps": residual})
        current.clear()

    def number(row, name):
        """Read a finite numeric field, or return NaN so incomplete samples fail qualification."""
        value = row.get(name)
        return value if isinstance(value, (int, float)) and math.isfinite(value) else math.nan

    for row in rows:
        fresh = all(0 <= number(row, key) <= 0.15 for key in (
            "odom_age_s", "throttle_command_age_s", "throttle_age_s", "steering_age_s"))
        coast = (fresh and number(row, "speed") > 0.5
                 and 0 <= number(row, "throttle_command") <= 0.005
                 and 0 <= number(row, "throttle") <= 0.005
                 and abs(number(row, "steering")) <= 0.05)
        if current:
            dt = number(row, "elapsed_s") - current[-1]["elapsed_s"]
            if (not 0 < dt <= 0.2 or row.get("resets") != current[-1].get("resets")
                    or row.get("collision_count") != current[-1].get("collision_count")):
                finish()
        if coast:
            current.append(row)
        else:
            finish()
    finish()
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
    """Analyze one or more telemetry files and save their combined coast report.

    Tag each accepted interval with its source recording and suggest no more than 80 percent
    of the smallest fitted deceleration when enough intervals exist.
    """
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
