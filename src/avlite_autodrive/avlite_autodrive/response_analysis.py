"""Response measurements and evidence-based speed-limit diagnostics, without driving."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .braking_analysis import analyze_response
from .configuration import load_config
from .plot_recording import read_samples


def number(row, key):
    value = row.get(key)
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


def analyze(rows, plan=None, actuator=None, summary=None):
    plan, actuator, summary = plan or {}, actuator or {}, summary or {}
    result = analyze_response(rows)
    settings = plan.get("settings", {})
    attempts = []
    trial_ids = sorted({r["response_trial"] for r in rows
                        if r.get("response_phase") == 1 and r.get("response_trial", 0) > 0})
    for trial in trial_ids:
        coast = [r for r in rows if r.get("response_phase") == 1 and r.get("response_trial") == trial]
        first, last = coast[0], coast[-1]
        start, end = first["elapsed_s"], last["elapsed_s"]
        before = [r for r in rows if start - 3 <= r["elapsed_s"] < start]
        # Use feedback only while its receive age is fresh. This is sampled delay,
        # not a precise actuator latency measurement at a 10 Hz recording rate.
        delays = {}
        for key, age in (("throttle_command", "throttle_command_age_s"),
                         ("throttle", "throttle_age_s")):
            zero = next((r for r in coast if number(r, key) is not None
                         and 0 <= r[key] <= 0.005 and number(r, age) is not None
                         and 0 <= r[age] <= 0.15), None)
            delays[key + "_zero_delay_s"] = zero["elapsed_s"] - start if zero else None
        acceleration = []
        for a, b in zip(before, before[1:]):
            dt = b["elapsed_s"] - a["elapsed_s"]
            if (0 < dt <= 0.2 and all(number(r, "speed") is not None
                    and number(r, "odom_age_s") is not None and 0 <= r["odom_age_s"] <= 0.15
                    and number(r, "steering") is not None and abs(r["steering"]) <= 0.05
                    for r in (a, b))):
                acceleration.append((b["speed"] - a["speed"]) / dt)
        attempts.append({"trial": trial, "start_s": start, "observed_coast_duration_s": end - start,
                         "start_speed_mps": first.get("speed"), "end_speed_mps": last.get("speed"),
                         "approach_acceleration_max_mps2": max(acceleration) if acceleration else None,
                         "qualified": any(e["response_trial"] == trial for e in result["coast_episodes"]),
                         **delays})
    requested = plan.get("response_test", {}).get("requested_trials")
    incident = (bool(summary.get("incident_detected")) or any(
        r.get("resets", 0) > rows[0].get("resets", 0)
        or (r.get("collision_count") is not None and rows[0].get("collision_count") is not None
            and r["collision_count"] != rows[0]["collision_count"]) for r in rows)) if rows else False
    # A killed recorder with no finish summary must not certify a partial run.
    incomplete = not summary.get("clean_run", False)
    if incident or incomplete or (requested and len(result["coast_episodes"]) < max(3, requested)):
        result["qualified"] = False
        result["suggested_braking_deceleration_mps2"] = None
    result["attempts"] = attempts
    result["requested_trials"] = requested
    result["recording_rejected"] = incident or incomplete
    result["next_step"] = ("Review the graph and qualified speed range before manually applying the conservative value."
                           if result["qualified"] else
                           "Calibration pending: collect the requested independent fresh straight coasts in a clean run; "
                           "keep braking_calibrated false.")
    moving = [r for r in rows if (number(r, "speed") or 0) > 0.5]
    controller_rows = [r for r in moving if number(r, "controller_diagnostics_age_s") is not None
                       and 0 <= r["controller_diagnostics_age_s"] <= 0.15]
    # Older recorder versions used the same diagnostics with this age field.
    reasons = {}
    for r in controller_rows:
        reason = r.get("speed_limit_reason")
        if reason is not None:
            key = str(int(reason))
            reasons[key] = reasons.get(key, 0) + 1
    peak = lambda key, samples: max((r[key] for r in samples if number(r, key) is not None), default=None)
    max_velocity = settings.get("max_velocity_mps")
    cap = (max_velocity if settings.get("braking_calibrated") else
           min(max_velocity, settings.get("commissioning_speed_mps", 2.5))) if max_velocity else None
    throttle_cap, feedforward = actuator.get("max_throttle"), actuator.get("feedforward")
    fresh_output = [r for r in moving if number(r, "throttle_command_age_s") is not None
                    and 0 <= r["throttle_command_age_s"] <= 0.15]
    saturated = sum(r.get("throttle_command", -1) >= throttle_cap - 1e-5 for r in fresh_output) if throttle_cap else 0
    notes = []
    if cap is not None and max_velocity is not None and cap < max_velocity:
        notes.append(f"Commissioning limits planned speed to {cap:g} m/s despite the {max_velocity:g} m/s shared ceiling.")
    if reasons.get("1"):
        notes.append("The planned corner/acceleration/braking profile also limits the target in this recording.")
    if throttle_cap and feedforward:
        notes.append(f"Feedforward alone reaches the {throttle_cap:g} throttle cap at {throttle_cap / feedforward:g} m/s. "
                     "This is not a physical speed limit; inspect upper-cap saturation and tracking before changing throttle.")
    notes.append("A larger speed setting cannot create more straight-line distance or grip. "
                 "Do not enable calibrated braking with a shared ceiling of 20 m/s; begin the next screen at 3.0 m/s.")
    result["speed_limits"] = {
        "configured_ceiling_mps": max_velocity, "effective_planned_ceiling_mps": cap,
        "measured_peak_mps": summary.get("max_speed_mps", peak("speed", moving)),
        "controller_target_peak_mps": peak("target_velocity_mps", controller_rows),
        "actuator_demand_peak_mps": peak("actuator_target_speed_mps", moving),
        "throttle_command_peak": peak("throttle_command", fresh_output),
        "throttle_cap": throttle_cap, "upper_throttle_saturation_samples": saturated,
        "fresh_throttle_samples": len(fresh_output), "fresh_speed_limit_reason_samples": reasons,
        "feedforward_only_cap_speed_mps": throttle_cap / feedforward if throttle_cap and feedforward else None,
        "notes": notes,
    }
    result["limitations"] += (" Sampled command/feedback delays are limited by telemetry rate. "
                               "Coast distance covers only the fitted interval, not a full stop. "
                               "Calibration transfers only to comparable vehicle/surface conditions.")
    return result


def plot(rows, result, filename):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array([r["elapsed_s"] for r in rows])
    values = lambda key: np.array([number(r, key) if number(r, key) is not None else np.nan for r in rows])
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, values("speed"), label="Measured speed")
    axes[0].plot(t, values("target_velocity_mps"), "--", label="Planner target before coast override")
    axes[0].set_ylabel("Speed (m/s)")
    axes[1].plot(t, values("throttle_command"), label="Throttle command")
    axes[1].plot(t, values("throttle"), "--", label="Throttle feedback")
    axes[1].set_ylabel("Normalized throttle")
    dt = np.diff(t)
    acceleration = np.divide(np.diff(values("speed")), dt, out=np.full_like(dt, np.nan), where=(dt > 0) & (dt <= 0.2))
    fresh = values("odom_age_s")
    acceleration[(fresh[:-1] > 0.15) | (fresh[1:] > 0.15) | ~np.isfinite(fresh[:-1]) | ~np.isfinite(fresh[1:])] = np.nan
    axes[2].plot(t[1:], acceleration, label="Measured acceleration (sampled)")
    axes[2].plot(t, values("avlite_acceleration"), "--", label="Requested acceleration")
    axes[2].set_ylabel("Acceleration (m/s²)")
    for ax in axes:
        for attempt in result["attempts"]:
            ax.axvspan(attempt["start_s"], attempt["start_s"] + attempt["observed_coast_duration_s"],
                       alpha=0.15, color="green" if attempt["qualified"] else "orange")
        ax.legend(loc="best")
        ax.grid(alpha=0.25)
    axes[-1].set_xlabel("Recording elapsed time (s)")
    status = "qualified for review" if result["qualified"] else "calibration pending"
    fig.suptitle(f"Vehicle response — {len(result['coast_episodes'])} qualifying coast intervals; {status}")
    fig.tight_layout()
    fig.savefig(filename, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    root = args.directory
    rows = read_samples(root / "telemetry.jsonl")
    if not rows:
        raise ValueError("No telemetry samples to analyze")
    read = lambda name: json.loads((root / name).read_text(encoding="utf-8-sig")) if (root / name).exists() else {}
    actuator = (load_config(root / "config/actuator.yaml").get("avlite_actuator_adapter", {}).get("ros__parameters", {})
                if (root / "config/actuator.yaml").exists() else {})
    result = analyze(rows, read("telemetry.plan.json"), actuator, read("telemetry.summary.json"))
    (root / "response-report.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    plot(rows, result, root / "response-report.png")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
