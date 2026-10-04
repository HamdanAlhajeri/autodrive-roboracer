"""Qualification and graphs for source-rate, deliberately separated coast trials."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .configuration import load_config
from .plot_recording import read_samples


def number(row, name):
    """Read a finite numeric field as a float, returning NaN for missing or invalid data."""
    value = row.get(name)
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else math.nan


def fresh(row):
    """Require recent odometry, actuator command/feedback and controller diagnostics.

    The 0.15-second limit keeps delayed values out of the coast fit.
    """
    return all(0 <= number(row, key + "_age_s") <= 0.15 for key in (
        "odom", "throttle_command", "throttle", "steering", "steering_command",
        "controller_diagnostics"))


def fit_coast(rows):
    """Fit speed against monotonic receive time for one continuous zero-throttle interval.

    Require enough fresh samples, a clear speed decrease and consistent timing. Compare
    travelled position with integrated speed to detect simulator/receive-clock mismatch.
    Return (fit, None) when accepted, or (None, reason) when rejected.
    """
    if len(rows) < 4:
        return None, "fewer than four fresh, zero-throttle samples"
    t = np.array([r["odom_received_monotonic_s"] for r in rows])
    t -= t[0]
    v = np.array([r["speed"] for r in rows])
    dt = np.diff(t)
    if not np.isfinite(t).all() or not np.isfinite(v).all() or np.any(dt <= 0):
        return None, "invalid or repeated measurement times"
    if t[-1] < 0.075 - 1e-8 or np.median(dt) > 0.040001 or np.max(dt) > 0.060001:
        return None, "insufficient source rate/duration or timing gap"
    drop = v[0] - v[-1]
    if drop < 0.15 or np.any(np.diff(v) > 0.03):
        return None, "insufficient or inconsistent speed decrease"
    xy = np.array([[number(r, "x"), number(r, "y")] for r in rows])
    if not np.isfinite(xy).all():
        return None, "position samples missing for simulation-time consistency check"
    position_distance = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    integrated_distance = float(np.sum((v[1:] + v[:-1]) * dt * 0.5))
    # Receive timestamps can remain smooth while Unity physics slows down.
    # Allow finite-sample alignment error, but reject materially different clocks.
    if abs(position_distance - integrated_distance) > max(0.02, 0.2 * integrated_distance):
        return None, "position travel disagrees with speed integrated over receive time"
    slope, intercept = np.polyfit(t, v, 1)
    residual = v - (intercept + slope * t)
    rms = float(np.sqrt(np.mean(residual**2)))
    # A 0.005 m/s noise floor prevents a perfectly quantized fit claiming zero uncertainty.
    sigma = max(0.005, float(np.sqrt(np.sum(residual**2) / (len(t) - 2))))
    slope_se = sigma / math.sqrt(float(np.sum((t - np.mean(t))**2)))
    # Student t(2) 97.5% quantile: conservative also for fits with >4 samples.
    lower = -slope - 4.303 * slope_se
    if slope >= 0 or rms > min(0.1, drop * 0.1) or lower < -slope * 0.5:
        return None, "fit residual or slope uncertainty too large"
    return {"start_s": rows[0]["elapsed_s"], "duration_s": float(t[-1]),
            "samples": len(rows), "median_interval_s": float(np.median(dt)),
            "max_interval_s": float(np.max(dt)), "start_speed_mps": float(v[0]),
            "end_speed_mps": float(v[-1]), "deceleration_mps2": float(-slope),
            "slope_standard_error_mps2": slope_se, "lower_deceleration_mps2": float(lower),
            "fit_rms_mps": rms,
            "interval_distance_m": integrated_distance,
            "position_distance_m": position_distance,
            "position_to_integrated_distance_ratio": position_distance / integrated_distance}, None


def analyze_response(rows, summary, expected_trials=3):
    """Decide whether a complete guided response run supports a braking estimate.

    Check run status and sample ordering, then fit separate fresh, nearly straight coast
    fragments. Count at most one fit per trial and use the most conservative accepted slope
    bound. Only a qualified set of independent trials produces a suggested braking value;
    this does not edit settings.
    """
    run_errors = []
    if (summary.get("status") != "completed" or not summary.get("clean_run")
            or summary.get("stop_reason") != "lap_target"
            or not summary.get("actuator_publication_valid")
            or summary.get("incident_detected") or summary.get("resets", 0)):
        run_errors.append("run did not finish cleanly with fresh actuator publications")
    if not rows or not summary.get("response_capture", {}).get("enabled"):
        run_errors.append("source-rate response capture missing; repeat the test")
    if summary.get("response_capture", {}).get("samples") != len(rows):
        run_errors.append("response capture does not match the completed summary sample count")
    if summary.get("final", {}).get("response_phase") != 4:
        run_errors.append("response experiment did not complete its coast trials")
    previous = None
    for row in rows:
        if (row.get("response_data_version") != 1 or row.get("capture_mode") != "odometry"
                or not all(math.isfinite(number(row, k)) for k in (
                    "odom_received_monotonic_s", "odom_stamp_ns", "sample_sequence", "speed"))
                or number(row, "odom_stamp_ns") <= 0):
            run_errors.append("invalid source-rate sample metadata")
            break
        if previous and any(number(row, k) <= number(previous, k) for k in (
                "odom_received_monotonic_s", "odom_stamp_ns", "sample_sequence")):
            run_errors.append("duplicate/reordered odometry or timestamp discontinuity")
            break
        previous = row
    trials = []
    trial_ids = sorted({int(number(r, "response_trial_id")) for r in rows
                        if 0 < number(r, "response_trial_id") <= 20})
    for trial_id in trial_ids:
        trial_rows = [r for r in rows if r.get("response_trial_id") == trial_id
                      and r.get("response_phase") == 2]
        fragments, fragment = [], []
        for row in trial_rows:
            eligible = (fresh(row) and number(row, "speed") > 0.5
                        and 0 <= number(row, "throttle_command") <= 0.005
                        and 0 <= number(row, "throttle") <= 0.005
                        and abs(number(row, "steering")) <= 0.05
                        and abs(number(row, "steering_command")) <= 0.05)
            if fragment and (number(row, "odom_received_monotonic_s")
                             - fragment[-1]["odom_received_monotonic_s"] > 0.06 + 1e-8):
                fragments.append(fragment)
                fragment = []
            if eligible:
                fragment.append(row)
            elif fragment:
                fragments.append(fragment)
                fragment = []
        if fragment:
            fragments.append(fragment)
        fits, reasons = [], []
        for fragment in fragments:
            fit, reason = fit_coast(fragment)
            if fit:
                fits.append(fit)
            else:
                reasons.append(reason)
        best = min(fits, key=lambda f: f["lower_deceleration_mps2"]) if fits else None
        feedback = [r for r in trial_rows if fresh(r)
                    and 0 <= number(r, "throttle_command") <= 0.005
                    and 0 <= number(r, "throttle") <= 0.005]
        trials.append({"trial_id": trial_id, "accepted": best is not None, "fit": best,
                       "rejections": sorted(set(reasons)) if reasons else (
                           [] if best else ["no fresh straight zero-throttle interval"]),
                       "sampled_coast_to_zero_feedback_s": (
                           feedback[0]["elapsed_s"] - trial_rows[0]["elapsed_s"]
                           if feedback and trial_rows else None)})
    accepted = [trial["fit"] for trial in trials if trial["accepted"]]
    qualified = not run_errors and len(accepted) >= max(3, expected_trials)
    # Descriptive powered response; never used to infer braking or lift a speed cap.
    rises, group = [], []
    for row in [*rows, {}]:
        powered = (fresh(row) and number(row, "response_phase") in (1, 3)
                   and number(row, "throttle") > 0.005
                   and number(row, "throttle_command") > 0.005)
        continuous = (not group or 0 < number(row, "odom_received_monotonic_s")
                      - group[-1]["odom_received_monotonic_s"] <= 0.06)
        if group and (not powered or not continuous):
            t = np.array([r["odom_received_monotonic_s"] for r in group])
            v = np.array([r["speed"] for r in group])
            if len(t) >= 4 and t[-1] - t[0] >= 0.15 and v[-1] - v[0] >= 0.15:
                slope = float(np.polyfit(t - t[0], v, 1)[0])
                rises.append({"start_s": group[0]["elapsed_s"],
                              "duration_s": float(t[-1] - t[0]),
                              "acceleration_mps2": slope})
            group = []
        if powered:
            group.append(row)
    intervals = np.diff([number(r, "odom_received_monotonic_s") for r in rows])
    valid_intervals = intervals[np.isfinite(intervals) & (intervals > 0)]
    timing = {"samples": len(rows), "gaps_over_150ms": int(np.sum(valid_intervals > 0.15))}
    if len(valid_intervals):
        timing.update(median_interval_s=float(np.median(valid_intervals)),
                      p95_interval_s=float(np.percentile(valid_intervals, 95)),
                      max_interval_s=float(np.max(valid_intervals)))
    return {"qualification_version": 2, "qualified": qualified,
            "run_rejections": run_errors, "trials": trials,
            "odometry_receive_timing": timing,
            "accepted_trials": len(accepted), "required_trials": max(3, expected_trials),
            "suggested_braking_deceleration_mps2": (
                0.8 * min(f["lower_deceleration_mps2"] for f in accepted) if qualified else None),
            "powered_acceleration_intervals": rises,
            "method": "80% of the smallest lower slope bound across independent trials",
            "limitations": "Monotonic receiver timing, not physics timestamps; sampled delays "
                           "include delivery/diagnostic latency. Interval distance is not a "
                           "full stopping distance. No automatic configuration changes."}


def speed_limit_diagnostics(rows, plan, actuator, summary):
    """Explain the effective speed ceiling and observed throttle saturation."""
    settings = plan.get("settings", {})

    def optional_number(row, key):
        value = number(row, key)
        return value if math.isfinite(value) else None

    moving = [r for r in rows if (optional_number(r, "speed") or 0) > 0.5]
    controller_rows = [r for r in moving
                       if 0 <= number(r, "controller_diagnostics_age_s") <= 0.15]
    # Older recorder versions used the same diagnostics with this age field.
    reasons = {}
    for r in controller_rows:
        reason = optional_number(r, "speed_limit_reason")
        if reason is not None:
            key = str(int(reason))
            reasons[key] = reasons.get(key, 0) + 1

    def peak(key, samples):
        return max((r[key] for r in samples if optional_number(r, key) is not None), default=None)

    max_velocity = settings.get("max_velocity_mps")
    cap = max_velocity
    if max_velocity is not None and not settings.get("braking_calibrated"):
        cap = min(max_velocity, settings.get("commissioning_speed_mps", 2.5))
    throttle_cap, feedforward = actuator.get("max_throttle"), actuator.get("feedforward")
    fresh_output = [r for r in moving if optional_number(r, "throttle_command_age_s") is not None
                    and 0 <= r["throttle_command_age_s"] <= 0.15]
    saturated = (sum(number(r, "throttle_command") >= throttle_cap - 1e-5 for r in fresh_output)
                 if throttle_cap else 0)
    notes = []
    if cap is not None and max_velocity is not None and cap < max_velocity:
        notes.append(f"Commissioning limits planned speed to {cap:g} m/s despite the "
                     f"{max_velocity:g} m/s shared ceiling.")
    if reasons.get("1"):
        notes.append("The planned corner/acceleration/braking profile also limits the target "
                     "in this recording.")
    if throttle_cap and feedforward:
        notes.append(f"Feedforward alone reaches the {throttle_cap:g} throttle cap at "
                     f"{throttle_cap / feedforward:g} m/s. This is not a physical speed limit; "
                     "inspect upper-cap saturation and tracking before changing throttle.")
    notes.append("A larger speed setting cannot create more straight-line distance or grip. "
                 "Do not enable calibrated braking with a shared ceiling of 20 m/s; "
                 "begin the next screen at 3.0 m/s.")
    return {
        "configured_ceiling_mps": max_velocity, "effective_planned_ceiling_mps": cap,
        "measured_peak_mps": summary.get("max_speed_mps", peak("speed", moving)),
        "controller_target_peak_mps": peak("target_velocity_mps", controller_rows),
        "actuator_demand_peak_mps": peak("actuator_target_speed_mps", moving),
        "throttle_command_peak": peak("throttle_command", fresh_output),
        "throttle_cap": throttle_cap, "upper_throttle_saturation_samples": saturated,
        "fresh_throttle_samples": len(fresh_output), "fresh_speed_limit_reason_samples": reasons,
        "feedforward_only_cap_speed_mps": (
            throttle_cap / feedforward if throttle_cap and feedforward else None),
        "notes": notes,
    }


def main():
    """Read a recording folder and write the response qualification report and graph.

    Use its saved effective configuration to determine the requested trial count. Missing or
    truncated inputs prevent qualification but still produce a report explaining the
    problem.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    directory = args.directory
    source = directory / "telemetry.response.jsonl"
    rows, input_errors = [], []
    if source.exists():
        for line in source.read_text().splitlines():
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("sample is not an object")
                rows.append(row)
            except ValueError:
                input_errors.append("response capture contains an incomplete/invalid JSON line")
                break

    def load_snapshot(name):
        """Load one saved JSON object, recording an input error if it is missing or malformed."""
        try:
            data = json.loads((directory / name).read_text())
            if not isinstance(data, dict):
                raise ValueError("snapshot is not an object")
            return data
        except (OSError, ValueError):
            input_errors.append("missing or invalid " + name)
            return {}

    summary = load_snapshot("telemetry.summary.json")
    config = load_snapshot("telemetry.planning-config.json")
    expected = config.get("response_test", {}).get("trials", 3)
    result = analyze_response(rows, summary, expected)
    if not config.get("response_test"):
        result["run_rejections"].append("effective response-test configuration missing")
    result["run_rejections"].extend(input_errors)
    if result["run_rejections"]:
        result["qualified"] = False
        result["suggested_braking_deceleration_mps2"] = None
    plan_path = directory / "telemetry.plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8-sig")) if plan_path.exists() else {}
    actuator_path = directory / "config/actuator.yaml"
    actuator = (load_config(actuator_path).get("avlite_actuator_adapter", {})
                .get("ros__parameters", {}) if actuator_path.exists() else {})
    telemetry_path = directory / "telemetry.jsonl"
    telemetry = read_samples(telemetry_path) if telemetry_path.exists() else rows
    result["speed_limits"] = speed_limit_diagnostics(telemetry, plan, actuator, summary)
    (directory / "response-report.json").write_text(json.dumps(result, indent=2) + "\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True, layout="constrained")
    t = [number(r, "elapsed_s") for r in rows]
    for key in ("speed", "target_velocity_mps"):
        axes[0].plot(t, [number(r, key) for r in rows], label=key)
    for trial in result["trials"]:
        if trial["fit"]:
            fit = trial["fit"]
            axes[0].axvspan(fit["start_s"], fit["start_s"] + fit["duration_s"], alpha=0.2)
    for key in ("throttle", "throttle_command"):
        axes[1].plot(t, [number(r, key) for r in rows], label=key)
    axes[2].step(t, [number(r, "response_phase") for r in rows], label="response phase")
    axes[3].plot(t[1:], 1000 * np.diff([number(r, "odom_received_monotonic_s") for r in rows]),
                 label="odometry receive interval")
    axes[0].set_ylabel("Speed (m/s)")
    axes[1].set_ylabel("Normalized throttle")
    axes[2].set_ylabel("Experiment")
    axes[2].set_yticks([1, 2, 3, 4, 5], ["cruise", "coast", "recover", "done", "abort"])
    axes[3].set_ylabel("Receive interval (ms)")
    axes[3].set_xlabel("Elapsed monotonic time (s)")
    for ax in axes:
        ax.grid(alpha=0.25)
        ax.legend()
    fig.suptitle("Response: " + ("qualified" if result["qualified"] else "NOT qualified"))
    fig.savefig(directory / "response-report.png", dpi=150)
    plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
