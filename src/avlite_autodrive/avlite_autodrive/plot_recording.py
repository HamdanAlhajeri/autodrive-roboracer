"""Export a recorded run as CSV and a standalone PNG for debugging."""

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path


def read_samples(path):
    rows = []
    with Path(path).open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            elapsed = row.get("elapsed_s") if isinstance(row, dict) else None
            if not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed):
                raise ValueError(f"{path}:{line_number}: missing or invalid elapsed_s")
            rows.append(row)
    if not rows:
        raise ValueError(f"{path}: recording is empty")
    return rows


def series(rows, key, freshness_key=None):
    values = []
    for row in rows:
        value = row.get(key)
        age = row.get(freshness_key) if freshness_key else None
        if (
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or (age is not None and age > 0.5)
        ):
            value = math.nan
        values.append(value)
    return values


def export_csv(rows, path):
    columns = list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def track_map_points(track_map):
    """Validate a world-frame observed outline before mixing it with the trajectory."""
    if track_map is None:
        return []
    if not isinstance(track_map, dict):
        raise ValueError("track map must be a JSON object")
    if isinstance(track_map.get("version"), bool) or track_map.get("version") != 1:
        raise ValueError("track map version must be 1")
    if track_map.get("frame_id") != "world" or track_map.get("units") != "m":
        raise ValueError("track map must use frame_id 'world' and units 'm'")
    points = track_map.get("points")
    if not isinstance(points, list):
        raise ValueError("track map points must be an array of [x, y] pairs")
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2 or not all(
            not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) for value in point
        ):
            raise ValueError("track map points must contain only finite [x, y] pairs")
    return points


def load_track_map(recording, explicit_path=None):
    """Use the selected sidecar only; never invent a track for older recordings."""
    path = Path(explicit_path) if explicit_path is not None else Path(recording).with_suffix(
        ".track.json"
    )
    try:
        track_map = json.loads(path.read_text(encoding="utf-8"))
        if not track_map_points(track_map):
            raise ValueError("map contains no observed points")
    except (OSError, ValueError) as error:
        print(f"Track overlay unavailable: {path}: {error}. Plotting trajectory only.", flush=True)
        return None
    print(f"Observed track map: {path} ({len(track_map['points'])} LiDAR hits)", flush=True)
    return track_map


def plot_track_overlay(axis, track_map):
    points = track_map_points(track_map)
    if points:
        axis.scatter([point[0] for point in points], [point[1] for point in points],
                     color="#737d89", s=3, alpha=0.5, linewidths=0, zorder=0,
                     label="Observed track / obstacles (LiDAR)")


def plot_path_events(axis, rows, x, y):
    """Locate incidents at the last fresh pre-event pose, never the reset destination."""
    labelled = set()
    for i in range(1, len(rows)):
        if not math.isfinite(x[i - 1]) or not math.isfinite(y[i - 1]):
            continue
        events = []
        for key, label, marker, color in (
            ("collision_count", "Before collision", "D", "#b2182b"),
            ("resets", "Before reset", "s", "#c66b00"),
        ):
            before, after = rows[i - 1].get(key), rows[i].get(key)
            if not all(isinstance(value, (int, float)) and math.isfinite(value)
                       for value in (before, after)) or after <= before:
                continue
            axis.scatter(x[i - 1], y[i - 1], marker=marker, s=65, facecolors="none",
                         edgecolors=color, linewidths=1.6, zorder=4,
                         label=label if key not in labelled else "_nolegend_")
            labelled.add(key)
            events.append(label.removeprefix("Before "))
        if events:
            axis.annotate(f"{' / '.join(events)} {rows[i]['elapsed_s']:.1f} s",
                          (x[i - 1], y[i - 1]), xytext=(7, 6), textcoords="offset points",
                          fontsize=8, color="#6c2430", zorder=5)


def plot_speed_targets(axis, rows, elapsed):
    for key, label, age in (
        ("planned_speed_mps", "Planned speed", "controller_diagnostics_age_s"),
        ("target_velocity_mps", "Controller target", "controller_diagnostics_age_s"),
        ("actuator_target_speed_mps", "Actuator demand", "actuator_diagnostics_age_s"),
    ):
        values = series(rows, key, age)
        if any(math.isfinite(value) for value in values):
            axis.plot(elapsed, values, label=label, linewidth=1.1, linestyle="--")


def plot_run(rows, path, title="AutoDRIVE telemetry", track_map=None):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed = series(rows, "elapsed_s")
    speed = series(rows, "speed", "odom_age_s")
    if not any(math.isfinite(value) for value in speed):
        raise ValueError(
            "No odometry recorded; this file has no speed or position data to plot. "
            "Check the simulator connection and rerun recording after any bridge restart."
        )
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), layout="constrained")
    start = rows[0].get("timestamp_utc", "")
    fig.suptitle(title + (f"\nStarted {start}" if start else ""), fontsize=14)

    def line(axis, key, label, age_key, **kwargs):
        axis.plot(elapsed, series(rows, key, age_key), label=label, linewidth=1.4, **kwargs)

    speed_ax, throttle_ax, steering_ax, accel_ax, path_ax, event_ax = axes.flat
    speed_ax.plot(elapsed, speed, color="#1778b8", label="Measured forward speed")
    plot_speed_targets(speed_ax, rows, elapsed)
    speed_ax.set(title="Measured speed", ylabel="m/s")
    line(throttle_ax, "throttle_command", "Command", "throttle_command_age_s")
    line(throttle_ax, "throttle", "Feedback", "throttle_age_s", alpha=0.8)
    throttle_ax.set(title="Throttle", ylabel="Normalized command (0 to 1)")
    line(steering_ax, "avlite_steer_rad", "AVLite command", "avlite_command_age_s")
    line(steering_ax, "steering", "Feedback", "steering_age_s", alpha=0.8)
    steering_ax.set(title="Steering", ylabel="Radians")
    line(accel_ax, "avlite_acceleration", "AVLite command", "avlite_command_age_s")
    accel_ax.set(title="Requested acceleration", ylabel="m/s²")

    x, y = series(rows, "x", "odom_age_s"), series(rows, "y", "odom_age_s")
    # Break the trajectory at resets rather than drawing a jump across the track.
    path_x, path_y = x.copy(), y.copy()
    for i in range(1, len(rows)):
        if rows[i].get("resets", 0) != rows[i - 1].get("resets", 0):
            path_x[i] = path_y[i] = math.nan
    plot_track_overlay(path_ax, track_map)
    path_ax.plot(path_x, path_y, color="#b8c4ce", linewidth=0.8, zorder=2)
    points = path_ax.scatter(x, y, c=speed, s=8, cmap="viridis", zorder=3)
    plot_path_events(path_ax, rows, x, y)
    fig.colorbar(points, ax=path_ax, label="Forward speed (m/s)")
    path_ax.set(title="Vehicle path", xlabel="X (m)", ylabel="Y (m)", aspect="equal")
    if path_ax.get_legend_handles_labels()[0]:
        path_ax.legend(loc="best", fontsize=7)
    for key, label in (("lap_count", "Laps"), ("collision_count", "Collisions"),
                       ("resets", "Observed resets")):
        event_ax.step(elapsed, series(rows, key), where="post", label=label)
    event_ax.set(title="Simulator counters", ylabel="Count")

    for axis in (speed_ax, throttle_ax, steering_ax, accel_ax, event_ax):
        axis.set_xlabel("Elapsed time (s)")
        axis.legend(loc="best", fontsize=8)
    for axis in axes.flat:
        axis.grid(alpha=0.2)
    try:
        fig.savefig(path, dpi=160)
    finally:
        plt.close(fig)


def lap_report_title(rows, summary=None):
    """Describe observed counters, without claiming a complete start-to-finish lap."""
    summary = summary or {}
    details = []
    for key, label in (("lap_count", "lap count"), ("collision_count", "collision count")):
        samples = [value for value in series(rows, key) if math.isfinite(value)]
        first = summary.get("initial", {}).get(key, samples[0] if samples else None)
        last = summary.get("final", {}).get(key, samples[-1] if samples else None)
        if not all(isinstance(value, (int, float)) and math.isfinite(value)
                   for value in (first, last)):
            details.append(f"{label} unavailable")
        elif key == "collision_count" and last >= first and not summary.get("resets", 0) \
                and all(b >= a for a, b in zip(samples, samples[1:])) \
                and not any(value > 0 for value in series(rows, "resets")):
            count = last - first
            details.append(f"{count:g} collision{'s' if count != 1 else ''}")
        else:
            details.append(f"{label} {first:g} → {last:g}")
    resets = max([summary.get("resets", 0)] + [
        value for value in series(rows, "resets") if math.isfinite(value)
    ])
    if resets:
        details.append(f"{resets:g} reset{'s' if resets != 1 else ''} observed")
    if summary.get("status") in ("failed", "interrupted"):
        details.append(f"recording {summary['status']}")
    title = "AVLite / AutoDRIVE practice track — " + ", ".join(details)
    timestamp = summary.get("started_utc") or next(
        (row["timestamp_utc"] for row in rows if row.get("timestamp_utc")), None
    )
    if timestamp:
        try:
            started = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
            if started.tzinfo is not None:
                started = started.astimezone(timezone.utc)
            title += "\n" + started.strftime("%d %B %Y") + " (UTC)"
        except ValueError:
            pass
    return title


def plot_lap_report(rows, path, summary=None, speed_ceiling=None, track_map=None):
    """Plot measured trajectory and speed side by side, keeping gaps in real data."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed = series(rows, "elapsed_s")
    speed = series(rows, "speed", "odom_age_s")
    x = series(rows, "x", "odom_age_s")
    y = series(rows, "y", "odom_age_s")
    valid = [i for i in range(len(rows))
             if all(math.isfinite(values[i]) for values in (x, y, speed))]
    if not valid:
        raise ValueError("No valid odometry positions recorded; cannot plot a vehicle path.")
    if speed_ceiling is not None and (
        isinstance(speed_ceiling, bool) or not isinstance(speed_ceiling, (int, float))
        or not math.isfinite(speed_ceiling) or speed_ceiling <= 0
    ):
        raise ValueError("Speed demand ceiling must be a finite positive number.")
    # A reset or missing coordinate must break the line instead of joining locations.
    path_x, path_y = x.copy(), y.copy()
    for i in range(len(rows)):
        if not math.isfinite(x[i]) or not math.isfinite(y[i]) or (
            i and rows[i].get("resets", 0) != rows[i - 1].get("resets", 0)
        ):
            path_x[i] = path_y[i] = math.nan

    with plt.rc_context({"font.size": 12, "axes.titlesize": 15, "figure.titlesize": 17}):
        fig, (path_ax, speed_ax) = plt.subplots(1, 2, figsize=(14, 7), layout="constrained")
        try:
            fig.suptitle(lap_report_title(rows, summary))
            plot_track_overlay(path_ax, track_map)
            path_ax.plot(path_x, path_y, color="#0668e8", linewidth=2, zorder=2)
            first, last = valid[0], valid[-1]
            path_ax.scatter(x[first], y[first], color="#188238", s=70,
                            label="Recording start", zorder=3)
            path_ax.scatter(x[last], y[last], color="#dc1936", s=80, marker="x",
                            linewidths=2, label="Recording end", zorder=3)
            plot_path_events(path_ax, rows, x, y)
            path_ax.set(title="Measured vehicle trajectory", xlabel="World x (m)",
                        ylabel="World y (m)", aspect="equal")
            path_ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1), fontsize=8,
                           borderaxespad=0)
            speed_ax.plot(elapsed, speed, color="#0668e8", linewidth=1.6, label="Odometry")
            plot_speed_targets(speed_ax, rows, elapsed)
            if speed_ceiling is not None:
                speed_ax.axhline(speed_ceiling, color="#737d89", linestyle="--",
                                 linewidth=1.7, label="Speed demand ceiling (saved settings)")
            speed_ax.set(title="Measured speed", xlabel="Recording elapsed time (s)",
                         ylabel="Longitudinal speed (m/s)")
            # Keep zero visible while still showing negative speed or overshoot.
            low, high = speed_ax.get_ylim()
            margin = 0.02 * max(abs(low), abs(high), 0.1)
            speed_ax.set_ylim(min(low, -margin), max(high, margin))
            speed_ax.legend(loc="best")
            for axis in (path_ax, speed_ax):
                axis.grid(alpha=0.2)
            fig.savefig(path, dpi=160)
        finally:
            plt.close(fig)


def measured_acceleration(rows):
    """Differentiate fresh samples without interpreting a reset as physical braking."""
    speed = series(rows, "speed", "odom_age_s")
    acceleration = [math.nan] * len(rows)
    for i in range(1, len(rows)):
        dt = rows[i]["elapsed_s"] - rows[i - 1]["elapsed_s"]
        if (0 < dt <= 0.5 and math.isfinite(speed[i]) and math.isfinite(speed[i - 1])
                and rows[i].get("resets", 0) == rows[i - 1].get("resets", 0)
                and rows[i].get("collision_count") == rows[i - 1].get("collision_count")):
            acceleration[i] = (speed[i] - speed[i - 1]) / dt
    return acceleration


def plot_control_report(rows, path):
    """Diagnostics for corner entry and real throttle/deceleration response."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed = series(rows, "elapsed_s")
    diagnostics_age = "controller_diagnostics_age_s"
    preview_keys = ("gap_preview_requested_m", "gap_preview_selected_m")
    bearing_keys = ("target_bearing_raw_rad", "target_bearing_rad")

    def has_values(keys):
        return any(math.isfinite(value) for key in keys
                   for value in series(rows, key, diagnostics_age))

    show_preview = has_values(preview_keys)
    show_bearing = has_values(bearing_keys)
    extra_panels = int(show_preview) + int(show_bearing)
    fig, axes = plt.subplots(4 if extra_panels else 3, 2,
                             figsize=(14, 14 if extra_panels else 11), layout="constrained")
    plot_axes = list(axes.flat)
    speed_ax, space_ax, accel_ax, throttle_ax, steer_ax, timing_ax = plot_axes[:6]
    if extra_panels == 1:
        fig.delaxes(plot_axes.pop())
    fig.suptitle("Corner-entry diagnostics", fontsize=16)

    def line(axis, key, label, age):
        axis.plot(elapsed, series(rows, key, age), label=label, linewidth=1.2)

    line(speed_ax, "speed", "Measured", "odom_age_s")
    plot_speed_targets(speed_ax, rows, elapsed)
    line(speed_ax, "curvature_speed_limit_mps", "Curvature limit", "controller_diagnostics_age_s")
    line(speed_ax, "clearance_speed_limit_mps", "Clearance limit", "controller_diagnostics_age_s")
    speed_ax.set(title="Speed decision", ylabel="m/s")
    line(space_ax, "lookahead_m", "Steering lookahead", "controller_diagnostics_age_s")
    line(space_ax, "clearance_m", "Observed corridor clearance", "controller_diagnostics_age_s")
    space_ax.set(title="Lookahead and clearance", ylabel="m")
    line(accel_ax, "avlite_acceleration", "Requested", "avlite_command_age_s")
    accel_ax.plot(elapsed, measured_acceleration(rows), label="Measured delta-v / delta-t",
                  linewidth=0.9, alpha=0.8)
    accel_ax.set(title="Requested and measured acceleration", ylabel="m/s squared")
    line(throttle_ax, "throttle_command", "Command", "throttle_command_age_s")
    line(throttle_ax, "throttle", "Feedback", "throttle_age_s")
    throttle_ax.set(title="Throttle and braking", ylabel="Normalized throttle")
    line(steer_ax, "avlite_steer_rad", "Command", "avlite_command_age_s")
    line(steer_ax, "steering", "Feedback", "steering_age_s")
    steer_ax.set(title="Steering", ylabel="Radians")
    for axis, flag, value, label, age in (
        (throttle_ax, "braking_requested", "throttle_command", "Braking request",
         "actuator_diagnostics_age_s"),
        (throttle_ax, "throttle_saturated", "throttle_command", "Throttle clipped",
         "actuator_diagnostics_age_s"),
        (steer_ax, "steering_saturated", "avlite_steer_rad", "Steering limited",
         "controller_diagnostics_age_s"),
    ):
        flags = series(rows, flag, age)
        values = series(rows, value)
        indices = [i for i, flag_value in enumerate(flags) if flag_value == 1]
        axis.scatter([elapsed[i] for i in indices], [values[i] for i in indices],
                     marker="x", s=18, label=label)
    line(timing_ax, "controller_loop_dt_s", "Controller interval", "controller_diagnostics_age_s")
    line(timing_ax, "controller_step_time_s", "Work time", "controller_diagnostics_age_s")
    line(timing_ax, "controller_lidar_age_s", "LiDAR age", "controller_diagnostics_age_s")
    timing_ax.axhline(0.05, color="gray", linestyle="--", label="20 Hz period")
    timing_ax.set(title="Timing and LiDAR freshness", ylabel="Seconds")
    if show_preview:
        preview_ax = plot_axes[6]
        line(preview_ax, "gap_preview_requested_m", "Requested gap preview", diagnostics_age)
        line(preview_ax, "gap_preview_selected_m", "Selected gap preview", diagnostics_age)
        line(preview_ax, "lookahead_m", "Steering pursuit distance", diagnostics_age)
        selected = series(rows, "gap_preview_selected_m", diagnostics_age)
        fallback = series(rows, "gap_preview_fallback", diagnostics_age)
        indices = [i for i, flag in enumerate(fallback)
                   if flag == 1 and math.isfinite(selected[i])]
        preview_ax.scatter([elapsed[i] for i in indices], [selected[i] for i in indices],
                           marker="x", s=25, color="#c66b00", label="Shorter-preview fallback")
        preview_ax.set(title="Gap preview and steering pursuit distance", ylabel="m")
    if show_bearing:
        bearing_ax = plot_axes[6 + int(show_preview)]
        line(bearing_ax, "target_bearing_raw_rad", "Raw gap bearing", diagnostics_age)
        line(bearing_ax, "target_bearing_rad", "Filtered gap bearing", diagnostics_age)
        bearing_ax.set(title="Chosen gap direction in vehicle frame",
                       ylabel="Bearing (radians)")
    for axis in plot_axes:
        for i in range(1, len(rows)):
            if rows[i].get("collision_count", 0) != rows[i - 1].get("collision_count", 0):
                axis.axvline(elapsed[i], color="#dc1936", linestyle=":", alpha=0.7)
            if rows[i].get("resets", 0) != rows[i - 1].get("resets", 0):
                axis.axvline(elapsed[i], color="gray", linestyle="--", alpha=0.5)
        axis.set_xlabel("Recording elapsed time (s)")
        axis.legend(loc="best", fontsize=8)
        axis.grid(alpha=0.2)
    try:
        fig.savefig(path, dpi=160)
    finally:
        plt.close(fig)


def saved_speed_ceiling(recording):
    """Read only this run's saved profile, never today's potentially changed settings."""
    profile = recording.parent / "config" / "avlite.yaml"
    if not profile.exists():
        return None
    from .configuration import load_config

    return load_config(profile)["c30_control"]["c32_ego_max_velocity"]


def plot_planned_report(rows, artifact, output):
    """Compare the executed path and speeds in closed-track distance coordinates."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from .race_map import ClosedPath

    path = ClosedPath(artifact["path"])
    fig, axes = plt.subplots(2, 2, figsize=(13, 10), layout="constrained")
    route_ax, speed_ax, error_ax, reason_ax = axes.flat
    for key, label in (("LeftBound", "Left boundary"), ("RightBound", "Right boundary")):
        xy = np.asarray(artifact["map"][key])
        route_ax.plot(*np.vstack([xy, xy[0]]).T, linewidth=1, label=label)
    route_ax.plot(*np.vstack([path.points, path.points[0]]).T, "k--", label="Planned line")
    route_ax.scatter(series(rows, "x", "odom_age_s"), series(rows, "y", "odom_age_s"),
                     c=series(rows, "speed", "odom_age_s"), s=5, label="Driven path")
    route_ax.set(aspect="equal", xlabel="World X (m)", ylabel="World Y (m)")
    speed_ax.plot(artifact["distance_m"], artifact["velocity"], color="black",
                  label="Planned speed")
    progress = series(rows, "path_progress_m", "controller_diagnostics_age_s")
    for key, label, age in (("speed", "Actual speed", "odom_age_s"),
                            ("target_velocity_mps", "Controller target",
                             "controller_diagnostics_age_s")):
        speed_ax.scatter(progress, series(rows, key, age), s=5, label=label)
    speed_ax.set(xlabel="Distance around planned lap (m)", ylabel="m/s")
    elapsed = series(rows, "elapsed_s")
    error_ax.plot(elapsed, series(rows, "path_deviation_m", "controller_diagnostics_age_s"))
    error_ax.set(xlabel="Elapsed time (s)", ylabel="Path deviation (m)")
    reason_ax.step(elapsed, series(rows, "speed_limit_reason", "controller_diagnostics_age_s"),
                   where="post")
    reason_ax.set(yticks=range(7), yticklabels=[
        "Speed ceiling", "Corner/braking", "Obstacle", "Invalid pose", "Stale scan",
        "Commissioning", "Invalid plan"], xlabel="Elapsed time (s)")
    route_ax.legend(fontsize=8)
    speed_ax.legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=0.2)
    fig.suptitle("Planned versus measured driving")
    try:
        fig.savefig(output, dpi=160)
    finally:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="JSONL file produced by record_lap")
    parser.add_argument("--title", help="Run label above the six-panel debugging graph")
    parser.add_argument("--lap-report", action="store_true",
                        help="Also save a .lap.png trajectory and speed report")
    parser.add_argument("--track-map", type=Path,
                        help="Reuse an observed .track.json map from the same simulator world; "
                             "defaults to this recording's sidecar")
    args = parser.parse_args()
    rows = read_samples(args.recording)
    track_map = load_track_map(args.recording, args.track_map)
    csv_path = args.recording.with_suffix(".csv")
    png_path = args.recording.with_suffix(".png")
    export_csv(rows, csv_path)
    plot_run(rows, png_path, title=args.title or args.recording.parent.name, track_map=track_map)
    print(f"CSV: {csv_path}\nGraph: {png_path}", flush=True)
    if args.lap_report:
        summary_path = args.recording.with_suffix(".summary.json")
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else None
        report_path = args.recording.with_suffix(".lap.png")
        plot_lap_report(rows, report_path, summary=summary,
                        speed_ceiling=saved_speed_ceiling(args.recording), track_map=track_map)
        print(f"Lap graph: {report_path}", flush=True)
    if any("target_velocity_mps" in row or "actuator_target_speed_mps" in row for row in rows):
        control_path = args.recording.with_suffix(".control.png")
        plot_control_report(rows, control_path)
        print(f"Control diagnostics: {control_path}", flush=True)
    artifact_path = args.recording.with_suffix(".plan.json")
    if artifact_path.exists():
        planned_path = args.recording.with_suffix(".planned.png")
        plot_planned_report(rows, json.loads(artifact_path.read_text()), planned_path)
        print(f"Planned-driving diagnostics: {planned_path}", flush=True)


if __name__ == "__main__":
    main()
