"""Export a recorded run as CSV and a standalone PNG for debugging."""

import argparse
import csv
import json
import math
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


def plot_run(rows, path, title="AutoDRIVE telemetry"):
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
    path_ax.plot(path_x, path_y, color="#b8c4ce", linewidth=0.8)
    points = path_ax.scatter(x, y, c=speed, s=8, cmap="viridis")
    fig.colorbar(points, ax=path_ax, label="Forward speed (m/s)")
    path_ax.set(title="Vehicle path", xlabel="X (m)", ylabel="Y (m)", aspect="equal")
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="JSONL file produced by record_lap")
    parser.add_argument("--title", help="Run label to show above the graphs")
    args = parser.parse_args()
    rows = read_samples(args.recording)
    csv_path = args.recording.with_suffix(".csv")
    png_path = args.recording.with_suffix(".png")
    export_csv(rows, csv_path)
    plot_run(rows, png_path, title=args.title or args.recording.parent.name)
    print(f"CSV: {csv_path}\nGraph: {png_path}", flush=True)


if __name__ == "__main__":
    main()
