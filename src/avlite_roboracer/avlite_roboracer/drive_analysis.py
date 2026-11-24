"""Summarize a floor run from its SLAM path and probe log: where the car stalled, and how
often the steering sat at its limit.

Inputs are the two CSV files saved in the car-day folders:
- slam trace: stamp_s,x,y,yaw,...   (the SLAM pose over time)
- probe log:  t_s,topic,value       (topics drive, erpm, ... sampled at the VESC)

Both files are time-aligned by their own first row, which is good enough to compare a
commanded speed with the speed the car really made. No ROS here.
"""

import argparse
import csv
import math


def load_path(path):
    with open(path, newline="") as handle:
        rows = [(float(r["stamp_s"]), float(r["x"]), float(r["y"]), float(r["yaw"]))
                for r in csv.DictReader(handle)]
    t0 = rows[0][0]
    return [(t - t0, x, y, yaw) for t, x, y, yaw in rows]


def load_probe(path, topic="drive"):
    with open(path, newline="") as handle:
        return [(float(r["t_s"]), float(r["value"])) for r in csv.DictReader(handle)
                if r["topic"] == topic]


def speed_and_radius(path, span=5):
    """Per-sample (t, speed m/s, turn radius m or None) over a span of samples."""
    out = []
    for i in range(span, len(path)):
        a, b = path[i - span], path[i]
        dt = b[0] - a[0]
        if dt <= 0:
            continue
        speed = math.hypot(b[1] - a[1], b[2] - a[2]) / dt
        turn = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])) / dt
        out.append((b[0], speed, speed / abs(turn) if abs(turn) > 0.02 else None))
    return out


def command_at(commands, t):
    value = None
    for stamp, speed in commands:
        if stamp > t:
            break
        value = speed
    return value


def stalls(samples, commands, min_command=0.3, ratio=0.25, min_s=3.0):
    """Intervals (start, end, command, mean speed) where the car was commanded but slow."""
    found, start, speeds, cmd = [], None, [], 0.0
    for t, speed, _ in samples:
        command = command_at(commands, t) or 0.0
        slow = command >= min_command and speed < ratio * command
        if slow and start is None:
            start, speeds, cmd = t, [], command
        if slow:
            speeds.append(speed)
        elif start is not None:
            if t - start >= min_s:
                found.append((start, t, cmd, sum(speeds) / len(speeds)))
            start = None
    if start is not None and samples[-1][0] - start >= min_s:
        found.append((start, samples[-1][0], cmd, sum(speeds) / len(speeds)))
    return found


def at_minimum_radius(samples, min_radius_m, tolerance=0.1):
    """Fraction of moving samples whose turn radius is within tolerance of the minimum."""
    moving = [r for _, speed, r in samples if speed > 0.1 and r is not None]
    if not moving:
        return 0.0
    return sum(r <= min_radius_m * (1 + tolerance) for r in moving) / len(moving)


def report(slam_csv, probe_csv, min_radius_m=1.4):
    samples = speed_and_radius(load_path(slam_csv))
    commands = load_probe(probe_csv)
    lines = [f"{len(samples)} path samples over {samples[-1][0]:.0f} s"]
    for start, end, cmd, mean in stalls(samples, commands):
        lines.append(f"stall {start:.0f}-{end:.0f} s: commanded {cmd:.2f} m/s, "
                     f"moved {mean:.2f} m/s on average")
    if len(lines) == 1:
        lines.append("no stall: the car followed the speed command")
    lines.append(f"{at_minimum_radius(samples, min_radius_m):.0%} of the moving samples were "
                 f"turning at about the minimum radius ({min_radius_m} m)")
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("slam_csv")
    parser.add_argument("probe_csv")
    parser.add_argument("--min-radius", type=float, default=1.4)
    args = parser.parse_args(argv)
    print(report(args.slam_csv, args.probe_csv, args.min_radius))


if __name__ == "__main__":
    main()
