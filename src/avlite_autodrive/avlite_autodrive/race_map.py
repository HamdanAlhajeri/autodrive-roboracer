"""Closed-track geometry and reference-assisted RaceMap preparation (no ROS)."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d
from shapely.geometry import LineString, Polygon

from .plot_recording import read_samples, track_map_points


def xy_array(value, name, minimum=3):
    points = np.asarray(value, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < minimum
            or not np.isfinite(points).all()):
        raise ValueError(f"{name} must contain at least {minimum} finite XY pairs")
    return points


class ClosedPath:
    """Periodic segment projection/interpolation including the closing segment."""

    def __init__(self, points):
        self.points = xy_array(points, "path")
        if np.linalg.norm(self.points[0] - self.points[-1]) < 1e-7:
            self.points = self.points[:-1]
        self.delta = np.roll(self.points, -1, axis=0) - self.points
        self.ds = np.linalg.norm(self.delta, axis=1)
        if len(self.points) < 3 or np.any(self.ds < 1e-6):
            raise ValueError("Path contains duplicate or degenerate segments")
        self.s = np.r_[0.0, np.cumsum(self.ds)]
        self.length = float(self.s[-1])
        self.tangents = self.delta / self.ds[:, None]

    def at(self, distance):
        distance = np.asarray(distance) % self.length
        index = np.minimum(np.searchsorted(self.s, distance, side="right") - 1,
                           len(self.points) - 1)
        fraction = (distance - self.s[index]) / self.ds[index]
        return self.points[index] + self.delta[index] * np.expand_dims(fraction, -1)

    def project(self, point):
        offset = np.asarray(point) - self.points
        fraction = np.clip(np.sum(offset * self.delta, axis=1) / self.ds**2, 0, 1)
        candidates = self.points + fraction[:, None] * self.delta
        i = int(np.argmin(np.sum((candidates - point)**2, axis=1)))
        tangent = self.tangents[i]
        error = np.asarray(point) - candidates[i]
        return (float((self.s[i] + fraction[i] * self.ds[i]) % self.length),
                float(tangent[0] * error[1] - tangent[1] * error[0]), i)

    def velocity_at(self, distance, velocity):
        # Interpolate squared speeds: preserves constant-acceleration bounds.
        v = np.asarray(velocity, dtype=float)
        return np.sqrt(np.interp(np.asarray(distance) % self.length,
                                 self.s, np.r_[v**2, v[0]**2]))


def corridor_polygon(left, right):
    a, b = Polygon(left), Polygon(right)
    if not a.is_valid or not b.is_valid or a.area == 0 or b.area == 0:
        raise ValueError("Track boundaries must be simple closed rings")
    outer, inner = (a, b) if a.area > b.area else (b, a)
    if not outer.contains(inner):
        raise ValueError("Track boundaries intersect or are not nested")
    return Polygon(outer.exterior.coords, [inner.exterior.coords])


def validate_map(data, radius=0.24, allowance=0.05):
    if not isinstance(data, dict):
        raise ValueError("RaceMap must be an object")
    if data.get("frame_id", "world") != "world" or data.get("units", "m") != "m":
        raise ValueError("RaceMap must use world coordinates in metres")
    if data.get("closed", True) is not True:
        raise ValueError("Planned driving requires a closed track")
    left = xy_array(data.get("LeftBound"), "LeftBound")
    right = xy_array(data.get("RightBound"), "RightBound")
    ref = np.asarray(data.get("ReferencePoint"), dtype=float)
    if ref.shape != (2,) or not np.isfinite(ref).all():
        raise ValueError("ReferencePoint must be a finite XY pair")
    if left.shape != right.shape:
        raise ValueError("Boundaries must be paired and equally sampled")
    mid = ClosedPath((left + right) / 2)
    if np.max(mid.ds) > 0.5:
        raise ValueError("Incomplete boundary sampling: midpoint gap exceeds 0.5 m")
    # Labels are relative to driving direction, not clockwise polygon winding.
    tangent = np.roll(mid.points, -1, axis=0) - np.roll(mid.points, 1, axis=0)
    delta = left[:len(tangent)] - right[:len(tangent)]
    if np.any(tangent[:, 0] * delta[:, 1] - tangent[:, 1] * delta[:, 0] <= 0):
        raise ValueError("Left/right boundary pairing or driving direction is invalid")
    if np.min(np.linalg.norm(delta, axis=1)) <= 2 * (radius + allowance):
        raise ValueError("Track is too narrow for the configured vehicle envelope")
    corridor = corridor_polygon(left, right)
    if not corridor.buffer(1e-8).covers(LineString(np.vstack([mid.points, mid.points[0]]))):
        raise ValueError("Paired boundary midline leaves the corridor")
    return left, right, corridor


def footprint(xy, heading, wheelbase=0.324, radius=0.29):
    front = np.asarray(xy) + wheelbase * np.array([np.cos(heading), np.sin(heading)])
    return LineString([xy, front]).buffer(radius)


def clean_lap(rows):
    """Select a full lap between two observed crossings, never a partial first lap."""
    for key in ("collision_count", "resets"):
        values = [r.get(key) for r in rows]
        if any(v is None for v in values) or max(values) != min(values):
            raise ValueError(f"Mapping recording has missing or changing {key}")
    crossings = []
    for i in range(1, len(rows)):
        a, b = rows[i - 1].get("lap_count"), rows[i].get("lap_count")
        if a is None or b is None or b < a or b > a + 1:
            raise ValueError("Missing/discontinuous lap counters")
        if b == a + 1:
            crossings.append(i)
    if len(crossings) < 2:
        raise ValueError("Record at least two finish crossings to supply a complete lap")
    lap = rows[crossings[0]:crossings[1] + 1]
    for r in lap:
        if any(r.get(k, float("inf")) > 0.5
               for k in ("odom_age_s", "lap_count_age_s", "collision_count_age_s")):
            raise ValueError("Selected lap contains stale or missing pose/counter telemetry")
    points = xy_array([[r.get("x"), r.get("y")] for r in lap], "recorded lap")
    points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-4]]
    if np.linalg.norm(points[-1] - points[0]) > 0.5:
        raise ValueError("Recorded lap does not close within 0.5 m")
    if np.linalg.norm(points[-1] - points[0]) < 0.02:
        points = points[:-1]
    if np.max(ClosedPath(points).ds) > 0.5:
        raise ValueError("Pose gaps exceed 0.5 m; record a slower mapping lap")
    return points


def build_map(rows, outline, spacing=0.1):
    trace = clean_lap(rows)
    hits = xy_array(track_map_points(outline), "LiDAR outline")
    if outline.get("truncated") or outline.get("reset_count", 0):
        raise ValueError("LiDAR outline was truncated or reset; record a fresh mapping run")
    path = ClosedPath(trace)
    stations = np.arange(0, path.length, spacing)
    reference = gaussian_filter1d(path.at(stations), 2.0, axis=0, mode="wrap")
    path = ClosedPath(reference)
    tangents = np.roll(reference, -1, axis=0) - np.roll(reference, 1, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    bounds = []
    for sign in (1, -1):
        widths = np.full(len(reference), np.nan)
        for i, point in enumerate(reference):
            offsets = hits - point
            across = offsets @ normals[i]
            along = offsets @ tangents[i]
            # A normal cross-section, not nearest-waypoint assignment: the latter
            # leaves artificial holes on the inside of tight bends.
            selected = across[(abs(along) <= 0.10) & (sign * across > 0.1)
                              & (abs(across) <= 2.5)]
            if len(selected):
                widths[i] = np.min(abs(selected))
        valid = np.flatnonzero(np.isfinite(widths))
        if len(valid) < 3:
            raise ValueError("Insufficient LiDAR support for both boundaries")
        gaps = np.diff(np.r_[path.s[valid], path.s[valid[0]] + path.length])
        if max(gaps) > 0.5:
            raise ValueError("Boundary observation gap exceeds 0.5 m; collect a slower lap")
        widths = np.interp(path.s[:-1], path.s[valid], widths[valid], period=path.length)
        boundary = reference + sign * widths[:, None] * normals
        # Outline hits are quantized and receive-time aligned, not an exact wall
        # survey. Smooth at 0.15 m scale; the final footprint is also checked
        # against raw hits so fitting cannot erase an obstacle.
        bounds.append(gaussian_filter1d(boundary, 1.5, axis=0, mode="wrap"))
    result = {
        "LeftBound": bounds[0].tolist(), "RightBound": bounds[1].tolist(),
        "ReferencePoint": [0.0, 0.0], "frame_id": "world", "units": "m", "closed": True,
        "source": "lidar_outline_with_simulator_ground_truth", "spacing_m": spacing,
        "recorded_path": trace.tolist(), "observed_hits": hits.tolist(),
    }
    validate_map(result)
    return result


def plot_map(data, output, plan=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 9), layout="constrained")
    if data.get("observed_hits"):
        hits = np.asarray(data["observed_hits"])
        ax.scatter(*hits.T, s=1, c="gray", alpha=0.3, label="Observed LiDAR hits")
    for key, label in (("LeftBound", "Left boundary"), ("RightBound", "Right boundary"),
                       ("recorded_path", "Recorded lap")):
        if key in data:
            p = np.asarray(data[key])
            ax.plot(*np.vstack([p, p[0]]).T, label=label)
    if plan is not None:
        p = np.asarray(plan["path"])
        ax.plot(*np.vstack([p, p[0]]).T, color="black", linewidth=1, label="Planned line")
        im = ax.scatter(*p.T, c=plan["velocity"], s=12, cmap="viridis")
        fig.colorbar(im, ax=ax, label="Planned speed (m/s)")
    ax.set(xlabel="World X (m)", ylabel="World Y (m)", aspect="equal",
           title="Reference-assisted track map and racing line")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.2)
    try:
        fig.savefig(output, dpi=160)
    finally:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--outline", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, help="Also generate and validate an AVLite plan")
    args = parser.parse_args()
    outline_path = args.outline or args.recording.with_suffix(".track.json")
    data = build_map(read_samples(args.recording), json.loads(outline_path.read_text()))
    data["source_recording"] = str(args.recording)
    data["source_recording_sha256"] = hashlib.sha256(args.recording.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    plot_map(data, args.output.with_suffix(".png"))
    print(f"RaceMap: {args.output}", flush=True)
    if args.config:
        from .configuration import load_config
        from .race_planning import prepare_plan
        config = load_config(args.config)
        config.setdefault("planning", {})["map_path"] = str(args.output.resolve())
        prepared = prepare_plan(config, args.config)
        prepared.save(args.output.with_suffix(".plan.json"))
        plot_map(data, args.output.with_suffix(".png"), prepared.artifact)
        print(f"Validated plan: {args.output.with_suffix('.plan.json')}", flush=True)


if __name__ == "__main__":
    main()
