"""Localization-quality report from exported records and a saved map (no ROS).

Typical use: build a map from one recording, replay a separate recording through SLAM
Toolbox in localization mode while recording its output, export that bag, then run this
report. Reference poses (for example from a motion-capture system) are used only to
measure error, never to correct the estimate.
"""

import csv
import math
from pathlib import Path

import numpy as np

from .geometry import interpolate, wrap_angle
from .localization import motion_mismatch, scan_alignment, scan_points
from .recording_check import NS, build_transforms, group_by_kind


def _stats(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return None
    array = np.asarray(values)
    return {"count": len(values), "median": float(np.median(array)),
            "p95": float(np.percentile(array, 95)), "max": float(np.max(array)),
            "mean": float(np.mean(array))}


def read_reference(path):
    """Read reference poses from CSV columns t_s, x, y, yaw (map frame, ROS time seconds)."""
    rows = []
    with Path(path).open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            rows.append((float(row["t_s"]), (float(row["x"]), float(row["y"]),
                                             float(row["yaw"]))))
    rows.sort()
    if len(rows) < 2:
        raise ValueError(f"{path}: need at least two reference poses")
    return rows


def _reference_at(reference, t, max_gap_s=0.1):
    times = [r[0] for r in reference]
    i = int(np.searchsorted(times, t))
    if i == 0 or i == len(reference):
        return None
    (t0, a), (t1, b) = reference[i - 1], reference[i]
    if t1 - t0 > max_gap_s:
        return None
    return interpolate(a, b, (t - t0) / (t1 - t0))


def _estimates(by_kind, buffer, frames):
    """Return map-frame estimates as dicts with stamp, pose, covariance and latency.

    Prefer the SLAM pose topic (it carries covariance); otherwise sample map -> base_link
    from /tf at each scan stamp.
    """
    poses = [r for r in by_kind.get("pose", []) if isinstance(r.get("stamp_ns"), int)]
    if poses:
        return [{"t": r["stamp_ns"] * NS, "pose": (r["x"], r["y"], r["yaw"]), "cov": r["cov"],
                 "latency": (r["rx_ns"] - r["stamp_ns"]) * NS} for r in poses], "pose_topic"
    estimates = []
    for scan in by_kind.get("scan", []):
        if not isinstance(scan.get("stamp_ns"), int):
            continue
        t = scan["stamp_ns"] * NS
        pose = buffer.lookup(frames["map"], frames["base_link"], t)
        if pose is not None:
            estimates.append({"t": t, "pose": pose, "cov": None, "latency": None})
    return estimates, "tf"


def localization_report(records, profile, grid=None, reference=None):
    """Measure freshness, uncertainty, scan alignment, motion consistency and reference error.

    Thresholds come from the profile; any that are still null are reported as unassessed
    rather than silently passing.
    """
    by_kind = group_by_kind(records)
    buffer, *_ = build_transforms(records)
    frames = {k: profile.frame(k) for k in ("map", "odom", "base_link")}
    estimates, source = _estimates(by_kind, buffer, frames)
    report = {"estimate_source": source, "estimates": len(estimates),
              "profile": profile.summary(), "limitations": []}
    if len(estimates) < 2:
        report["status"] = "fail"
        report["limitations"].append("fewer than two localization estimates in the recording")
        return report
    times = [e["t"] for e in estimates]
    gaps = [b - a for a, b in zip(times, times[1:])]
    position_sd = [math.sqrt(max(e["cov"][0], 0) + max(e["cov"][1], 0))
                   for e in estimates if e["cov"] and None not in e["cov"][:2]]
    yaw_sd = [math.sqrt(max(e["cov"][2], 0)) for e in estimates
              if e["cov"] and e["cov"][2] is not None]
    mismatch_t, mismatch_r, unmatched = [], [], 0
    previous = None
    for e in estimates:
        odom = buffer.lookup(frames["odom"], frames["base_link"], e["t"])
        if odom is None:
            unmatched += 1
            previous = None
            continue
        if previous is not None:
            dt, dr = motion_mismatch(previous[0], e["pose"], previous[1], odom)
            mismatch_t.append(dt)
            mismatch_r.append(dr)
        previous = (e["pose"], odom)
    report.update({
        "pose_interval_s": _stats(gaps),
        "processing_latency_s": _stats([e["latency"] for e in estimates]),
        "position_stddev_m": _stats(position_sd),
        "yaw_stddev_rad": _stats(yaw_sd),
        "motion_mismatch_m": _stats(mismatch_t),
        "motion_mismatch_rad": _stats(mismatch_r),
        "estimates_without_odometry": unmatched,
    })
    if not position_sd:
        report["limitations"].append("no pose covariance available (estimate source: tf)")
    tolerance = profile.get("localization.scan_alignment_tolerance_m", 0.1)
    mount = profile.laser_mount
    if grid is None or mount is None:
        report["scan_alignment"] = None
        report["limitations"].append(
            "scan alignment not measured: " + ("no map supplied" if grid is None
                                               else "laser mount not measured"))
    else:
        fractions, medians = [], []
        for scan in by_kind.get("scan", []):
            if not isinstance(scan.get("stamp_ns"), int):
                continue
            pose = buffer.lookup(frames["map"], frames["base_link"], scan["stamp_ns"] * NS)
            if pose is None:
                continue
            points = scan_points(scan["ranges"], scan["angle_min"], scan["angle_increment"],
                                 scan["range_min"], scan["range_max"])
            fraction, middle = scan_alignment(grid, points, pose, mount, tolerance)
            fractions.append(fraction)
            medians.append(middle)
        report["scan_alignment"] = _stats(fractions)
        report["scan_alignment_median_distance_m"] = _stats(medians)
    if reference is not None:
        position_error, yaw_error = [], []
        for e in estimates:
            truth = _reference_at(reference, e["t"])
            if truth is not None:
                position_error.append(math.hypot(e["pose"][0] - truth[0],
                                                 e["pose"][1] - truth[1]))
                yaw_error.append(abs(wrap_angle(e["pose"][2] - truth[2])))
        report["reference_position_error_m"] = _stats(position_error)
        report["reference_yaw_error_rad"] = _stats(yaw_error)
    else:
        report["limitations"].append("no reference poses: absolute error not measured")
    report["assessment"] = _assess(report, profile)
    statuses = [a["status"] for a in report["assessment"]]
    report["status"] = ("fail" if "fail" in statuses else
                        "unassessed" if "unassessed" in statuses else "pass")
    return report


def _assess(report, profile):
    """Compare statistics with profile thresholds; null thresholds are 'unassessed'.

    Motion consistency uses the worst update: one correction jump is exactly the event
    that would stop a racing car, so a percentile must not hide it.
    """
    rules = [
        ("position_stddev_m", "localization.max_position_stddev_m", "p95", "max"),
        ("yaw_stddev_rad", "localization.max_yaw_stddev_rad", "p95", "max"),
        ("motion_mismatch_m", "localization.max_motion_mismatch_m", "max", "max"),
        ("motion_mismatch_rad", "localization.max_motion_mismatch_rad", "max", "max"),
        ("scan_alignment", "localization.min_scan_alignment", "median", "min"),
        ("pose_interval_s", "timing.pose_timeout_s", "max", "max"),
    ]
    results = []
    for metric, key, statistic, kind in rules:
        threshold = profile.get(key)
        values = report.get(metric)
        if threshold is None or values is None:
            results.append({"metric": metric, "status": "unassessed", "threshold": threshold,
                            "value": None if values is None else values[statistic]})
            continue
        value = values[statistic]
        ok = value <= threshold if kind == "max" else value >= threshold
        results.append({"metric": metric, "statistic": statistic, "value": value,
                        "threshold": threshold, "status": "pass" if ok else "fail"})
    return results


def render_markdown(recording_report, localization):
    """Format both reports as a short Markdown summary for docs/validation."""
    lines = ["# Localization-quality report", ""]
    if recording_report is not None:
        lines += [f"Recorded-data checks: **{recording_report['status']}**", "",
                  "| Check | Status | Detail |", "| --- | --- | --- |"]
        lines += [f"| {c['name']} | {c['status']} | {c['detail']} |"
                  for c in recording_report["checks"]]
        lines.append("")
    if localization is not None:
        lines += [f"Localization: **{localization['status']}** "
                  f"({localization['estimates']} estimates from {localization['estimate_source']})",
                  "", "| Metric | Median | p95 | Max |", "| --- | --- | --- | --- |"]
        for key in ("pose_interval_s", "processing_latency_s", "position_stddev_m",
                    "yaw_stddev_rad", "motion_mismatch_m", "motion_mismatch_rad",
                    "scan_alignment", "reference_position_error_m", "reference_yaw_error_rad"):
            values = localization.get(key)
            if values:
                lines.append(f"| {key} | {values['median']:.4f} | {values['p95']:.4f} | "
                             f"{values['max']:.4f} |")
        lines += ["", "| Acceptance metric | Value | Threshold | Status |",
                  "| --- | --- | --- | --- |"]
        for item in localization.get("assessment", []):
            value = "n/a" if item["value"] is None else f"{item['value']:.4f}"
            lines.append(f"| {item['metric']} | {value} | {item['threshold']} | "
                         f"{item['status']} |")
        if localization["limitations"]:
            lines += ["", "Limitations:", ""] + [f"- {x}" for x in localization["limitations"]]
    return "\n".join(lines) + "\n"
