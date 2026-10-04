"""Recorded-data checks: timestamps, transforms and odometry consistency (no ROS).

Each check returns a status of pass, warn, fail or info with the measured values. These
are prerequisites for autonomous mapping: missing motion measurements, conflicting
transform publishers or unusable clocks must be resolved during commissioning.
"""

from collections import defaultdict
import math
from statistics import median

from .geometry import TransformBuffer, relative, wrap_angle

NS = 1e-9


def _check(name, status, detail, **values):
    return {"name": name, "status": status, "detail": detail, **values}


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * (len(ordered) - 1) + 0.5))]


def stream_timing(records):
    """Summarize one topic's header stamps and receive times.

    Latency is receive time minus header stamp. A large negative latency means the sensor
    stamps come from a different clock, which breaks transform lookups and SLAM.
    """
    stamps = [r.get("stamp_ns") for r in records]
    valid = [s for s in stamps if isinstance(s, int)]
    gaps = [(b - a) * NS for a, b in zip(valid, valid[1:])]
    latency = [(r["rx_ns"] - r["stamp_ns"]) * NS for r in records
               if isinstance(r.get("stamp_ns"), int)]
    duration = (valid[-1] - valid[0]) * NS if len(valid) > 1 else 0.0
    positive = [g for g in gaps if g > 0]
    return {
        "count": len(records),
        "missing_stamps": len(stamps) - len(valid),
        "nonmonotonic": sum(g < 0 for g in gaps),
        "duplicates": sum(g == 0 for g in gaps),
        "duration_s": duration,
        "rate_hz": (len(valid) - 1) / duration if duration > 0 else None,
        "median_period_s": median(positive) if positive else None,
        "max_gap_s": max(gaps) if gaps else None,
        "latency_median_s": median(latency) if latency else None,
        "latency_min_s": min(latency) if latency else None,
        "latency_max_s": max(latency) if latency else None,
    }


def check_timestamps(by_kind, profile):
    """Check that scan, odometry and (if configured) IMU streams are present and usable."""
    checks = []
    timeout = profile.get("timing.sensor_timeout_s", 0.25)
    required = ["scan", "odom"] + (["imu"] if profile.topic("imu") else [])
    for kind in required:
        records = by_kind.get(kind, [])
        if len(records) < 2:
            checks.append(_check(f"{kind}_timing", "fail", f"fewer than two {kind} messages"))
            continue
        timing = stream_timing(records)
        problems = []
        if timing["missing_stamps"]:
            problems.append(f"{timing['missing_stamps']} missing/invalid stamps")
        if timing["nonmonotonic"] or timing["duplicates"]:
            problems.append(f"{timing['nonmonotonic']} backwards and "
                            f"{timing['duplicates']} repeated stamps")
        if timing["max_gap_s"] is not None and timing["max_gap_s"] > timeout:
            problems.append(f"gap {timing['max_gap_s']:.3f}s exceeds sensor timeout {timeout}s")
        if timing["latency_min_s"] is not None and timing["latency_min_s"] < -0.05:
            problems.append("header stamps are ahead of receive time: clocks disagree")
        if timing["latency_max_s"] is not None and timing["latency_max_s"] > 1.0:
            problems.append("header stamps lag receive time by more than 1s")
        checks.append(_check(f"{kind}_timing", "fail" if problems else "pass",
                             "; ".join(problems) or "stamps monotonic, gaps within timeout",
                             **timing))
    return checks


def build_transforms(records):
    """Load /tf and /tf_static records into a TransformBuffer and collect publisher facts.

    Return (buffer, parents, static_children, dynamic_children, conflicts). A frame with two
    parents cannot be represented in one tree, so only the first parent is buffered.
    """
    buffer = TransformBuffer(max_extrapolation_s=0.2)
    parents = defaultdict(set)
    static_children, dynamic_children = set(), set()
    conflicts = []
    for record in records:
        if record["kind"] not in ("tf", "tf_static"):
            continue
        static = record["kind"] == "tf_static"
        for t in record["transforms"]:
            parents[t["child"]].add(t["parent"])
            (static_children if static else dynamic_children).add(t["child"])
            stamp = t["stamp_ns"] if isinstance(t["stamp_ns"], int) else record["rx_ns"]
            try:
                buffer.add(t["parent"], t["child"], stamp * NS,
                           (t["x"], t["y"], t["yaw"]), static=static)
            except ValueError as exc:
                conflicts.append(str(exc))
    return buffer, parents, static_children, dynamic_children, conflicts


def check_transforms(records, by_kind, profile, require_map=False):
    """Check the frame tree map -> odom -> base_link -> laser and single ownership.

    One publisher per transform is checked from the recording: each child frame must have
    one parent and must not appear on both /tf and /tf_static. The measured LiDAR mount must
    agree with the published static transform.
    """
    checks = []
    buffer, parents, static_children, dynamic_children, _ = build_transforms(records)
    multi = {child: sorted(p) for child, p in parents.items() if len(p) > 1}
    both = sorted(static_children & dynamic_children)
    problems = [f"{c} has parents {p}" for c, p in multi.items()]
    problems += [f"{c} is on both /tf and /tf_static" for c in both]
    checks.append(_check("transform_ownership", "fail" if problems else "pass",
                         "; ".join(problems) or "each frame has one parent and one stream",
                         parents={c: sorted(p) for c, p in parents.items()}))
    frames = {k: profile.frame(k) for k in ("map", "odom", "base_link", "laser")}
    scans = by_kind.get("scan", [])
    stamps = [r["stamp_ns"] * NS for r in scans if isinstance(r.get("stamp_ns"), int)]
    links = [("odom", "base_link"), ("base_link", "laser")]
    if require_map:
        links.insert(0, ("map", "odom"))
    for parent, child in links:
        name = f"tf_{parent}_to_{child}"
        if not stamps:
            checks.append(_check(name, "fail", "no scans to evaluate transform availability"))
            continue
        found = sum(buffer.lookup(frames[parent], frames[child], s) is not None for s in stamps)
        fraction = found / len(stamps)
        checks.append(_check(name, "pass" if fraction >= 0.99 else "fail",
                             f"available at {fraction:.1%} of scan stamps",
                             available_fraction=fraction))
    mount = profile.laser_mount
    if stamps and mount is not None:
        published = buffer.lookup(frames["base_link"], frames["laser"], stamps[0])
        if published is not None:
            error = math.hypot(published[0] - mount[0], published[1] - mount[1])
            yaw_error = abs(wrap_angle(published[2] - mount[2]))
            ok = error <= 0.01 and yaw_error <= math.radians(1)
            checks.append(_check("laser_mount_matches_profile", "pass" if ok else "fail",
                                 f"published mount differs by {error:.3f} m, "
                                 f"{math.degrees(yaw_error):.2f} deg",
                                 published=list(published), profile=list(mount)))
    return checks, buffer


def check_odometry(by_kind, profile):
    """Compare odometry pose increments with its own twist and with the IMU yaw rate.

    Wheel odometry that disagrees with itself, jumps, or rotates opposite to the IMU makes
    SLAM unreliable, so these findings must be resolved before autonomous mapping.
    """
    checks = []
    odom = [r for r in by_kind.get("odom", []) if isinstance(r.get("stamp_ns"), int)]
    if len(odom) < 3:
        return [_check("odometry_consistency", "fail", "insufficient odometry")]
    frames_ok = all(r["frame"] == profile.frame("odom") and r["child"] == profile.frame("base_link")
                    for r in odom)
    checks.append(_check("odometry_frames", "pass" if frames_ok else "fail",
                         f"expected {profile.frame('odom')} -> {profile.frame('base_link')}",
                         observed=sorted({(r["frame"], r["child"]) for r in odom})))
    distance_errors, yaw_rate_errors, jumps, travelled = [], [], 0, 0.0
    for a, b in zip(odom, odom[1:]):
        dt = (b["stamp_ns"] - a["stamp_ns"]) * NS
        if dt <= 0:
            continue
        delta = relative((a["x"], a["y"], a["yaw"]), (b["x"], b["y"], b["yaw"]))
        step = math.hypot(delta[0], delta[1])
        travelled += step
        predicted = 0.5 * (a["vx"] + b["vx"]) * dt
        distance_errors.append(abs(delta[0] - predicted))
        yaw_rate_errors.append(abs(delta[2] / dt - 0.5 * (a["wz"] + b["wz"])))
        if step > max(0.5, 3 * abs(predicted) + 0.1):
            jumps += 1
    if not distance_errors:
        return checks + [_check("odometry_self_consistency", "fail",
                                "odometry stamps never advance")]
    p95 = _percentile(distance_errors, 0.95)
    yaw95 = _percentile(yaw_rate_errors, 0.95)
    status = "fail" if jumps else "warn" if (p95 or 0) > 0.05 or (yaw95 or 0) > 0.3 else "pass"
    checks.append(_check(
        "odometry_self_consistency", status,
        f"{jumps} pose jumps; p95 pose-vs-twist error {p95:.3f} m, yaw-rate {yaw95:.3f} rad/s",
        jumps=jumps, distance_error_p95_m=p95, yaw_rate_error_p95_rps=yaw95,
        travelled_m=travelled))
    imu = [r for r in by_kind.get("imu", []) if isinstance(r.get("stamp_ns"), int)]
    if imu:
        pairs = []
        index = 0
        for r in imu:
            while index + 1 < len(odom) and odom[index + 1]["stamp_ns"] <= r["stamp_ns"]:
                index += 1
            if abs(odom[index]["stamp_ns"] - r["stamp_ns"]) * NS <= 0.05:
                pairs.append((r["wz"], odom[index]["wz"]))
        turning = [(i, o) for i, o in pairs if abs(o) > 0.2]
        if len(turning) < 10:
            checks.append(_check("imu_yaw_rate", "warn",
                                 "too little turning to compare IMU and odometry yaw rates",
                                 pairs=len(pairs)))
        else:
            agreement = sum(i * o > 0 for i, o in turning) / len(turning)
            rms = math.sqrt(sum((i - o) ** 2 for i, o in turning) / len(turning))
            checks.append(_check(
                "imu_yaw_rate", "fail" if agreement < 0.9 else "pass",
                f"IMU and odometry agree on turn direction {agreement:.0%}; RMS {rms:.3f} rad/s",
                sign_agreement=agreement, rms_rps=rms, samples=len(turning)))
    return checks


def check_scan_sync(by_kind):
    """Measure how far each scan stamp is from the nearest odometry stamp."""
    scans = sorted(r["stamp_ns"] for r in by_kind.get("scan", [])
                   if isinstance(r.get("stamp_ns"), int))
    odom = sorted(r["stamp_ns"] for r in by_kind.get("odom", [])
                  if isinstance(r.get("stamp_ns"), int))
    if not scans or not odom:
        return [_check("scan_odom_sync", "fail", "missing scans or odometry")]
    skews, index = [], 0
    for stamp in scans:
        while index + 1 < len(odom) and odom[index + 1] <= stamp:
            index += 1
        candidates = odom[index:index + 2]
        skews.append(min(abs(o - stamp) for o in candidates) * NS)
    worst = max(skews)
    return [_check("scan_odom_sync", "pass" if worst <= 0.1 else "warn",
                   f"max scan-to-odometry skew {worst:.3f}s",
                   max_skew_s=worst, median_skew_s=median(skews))]


def group_by_kind(records):
    by_kind = defaultdict(list)
    for record in records:
        by_kind[record["kind"]].append(record)
    return by_kind


def check_recording(records, profile, require_map=False):
    """Run every recorded-data check and return a report with an overall status."""
    by_kind = group_by_kind(records)
    checks = check_timestamps(by_kind, profile)
    transform_checks, _ = check_transforms(records, by_kind, profile, require_map)
    checks += transform_checks
    checks += check_odometry(by_kind, profile)
    checks += check_scan_sync(by_kind)
    status = ("fail" if any(c["status"] == "fail" for c in checks)
              else "warn" if any(c["status"] == "warn" for c in checks) else "pass")
    return {"status": status, "checks": checks,
            "message_counts": {kind: len(items) for kind, items in sorted(by_kind.items())}}
