"""Compact, ROS-free records exported from rosbag2 recordings.

rosbag2 is the source of truth for replay. Analysis and tests use these JSON records so
they run without ROS: one object per message with its topic, kind, receive time and
header stamp in nanoseconds. Converters accept any object shaped like the ROS message.
"""

import json
import math
from pathlib import Path

from .geometry import yaw_from_quaternion

KINDS = ("scan", "odom", "imu", "tf", "tf_static", "pose", "graph", "drive", "status")


def stamp_ns(header):
    """Return a header stamp in integer nanoseconds, or None if missing or invalid."""
    try:
        sec, nanosec = header.stamp.sec, header.stamp.nanosec
    except AttributeError:
        return None
    if not (isinstance(sec, int) and isinstance(nanosec, int)):
        return None
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None
    value = sec * 1_000_000_000 + nanosec
    return value or None


def _finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def scan_record(msg):
    """Keep scan geometry and ranges; nonfinite ranges become null (no return/corrupt)."""
    return {
        "frame": msg.header.frame_id, "stamp_ns": stamp_ns(msg.header),
        "angle_min": float(msg.angle_min), "angle_increment": float(msg.angle_increment),
        "range_min": float(msg.range_min), "range_max": float(msg.range_max),
        "ranges": [None if not math.isfinite(r) else round(float(r), 4) for r in msg.ranges],
    }


def _covariance(values, indices):
    try:
        return [_finite_or_none(values[i]) for i in indices]
    except (IndexError, TypeError):
        return [None] * len(indices)


def odom_record(msg):
    """Keep planar pose, body velocity, yaw rate and the x/y/yaw covariance diagonal."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return {
        "frame": msg.header.frame_id, "child": msg.child_frame_id,
        "stamp_ns": stamp_ns(msg.header),
        "x": float(p.x), "y": float(p.y), "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
        "vx": float(msg.twist.twist.linear.x), "vy": float(msg.twist.twist.linear.y),
        "wz": float(msg.twist.twist.angular.z),
        "cov": _covariance(msg.pose.covariance, (0, 7, 35)),
    }


def imu_record(msg):
    return {"frame": msg.header.frame_id, "stamp_ns": stamp_ns(msg.header),
            "wz": float(msg.angular_velocity.z),
            "ax": float(msg.linear_acceleration.x)}


def pose_record(msg):
    """Convert PoseWithCovarianceStamped (SLAM Toolbox's pose output)."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return {"frame": msg.header.frame_id, "stamp_ns": stamp_ns(msg.header),
            "x": float(p.x), "y": float(p.y), "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
            "cov": _covariance(msg.pose.covariance, (0, 7, 35))}


def tf_record(msg):
    """Convert a TFMessage into planar parent->child transforms with their stamps."""
    transforms = []
    for t in msg.transforms:
        q, v = t.transform.rotation, t.transform.translation
        transforms.append({
            "parent": t.header.frame_id, "child": t.child_frame_id,
            "stamp_ns": stamp_ns(t.header), "x": float(v.x), "y": float(v.y),
            "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
        })
    return {"transforms": transforms}


def graph_record(msg):
    """Extract pose-graph node positions from SLAM Toolbox's MarkerArray, ordered by id.

    SLAM Toolbox draws each vertex as a sphere marker whose id increases with time; edges
    use another marker type and are ignored.
    """
    sphere = 2
    nodes = sorted((m.id, float(m.pose.position.x), float(m.pose.position.y))
                   for m in msg.markers if m.type == sphere)
    frame = msg.markers[0].header.frame_id if msg.markers else None
    return {"frame": frame, "nodes": [[i, x, y] for i, x, y in nodes]}


def drive_record(msg):
    return {"frame": msg.header.frame_id, "stamp_ns": stamp_ns(msg.header),
            "steering": float(msg.drive.steering_angle), "speed": float(msg.drive.speed)}


def status_record(msg):
    """Keep a JSON status snapshot (supervisor or actuator); invalid JSON becomes null."""
    try:
        return {"data": json.loads(msg.data)}
    except (TypeError, ValueError):
        return {"data": None}


CONVERTERS = {"scan": scan_record, "odom": odom_record, "imu": imu_record,
              "tf": tf_record, "tf_static": tf_record, "pose": pose_record,
              "graph": graph_record, "drive": drive_record, "status": status_record}


def make_record(kind, topic, receive_ns, msg):
    """Wrap one converted message with its kind, topic and receive time."""
    record = {"kind": kind, "topic": topic, "rx_ns": int(receive_ns)}
    record.update(CONVERTERS[kind](msg))
    return record


def write_records(path, records):
    with Path(path).open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, allow_nan=False) + "\n")


def read_records(path):
    """Read exported JSONL records, sorted by receive time (stable for equal times)."""
    records = []
    with Path(path).open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            record = json.loads(line)
            if (not isinstance(record, dict) or record.get("kind") not in KINDS
                    or not isinstance(record.get("rx_ns"), int)):
                raise ValueError(f"{path}:{number}: invalid record")
            records.append(record)
    if not records:
        raise ValueError(f"{path}: no records")
    records.sort(key=lambda r: r["rx_ns"])
    return records
