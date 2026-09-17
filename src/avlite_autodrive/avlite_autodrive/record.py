"""Record actual simulator lap/collision and command telemetry; never controls the car."""

import argparse
import json
import math
import signal
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float32, Bool, String
from ackermann_msgs.msg import AckermannDriveStamped
from .ros_utils import PREFIX, odom_state
from .diagnostics import ACTUATOR_FIELDS, CONTROLLER_FIELDS, diagnostic_values
from .lap_tracking import LapProgress
from .track_map import TrackMap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=600)
    parser.add_argument("--output", required=True)
    parser.add_argument("--track-map", action="store_true",
                        help="Save a LiDAR hit outline registered with simulator world poses")
    parser.add_argument("--stop-after-lap", action="store_true")
    parser.add_argument("--laps", type=int, help="Stop after this many observed lap crossings")
    parser.add_argument("--stop-on-incident", action="store_true",
                        help="End capture on a collision, reset or lap-counter discontinuity")
    parser.add_argument("--wait-for-odom", type=float, default=30,
                        help="Seconds to wait for valid odometry before starting the timer")
    parser.add_argument("--odom-timeout", type=float, default=10,
                        help="Fail if valid odometry stops for this many seconds")
    args = parser.parse_args()
    if args.laps is not None and args.laps < 1:
        parser.error("--laps must be a positive integer")
    if args.stop_after_lap and args.laps not in (None, 1):
        parser.error("Use --laps alone for more than one lap")
    progress = LapProgress(args.laps or (1 if args.stop_after_lap else None))
    track_map = TrackMap() if args.track_map else None
    for name in ("seconds", "wait_for_odom", "odom_timeout"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("avlite_lap_recorder")
    state, initial = {}, {}
    counters = {"distance_m": 0.0, "resets": 0, "max_speed_mps": 0.0,
                "odometry_samples": 0}
    last_received = {}
    previous = None
    counter_names = ("lap_count", "collision_count")
    counter_max_age_s = {name: None for name in counter_names}
    counter_telemetry_status = "missing"
    counter_telemetry_error = None
    counter_baseline_before_motion = True
    first_motion_elapsed_s = None
    recording_started_stationary = None
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def scalar(name, msg):
        # AutoDRIVE sends infinity until the first timed lap; keep valid JSON.
        value = msg.data if math.isfinite(msg.data) else None
        if name in counter_names and (value is None or value < 0):
            value = None
        state[name] = value
        last_received[name] = time.monotonic()
        if name not in counter_names or (start is not None and value is not None):
            initial.setdefault(name, value)
        if name == "lap_count" and start is not None:
            progress.observe(value, time.monotonic() - start)

    def odom(msg):
        nonlocal previous, first_motion_elapsed_s, recording_started_stationary
        try:
            x, y, heading, speed = odom_state(msg)
        except ValueError:
            counters["invalid_odom_samples"] = counters.get("invalid_odom_samples", 0) + 1
            return
        last_received["odom"] = time.monotonic()
        if track_map is not None:
            track_map.add_odometry(msg, last_received["odom"])
        if recording_started_stationary is None:
            recording_started_stationary = abs(speed) <= 0.1
        if start is not None and first_motion_elapsed_s is None and abs(speed) > 0.1:
            first_motion_elapsed_s = time.monotonic() - start
        counters["odometry_samples"] += 1
        if previous is not None:
            delta = math.hypot(x - previous[0], y - previous[1])
            if delta > 1:
                counters["resets"] += 1
            else:
                counters["distance_m"] += delta
        previous = (x, y)
        state.update(x=x, y=y, heading=heading, speed=speed)
        counters["max_speed_mps"] = max(counters["max_speed_mps"], abs(speed))

    def command(msg):
        last_received["avlite_command"] = time.monotonic()
        state.update(
            avlite_steer_rad=(
                msg.drive.steering_angle if math.isfinite(msg.drive.steering_angle) else None
            ),
            avlite_acceleration=(
                msg.drive.acceleration if math.isfinite(msg.drive.acceleration) else None
            ),
        )

    def reset(msg):
        if msg.data:
            counters["resets"] += 1
            if track_map is not None:
                track_map.reset()

    def scan(msg):
        if track_map is not None and start is not None:
            track_map.add_scan(msg, time.monotonic())

    def diagnostics(name, fields, msg):
        values = diagnostic_values(msg.data, fields)
        if values is not None:
            state.update(values)
            last_received[name] = time.monotonic()

    def race_plan(msg):
        # Capture the plan actually used by the running controller, not a later
        # recomputation using possibly edited files. The topic is latched at startup.
        try:
            artifact = json.loads(msg.data)
            if (artifact.get("version") != 1 or artifact.get("frame_id") != "world"
                    or not artifact.get("path") or not artifact.get("map")
                    or len(artifact["velocity"]) != len(artifact["path"])):
                raise ValueError("invalid race-plan artifact")
            payload = json.dumps(artifact, indent=2, allow_nan=False) + "\n"
            path.with_suffix(".plan.json").write_text(payload)
            path.with_suffix(".racemap.json").write_text(
                json.dumps(artifact["map"], indent=2, allow_nan=False) + "\n")
            path.with_suffix(".planning-config.json").write_text(
                json.dumps(artifact["resolved_config"], indent=2, allow_nan=False) + "\n")
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            node.get_logger().error(f"Ignoring invalid race plan: {exc}")

    def check_counters(now, elapsed):
        nonlocal counter_telemetry_status, counter_telemetry_error
        nonlocal counter_baseline_before_motion
        missing, stale, invalid = [], [], []
        for name in counter_names:
            if name not in initial:
                missing.append(name)
            if name in last_received:
                age = max(0.0, now - last_received[name])
                counter_max_age_s[name] = max(counter_max_age_s[name] or 0.0, age)
                if name in initial and age > 0.5:
                    stale.append(name)
                if state.get(name) is None:
                    invalid.append(name)
        counter_telemetry_status = (
            "invalid" if invalid else "stale" if stale else "missing" if missing else "fresh"
        )
        if abs(state.get("speed", 0.0)) > 0.1 and missing:
            counter_baseline_before_motion = False
        if counter_telemetry_error is None:
            if invalid:
                counter_telemetry_error = "Invalid lap/collision telemetry: " + ", ".join(invalid)
            elif stale:
                counter_telemetry_error = (
                    "Lap/collision telemetry stopped for more than 0.5s: " + ", ".join(stale)
                )
            elif missing and elapsed >= args.wait_for_odom:
                counter_telemetry_error = (
                    f"No lap/collision baseline received within {args.wait_for_odom:g}s: "
                    + ", ".join(missing)
                )

    def write_sample(stream, now):
        sample = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "elapsed_s": now - start,
            **state,
            **counters,
            **{key + "_age_s": now - received for key, received in last_received.items()},
        }
        stream.write(json.dumps(sample, allow_nan=False) + "\n")
        stream.flush()

    for name in (
        "lap_count",
        "collision_count",
        "last_lap_time",
        "throttle_command",
        "steering_command",
        "throttle",
        "steering",
    ):
        kind = Int32 if name.endswith("count") else Float32
        node.create_subscription(kind, PREFIX + "/" + name, lambda m, n=name: scalar(n, m), 10)
    node.create_subscription(Odometry, PREFIX + "/odom", odom, qos_profile_sensor_data)
    if track_map is not None:
        node.create_subscription(LaserScan, PREFIX + "/lidar", scan, qos_profile_sensor_data)
    node.create_subscription(AckermannDriveStamped, "/avlite/control_command", command, 10)
    node.create_subscription(Bool, "/autodrive/reset_command", reset, 10)
    node.create_subscription(String, "/avlite/race_plan", race_plan,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for name, fields in (("controller_diagnostics", CONTROLLER_FIELDS),
                         ("actuator_diagnostics", ACTUATOR_FIELDS)):
        node.create_subscription(String, "/avlite/" + name,
                                 lambda msg, n=name, f=fields: diagnostics(n, f, msg), 10)
    waiting_since = time.monotonic()
    start, started_utc = None, None
    completed = False
    lap_seen_at = None
    error = None
    incident_detected = False
    stop_reason = "time_limit"
    try:
        with path.open("w") as stream:
            print(f"Waiting up to {args.wait_for_odom:g}s for valid simulator odometry...",
                  flush=True)
            while not stopped and "odom" not in last_received:
                rclpy.spin_once(node, timeout_sec=0.05)
                if ("odom" not in last_received
                        and time.monotonic() - waiting_since >= args.wait_for_odom):
                    error = (
                        "No valid odometry received. Start/connect the simulator and finish "
                        "restarting the controllers before recording. If the bridge was "
                        "restarted, rerun the recording command to attach to its new connection."
                    )
                    break
            if not stopped and error is None:
                start = time.monotonic()
                # Counters observed during odometry discovery establish only a
                # baseline. Crossings before capture starts are never credited.
                for name in counter_names:
                    if state.get(name) is not None:
                        initial[name] = state[name]
                progress.observe(state.get("lap_count"), 0.0)
                started_utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                print(f"Odometry received. Recording {args.seconds:g}s of telemetry.", flush=True)
            next_sample = 0
            while (start is not None and not stopped
                   and time.monotonic() - start < args.seconds):
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                elapsed = now - start
                check_counters(now, elapsed)
                if now - last_received["odom"] >= args.odom_timeout:
                    error = (
                        f"Valid odometry stopped for {args.odom_timeout:g}s. Partial data saved. "
                        "Check the simulator connection, finish any controller/bridge restart, "
                        "then rerun the recording command."
                    )
                if progress.requested_laps is not None and counter_telemetry_error is not None:
                    error = error or counter_telemetry_error
                if elapsed >= next_sample:
                    write_sample(stream, now)
                    next_sample = elapsed + 0.1
                if error is not None:
                    break
                completed = progress.completed_laps > 0
                incident_detected = incident_detected or bool(
                    counters["resets"] or progress.counter_discontinuity
                    or ("collision_count" in initial
                        and state.get("collision_count") != initial["collision_count"])
                )
                if args.stop_on_incident and incident_detected:
                    stop_reason = "incident"
                    break
                if progress.target_reached and lap_seen_at is None:
                    lap_seen_at = elapsed
                # Allow collision feedback from the finish frame to arrive too.
                if lap_seen_at is not None and elapsed > lap_seen_at + 1:
                    stop_reason = "lap_target"
                    break
            if start is not None:
                # Stop conditions can arrive between periodic sample deadlines.
                # Persist the event and final input ages for plots and CSV too.
                now = time.monotonic()
                check_counters(now, now - start)
                if progress.requested_laps is not None and counter_telemetry_error is not None:
                    error = error or counter_telemetry_error
                write_sample(stream, now)
    except KeyboardInterrupt:
        stopped = True
    finally:
        summary = {
            "status": "failed" if error else "interrupted" if stopped else "completed",
            "error": error,
            "stop_reason": "sensor_error" if error else "interrupted" if stopped else stop_reason,
            "started_utc": started_utc,
            "ended_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "has_odometry": "speed" in state,
            "elapsed_s": time.monotonic() - start if start is not None else 0.0,
            "waiting_s": (start if start is not None else time.monotonic()) - waiting_since,
            "initial": initial,
            "final": state,
            **counters,
            **progress.summary(),
            "incident_detected": incident_detected,
            "counter_telemetry_status": counter_telemetry_status,
            "counter_telemetry_error": counter_telemetry_error,
            "counter_max_age_s": counter_max_age_s,
            "counter_baseline_before_motion": counter_baseline_before_motion,
            "counter_telemetry_valid": bool(
                counter_telemetry_status == "fresh" and counter_telemetry_error is None
                and counter_baseline_before_motion
            ),
            "recording_started_stationary": recording_started_stationary,
            "first_motion_elapsed_s": first_motion_elapsed_s,
            "first_lap_driving_s": (
                progress.finish_times_s[0] - first_motion_elapsed_s
                if recording_started_stationary and progress.finish_times_s
                and first_motion_elapsed_s is not None else None
            ),
            "clean_lap": bool(
                completed
                and error is None
                and not stopped
                and counters["resets"] == 0
                and "collision_count" in initial
                and state["collision_count"] == initial["collision_count"]
                and not incident_detected
            ),
        }
        summary["clean_run"] = bool(
            summary["clean_lap"] and summary["counter_telemetry_valid"]
            and not progress.counter_discontinuity
            and (progress.requested_laps is None
                 or (progress.target_reached and stop_reason == "lap_target"))
        )
        summary["clean_lap"] = summary["clean_lap"] and summary["counter_telemetry_valid"]
        if summary["clean_run"]:
            screening_failure_reason = None
        elif error or counter_telemetry_error:
            screening_failure_reason = error or counter_telemetry_error
        elif stopped:
            screening_failure_reason = "Recording was interrupted"
        elif not summary["counter_telemetry_valid"]:
            screening_failure_reason = "Missing or incomplete counter telemetry"
        elif incident_detected:
            screening_failure_reason = "Collision, reset or counter discontinuity observed"
        elif not progress.target_reached:
            screening_failure_reason = "Requested lap target was not reached"
        else:
            screening_failure_reason = (
                "Recording ended before the one-second finish feedback window completed"
            )
        summary["screening_failure_reason"] = screening_failure_reason
        if track_map is not None:
            outline = track_map.snapshot()
            outline_path = path.with_suffix(".track.json")
            outline_path.write_text(json.dumps(outline, allow_nan=False) + "\n", encoding="utf-8")
            summary["track_map"] = {
                "file": outline_path.name,
                "point_count": len(outline["points"]),
                **{key: value for key, value in outline.items() if key != "points"},
            }
            print(f"Track outline: {outline_path} ({len(outline['points'])} observed cells)",
                  flush=True)
            if not outline["points"]:
                print("No synchronized LiDAR hits captured; graphs will show the path only.",
                      flush=True)
        path.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    if error:
        print(f"Recording failed: {error}", flush=True)
        return 2
    return 130 if stopped else 0


if __name__ == "__main__":
    raise SystemExit(main())
