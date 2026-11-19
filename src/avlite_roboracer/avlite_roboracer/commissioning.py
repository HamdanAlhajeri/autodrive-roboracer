"""Command latency and stopping distance from a recorded commissioning run (no ROS).

Record the driver's command topic and odometry while issuing speed steps and stops at
low speed (manual teleop or the actuator test). Each event is measured from the command
stamp to the odometry response. The suggested profile values are the worst observed
cases; review them before copying them into config/roboracer/hardware.yaml.

    python -m avlite_roboracer.commissioning records.jsonl --output commissioning.json
"""

import argparse
import json
import math
from pathlib import Path

from .records import read_records

NS = 1e-9


def _time(record):
    stamp = record.get("stamp_ns")
    return (stamp if isinstance(stamp, int) else record["rx_ns"]) * NS


def _series(records):
    drive = sorted(((_time(r), r["speed"]) for r in records if r["kind"] == "drive"))
    odom = sorted(((_time(r), r["vx"], r["x"], r["y"]) for r in records if r["kind"] == "odom"))
    return drive, odom


def _after(odom, t):
    return [o for o in odom if o[0] >= t]


def _acceleration(odom, response, command_t, commanded, window_s):
    """Mean acceleration from the first response until the car reaches 90% of the commanded speed
    (or the window ends while it is still speeding up). None without a usable stretch."""
    if response is None:
        return None
    rising = [response]
    for sample in _after(odom, response[0]):
        if sample[0] - command_t > window_s or sample[1] < rising[-1][1] - 0.05:
            break
        rising.append(sample)
        if sample[1] >= 0.9 * commanded:
            break
    first, last = rising[0], rising[-1]
    if last[0] - first[0] < 0.1 or last[1] - first[1] < 0.1:
        return None
    return (last[1] - first[1]) / (last[0] - first[0])


def analyze(records, moving_mps=0.3, stopped_mps=0.05, response_mps=0.1, window_s=5.0):
    """Find start steps and stop commands; measure latency, deceleration and distance."""
    drive, odom = _series(records)
    if len(drive) < 2 or len(odom) < 2:
        raise ValueError("Recording needs drive commands and odometry")
    starts, stops = [], []
    for (t0, before), (t1, after) in zip(drive, drive[1:]):
        if before <= stopped_mps < moving_mps <= after:
            response = next((o for o in _after(odom, t1)
                             if o[1] >= response_mps and o[0] - t1 <= window_s), None)
            starts.append({"t": t1, "commanded_mps": after,
                           "latency_s": None if response is None else response[0] - t1,
                           "acceleration_mps2": _acceleration(odom, response, t1, after, window_s)})
        elif before >= moving_mps and after <= stopped_mps:
            following = [o for o in _after(odom, t1) if o[0] - t1 <= window_s]
            if not following:
                continue
            v0 = following[0][1]
            responding = next((o for o in following if o[1] <= v0 - response_mps), None)
            halted = next((o for o in following if o[1] <= stopped_mps), None)
            distance = 0.0
            for a, b in zip(following, following[1:]):
                if halted is not None and a[0] >= halted[0]:
                    break
                distance += math.hypot(b[2] - a[2], b[3] - a[3])
            event = {"t": t1, "initial_speed_mps": v0,
                     "latency_s": None if responding is None else responding[0] - t1,
                     "stop_time_s": None if halted is None else halted[0] - t1,
                     "stopping_distance_m": distance if halted is not None else None}
            if halted is not None and responding is not None and halted[0] > responding[0]:
                event["deceleration_mps2"] = (responding[1] - halted[1]) / (
                    halted[0] - responding[0])
            stops.append(event)
    return {"starts": starts, "stops": stops, "suggested_profile": suggest(starts, stops)}


def suggest(starts, stops):
    """Worst observed values; None where no complete event supports a value."""
    complete = [s for s in stops if s["stopping_distance_m"] is not None
                and s.get("deceleration_mps2")]
    latencies = [e["latency_s"] for e in starts + stops if e["latency_s"] is not None]
    accelerations = [s["acceleration_mps2"] for s in starts if s.get("acceleration_mps2")]
    # The slowest start is the one the plan may rely on, so the lowest value is suggested.
    acceleration = min(accelerations) if accelerations else None
    if not complete:
        return {"measurements.command_latency_s": max(latencies) if latencies else None,
                "measurements.max_acceleration_mps2": acceleration}
    fastest = max(complete, key=lambda s: s["initial_speed_mps"])
    return {
        "measurements.command_latency_s": max(latencies) if latencies else None,
        "measurements.max_acceleration_mps2": acceleration,
        "measurements.braking_deceleration_mps2": min(s["deceleration_mps2"] for s in complete),
        "measurements.stop_test_speed_mps": fastest["initial_speed_mps"],
        "measurements.stopping_distance_m": max(s["stopping_distance_m"] for s in complete
                                                if s["initial_speed_mps"]
                                                >= 0.9 * fastest["initial_speed_mps"]),
        "stop_events": len(complete),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("records", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = analyze(read_records(args.records))
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["suggested_profile"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
