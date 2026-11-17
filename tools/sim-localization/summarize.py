"""Summarize a simulator localization experiment directory (see experiment.sh)."""

import json
import math
from pathlib import Path
import sys


def stats(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return None
    pick = lambda q: values[min(len(values) - 1, int(q * (len(values) - 1) + 0.5))]
    return {"n": len(values), "median": round(pick(0.5), 4), "p95": round(pick(0.95), 4),
            "max": round(values[-1], 4)}


def diagnostics(path):
    rows = []
    if path.exists():
        for line in path.read_text(errors="replace").splitlines():
            line = line.strip()
            if line.startswith("{"):
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return rows


def lap_times(path):
    times = []
    if path.exists():
        for line in path.read_text(errors="replace").splitlines():
            try:
                value = round(float(line.strip()), 3)
            except ValueError:
                continue
            # The topic repeats the last lap time and reads inf before the first lap.
            if math.isfinite(value) and value > 0 and (not times or value != times[-1]):
                times.append(value)
    return times


def run_summary(out, label):
    rows = diagnostics(out / f"{label}-diagnostics.txt")
    counters_path = out / f"{label}-counters.json"
    counters = json.loads(counters_path.read_text()) if counters_path.exists() else {}
    summary = {**counters, "lap_times_s": lap_times(out / f"{label}-lap-times.txt"),
               "diagnostic_samples": len(rows),
               "step_time_s": stats([r.get("controller_step_time_s") for r in rows]),
               "overruns": sum(bool(r.get("controller_overrun")) for r in rows)}
    if any("localization_ok" in r for r in rows):
        ok = [r["localization_ok"] for r in rows if r.get("localization_ok") is not None]
        summary.update({
            "localization_ok_fraction": round(sum(ok) / len(ok), 4) if ok else None,
            "localization_error_m": stats([r.get("localization_error_m") for r in rows]),
            "localization_yaw_error_rad": stats([r.get("localization_yaw_error_rad")
                                                 for r in rows]),
            "pose_age_s": stats([r.get("localization_pose_age_s") for r in rows]),
            "scan_alignment": stats([r.get("localization_scan_alignment") for r in rows]),
        })
    return summary


def main(out):
    out = Path(out)
    result = {label: run_summary(out, label) for label in ("mapping", "slam", "avlite", "baseline")}
    origin = out / "map_origin_world.txt"
    result["map_origin_world"] = origin.read_text().strip() if origin.exists() else None
    (out / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    lines = ["# Simulator localization experiment", "",
             "| Run | Laps | Collisions | Lap times (s) | Loc. OK | Error median / p95 / max (m) |",
             "| --- | --- | --- | --- | --- | --- |"]
    for label in ("mapping", "slam", "avlite", "baseline"):
        r = result[label]
        err = r.get("localization_error_m")
        error = "-" if not err else "{median} / {p95} / {max}".format(**err)
        lines.append(f"| {label} | {r.get('laps')} | {r.get('collisions')} | "
                     f"{', '.join(map(str, r['lap_times_s'])) or '-'} | "
                     f"{r.get('localization_ok_fraction', '-')} | {error} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
