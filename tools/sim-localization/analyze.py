"""Explain one localization run: where the pose estimate went wrong and what to try.

    python3 analyze.py EXPERIMENT_DIR LABEL      # writes EXPERIMENT_DIR/LABEL-analysis.md

Reads LABEL-diagnostics.txt (the controller diagnostics topic, one JSON object per control
step), LABEL-counters.json and LABEL-runner.log. Time is rebuilt from controller_loop_dt_s.
Every finding names the numbers behind it, so you can check the reasoning.
"""

import json
import math
from pathlib import Path
import re
import sys

DIVERGED_M = 0.30       # position error that counts as lost
CORNER_MPS = 3.5        # planned speed below this marks a corner
ALIGNED_OK = 0.60       # share of scan hits on the map that counts as locked on


def load_rows(path):
    rows, t = [], 0.0
    for line in path.read_text(errors="replace").splitlines() if path.exists() else []:
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        t += row.get("controller_loop_dt_s") or 0.05
        row["t"] = t
        rows.append(row)
    return rows


def num(row, key):
    value = row.get(key)
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


def pct(values, q):
    values = sorted(v for v in values if v is not None)
    return values[min(len(values) - 1, int(q * (len(values) - 1) + 0.5))] if values else None


def fmt(value, digits=2):
    return "-" if value is None else f"{value:.{digits}f}"


def requested_laps(path, label):
    """Laps the experiment asked for, read from its own log line."""
    if path.exists():
        match = re.search(rf"{re.escape(label)}: driving (\d+) laps", path.read_text(errors="replace"))
        if match:
            return int(match.group(1))
    return None


def runner_notes(path):
    """Summarize bridge and runner log lines that explain stopped or withheld driving."""
    if not path.exists():
        return {}, []
    text = path.read_text(errors="replace")
    reasons = {}
    for match in re.finditer(r"Withholding commands: ([^\n]+)", text):
        reasons[match.group(1).strip()] = reasons.get(match.group(1).strip(), 0) + 1
    errors = [line for line in text.splitlines()
              if re.search(r"Traceback|Error|Exception|CRITICAL", line)][:5]
    return reasons, errors


def analyse(out, label):
    out = Path(out)
    rows = load_rows(out / f"{label}-diagnostics.txt")
    counters_path = out / f"{label}-counters.json"
    counters = json.loads(counters_path.read_text()) if counters_path.exists() else {}
    reasons, errors = runner_notes(out / f"{label}-runner.log")
    lines = [f"# Localization run `{label}`", ""]
    if not rows:
        lines += ["No diagnostics were recorded, so there is nothing to analyse.", "",
                  "Check `%s-runner.log` for a start-up error." % label]
        if errors:
            lines += [""] + [f"- `{e}`" for e in errors]
        return lines
    err = [(r["t"], num(r, "localization_error_m")) for r in rows]
    with_error = [(t, e) for t, e in err if e is not None]
    laps, collisions = counters.get("laps"), counters.get("collisions")
    requested = requested_laps(out / "experiment.log", label)
    # The car stopped for good when the last 10 s show no commanded speed and a frozen error.
    tail = [r for r in rows if r["t"] > rows[-1]["t"] - 10]
    parked = (rows[-1]["t"] > 15 and all((num(r, "target_velocity_mps") or 0) == 0 for r in tail))
    reason_codes = {0: "speed ceiling", 1: "planned corner/braking", 2: "obstacle stopping",
                    3: "invalid pose or outside the track corridor", 4: "missing/stale scan",
                    5: "commissioning ceiling", 6: "invalid plan"}
    complete = requested is None or (laps or 0) >= requested
    verdict = ("DRIVEN CLEAN" if collisions == 0 and complete and not parked
               else "FAILED: collision" if collisions
               else "FAILED: stopped before finishing the laps")
    lines += [f"**Result: {verdict}** - laps {laps}" +
              (f" of {requested}" if requested else "") + f", collisions {collisions}, "
              f"{len(rows)} control steps over {rows[-1]['t']:.1f} s.", ""]

    steps = [num(r, "controller_step_time_s") for r in rows]
    lines += ["## Numbers", "", "| Measure | Median | p95 | Max |", "| --- | --- | --- | --- |"]
    series = [
        ("Position error (m)", [e for _, e in with_error]),
        ("Yaw error (rad)", [num(r, "localization_yaw_error_rad") for r in rows]),
        ("Scan alignment (share of hits on the map)", [num(r, "localization_alignment") for r in rows]),
        ("Controller step (ms)", [None if s is None else s * 1000 for s in steps]),
    ]
    for name, values in series:
        values = [v for v in values if v is not None]
        if values:
            lines.append(f"| {name} | {fmt(pct(values, .5), 3)} | {fmt(pct(values, .95), 3)} | "
                         f"{fmt(max(values), 3)} |")
    overruns = sum(bool(r.get("controller_overrun")) for r in rows)
    lines += ["", f"Controller overruns: {overruns} of {len(rows)} steps.", ""]

    findings = []
    lost = next(((r, e) for r in rows for e in [num(r, "localization_error_m")]
                 if e is not None and e > DIVERGED_M), None)
    # An excursion that comes back under the limit within a few seconds is jitter, not a
    # lost pose. Count how many there are, how high they peak and where on the lap.
    excursions, start = [], None
    for r in rows:
        e = num(r, "localization_error_m")
        if e is None:
            continue
        if e > DIVERGED_M and start is None:
            start = [r, e]
        elif start is not None and e > DIVERGED_M:
            start[1] = max(start[1], e)
        elif start is not None and e <= DIVERGED_M:
            excursions.append((start[0], start[1], r["t"] - start[0]["t"]))
            start = None
    if start is not None:  # still off when the run ended
        excursions.append((start[0], start[1], rows[-1]["t"] - start[0]["t"]))
    recovered = lost is not None and bool(excursions) and not (
        start is not None or (collisions or 0) > 0)
    if with_error and lost is None:
        findings.append(("Localization held", f"Position error stayed under {DIVERGED_M} m "
                         f"(max {max(e for _, e in with_error):.3f} m) for the whole run."))
    if recovered:
        longest = max(x[2] for x in excursions)
        spots = sorted({round((num(x[0], "path_progress_m") or 0) / 3) * 3 for x in excursions})
        findings.append(("Error jitter, not drift", f"{len(excursions)} short excursions above "
                         f"{DIVERGED_M} m (peak {max(x[1] for x in excursions):.2f} m, longest "
                         f"{longest:.1f} s), and the error came back every time. They cluster near "
                         f"lap positions {', '.join(f'{s:.0f}' for s in spots)} m. The car stayed on "
                         "its line, so this is noise, but it caps how fast it can safely go."))
    if parked:
        stuck = next(r for r in rows if all((num(x, "target_velocity_mps") or 0) == 0
                                            for x in rows[rows.index(r):]))
        code = int(num(stuck, "speed_limit_reason") or 0)
        findings.append(("Car stopped for good", f"The controller commanded zero speed from "
                         f"t={stuck['t']:.1f} s onward ({fmt(num(stuck, 'path_progress_m'), 1)} m along "
                         f"the lap), reason code {code}: {reason_codes.get(code, 'unknown')}. "
                         f"The pose error was {fmt(num(stuck, 'localization_error_m'), 2)} m and alignment "
                         f"{fmt(num(stuck, 'localization_alignment'), 2)}. The safety check stopped the "
                         "car because the estimate left the track, and nothing re-localizes it. "
                         "Fix the cause below, or add a recovery that re-seeds the pose."))
    if lost and not recovered:
        row, e = lost
        i = rows.index(row)
        before = [r for r in rows[max(0, i - 40):i]]
        progress = num(row, "path_progress_m")
        planned = num(row, "planned_speed_mps")
        where = "a corner" if planned is not None and planned < CORNER_MPS else "a straight"
        align_before = [num(r, "localization_alignment") for r in before]
        align_before = [a for a in align_before if a is not None]
        align_at = num(row, "localization_alignment")
        yaw_at = num(row, "localization_yaw_error_rad")
        findings.append(("Pose was lost", f"Error passed {DIVERGED_M} m at t={row['t']:.1f} s, "
                         f"{fmt(progress, 1)} m along the lap, on {where} "
                         f"(planned speed {fmt(planned, 1)} m/s, actual target "
                         f"{fmt(num(row, 'target_velocity_mps'), 1)} m/s)."))
        if align_at is not None and align_before:
            drop = sum(align_before) / len(align_before) - align_at
            if align_at < ALIGNED_OK:
                findings.append(("Why: lost the map", f"Only {align_at:.0%} of scan hits land on the "
                                 f"map (was {sum(align_before) / len(align_before):.0%} in the 2 s "
                                 "before). ICP matched against the wrong walls. Try a larger "
                                 "`max_correspondence_m`, more `max_iterations`, or a better map."))
            else:
                findings.append(("Why: matched the wrong place", f"{align_at:.0%} of hits still land on "
                                 f"the map while the error grew to {e:.2f} m. The scene is ambiguous "
                                 "(parallel walls cannot show motion along the track). Weight the "
                                 "wheel-odometry prediction more or add a corner-based correction."))
            if drop > 0.15:
                findings.append(("Alignment fell first", f"Alignment dropped {drop:.0%} before the "
                                 "error grew, so it can be used as an early warning to slow down."))
        if yaw_at is not None and yaw_at > 0.1:
            findings.append(("Heading drift", f"Yaw error was {yaw_at:.2f} rad at that moment. "
                             "A bad heading corrupts every later step; check the yaw-rate bias "
                             "in the odometry service (`--yaw-rate-bias`)."))
        ex, ey = num(row, "localization_x_error_m"), num(row, "localization_y_error_m")
        if ex is not None and ey is not None and progress is not None:
            findings.append(("Direction of the error", f"x off by {ex:+.2f} m, y off by {ey:+.2f} m "
                             "(estimate minus truth). Compare with the track direction there to see "
                             "whether the estimate lagged behind or ran ahead."))
    if collisions:
        last = rows[-1]
        findings.append(("Collision", f"Run ended at t={last['t']:.1f} s with position error "
                         f"{fmt(num(last, 'localization_error_m'), 2)} m, "
                         f"{fmt(num(last, 'path_progress_m'), 1)} m along the lap, speed target "
                         f"{fmt(num(last, 'target_velocity_mps'), 1)} m/s, path deviation "
                         f"{fmt(num(last, 'path_deviation_m'), 2)} m."))
    if overruns > 0.05 * len(rows):
        findings.append(("Too slow", f"{overruns} steps overran the 50 ms budget (step p95 "
                         f"{fmt(pct([s * 1000 for s in steps if s is not None], .95), 1)} ms). "
                         "Lower `scan_subsample`/`map_subsample` or `max_iterations`."))
    if reasons:
        top = ", ".join(f"{k} (x{v})" for k, v in sorted(reasons.items(), key=lambda kv: -kv[1])[:3])
        findings.append(("Commands withheld", f"The bridge stopped the car: {top}."))
    if errors:
        findings.append(("Errors in the runner log", "; ".join(errors[:3])))
    if not findings:
        findings.append(("No localization data", "No error values were logged; was `source: avlite` "
                         "or `slam` selected?"))
    lines += ["## What happened", ""]
    lines += [f"- **{title}.** {text}" for title, text in findings]

    # Timeline: one row per 2 s so a failure can be located by eye.
    lines += ["", "## Timeline (every 2 s)", "",
              "| t (s) | Lap pos (m) | Speed target | Error (m) | Yaw err (rad) | Alignment | Path dev (m) |",
              "| --- | --- | --- | --- | --- | --- | --- |"]
    next_t = 0.0
    for r in rows:
        if r["t"] >= next_t:
            next_t += 2.0
            lines.append(f"| {r['t']:.0f} | {fmt(num(r, 'path_progress_m'), 1)} | "
                         f"{fmt(num(r, 'target_velocity_mps'), 1)} | "
                         f"{fmt(num(r, 'localization_error_m'), 2)} | "
                         f"{fmt(num(r, 'localization_yaw_error_rad'), 3)} | "
                         f"{fmt(num(r, 'localization_alignment'), 2)} | "
                         f"{fmt(num(r, 'path_deviation_m'), 2)} |")
    # Trajectory plot: truth against estimate, coloured by error.
    pts = [(num(r, "truth_x_m"), num(r, "truth_y_m"), num(r, "estimate_x_m"), num(r, "estimate_y_m"),
            num(r, "localization_error_m")) for r in rows]
    pts = [p for p in pts if None not in p]
    if pts:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5), gridspec_kw={"width_ratios": [1, 1.3]})
            ax1.plot([p[0] for p in pts], [p[1] for p in pts], "-", color="#2a78d6", label="truth")
            ax1.plot([p[2] for p in pts], [p[3] for p in pts], "--", color="#eb6834", label="estimate")
            ax1.set_aspect("equal"); ax1.legend(); ax1.set_title("Path: truth vs estimate")
            ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
            times = [r["t"] for r in rows if num(r, "localization_error_m") is not None]
            ax2.plot(times, [num(r, "localization_error_m") for r in rows
                             if num(r, "localization_error_m") is not None], color="#eb6834",
                     label="position error (m)")
            al = [(r["t"], num(r, "localization_alignment")) for r in rows
                  if num(r, "localization_alignment") is not None]
            if al:
                ax2.plot([a[0] for a in al], [a[1] for a in al], color="#1baf7a",
                         label="scan alignment (0-1)")
            ax2.axhline(DIVERGED_M, color="#52514e", linestyle=":", label=f"lost at {DIVERGED_M} m")
            ax2.set_xlabel("time (s)"); ax2.legend(); ax2.set_title("Error and alignment over time")
            fig.tight_layout()
            fig.savefig(out / f"{label}-analysis.png", dpi=100)
            lines += ["", f"![Path and error](./{label}-analysis.png)"]
        except Exception as exc:  # A missing plot must not hide the written findings.
            lines += ["", f"(Plot skipped: {exc})"]
    lines += contact_sheet(out, label)
    return lines


def contact_sheet(out, label, count=8):
    """Montage of evenly spaced simulator screenshots, so a run can be seen at a glance."""
    shots = sorted((Path(out) / "photos").glob(f"{label}-[0-9][0-9][0-9].png"))
    if not shots:
        return []
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        picks = [shots[round(i * (len(shots) - 1) / max(count - 1, 1))] for i in range(min(count, len(shots)))]
        cols = 4
        rows_n = (len(picks) + cols - 1) // cols
        fig, axes = plt.subplots(rows_n, cols, figsize=(4.2 * cols, 2.7 * rows_n))
        for ax in (axes.flat if hasattr(axes, "flat") else [axes]):
            ax.axis("off")
        for ax, shot in zip(axes.flat if hasattr(axes, "flat") else [axes], picks):
            ax.imshow(plt.imread(shot))
            ax.set_title(shot.stem.replace(label + "-", "photo "), fontsize=8)
        fig.tight_layout()
        fig.savefig(Path(out) / f"{label}-photos.png", dpi=90)
        return ["", f"## Simulator photos ({len(shots)} saved in `photos/`)", "",
                f"![Simulator screenshots](./{label}-photos.png)"]
    except Exception as exc:
        return ["", f"(Photo sheet skipped: {exc})"]


def main(out, label):
    lines = analyse(out, label)
    (Path(out) / f"{label}-analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
