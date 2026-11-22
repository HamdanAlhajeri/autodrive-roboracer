"""Summarize run_trials.sh results: laps, crashes and speed against the plan, per trial and per label.

    python summarize_trials.py OUT_DIR label=DIR [label=DIR ...] [--map config/maps/practice.json]

A trial is a crash when the collision counter rises; it survives if it never does. Writes
trials.json and a chart to OUT_DIR and prints a table.
"""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_pose_measure import RacingLine, arr, load


def trial(path):
    rec = load(path)
    line = RacingLine(rec["plan"][0]["d"]["path"], rec["plan"][0]["d"]["velocity"])
    truth = arr(rec["truth"], "tr", "x", "y", "yaw", "v", "w")
    t0 = truth[0, 0]
    truth[:, 0] -= t0
    v = truth[:, 4]
    moving = v > 0.5
    _, _, _, _, planned = line.project(truth[:, 1:3])
    col = arr(rec["collision"], "tr", "n")
    col[:, 0] -= t0
    hits = [float(col[i + 1, 0]) for i in np.flatnonzero(np.diff(col[:, 1]) > 0)]
    lap = arr(rec["lap"], "tr", "n")
    laps = int(np.sum(np.diff(lap[:, 1]) > 0))
    rates = []
    for i in range(0, len(truth) - 15, 5):
        j = i + 15
        dt = truth[j, 0] - truth[i, 0]
        if dt > 0 and v[i:j].min() > 2:
            rates.append(np.hypot(*(truth[j, 1:3] - truth[i, 1:3])) / dt / v[i:j].mean())
    wheel = arr(rec["wheel"], "tr", "v")
    wheel[:, 0] -= t0
    wv = np.interp(truth[:, 0], wheel[:, 0], wheel[:, 1])
    fast = v > 2
    # estimate error 5 s in, to see whether the localizer started healthy
    start_error = None
    diag = [(r["tr"] - t0, r["d"]) for r in rec["diag"] if r["d"].get("localization_error_m") is not None]
    for t, d in diag:
        if t >= 5:
            start_error = d["localization_error_m"]
            break
    over = (v - planned)[moving]
    return {"duration_s": float(truth[-1, 0]), "driving_s": float(moving.sum() / (len(truth) / truth[-1, 0])),
            "laps": laps, "crashed": bool(hits), "first_contact_s": hits[0] if hits else None,
            "contacts": len(hits), "peak_speed": float(v.max()),
            "share_over_1mps": float((over > 1).mean()) if len(over) else None,
            "realtime_factor": float(np.median(rates)) if rates else None,
            "wheel_to_true_speed": float(np.median(wv[fast] / v[fast])) if fast.any() else None,
            "estimate_error_at_5s_m": start_error}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out")
    parser.add_argument("groups", nargs="+")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    results = {}
    for item in args.groups:
        label, folder = item.split("=", 1)
        results[label] = [dict(trial(f), file=f.name) for f in sorted(Path(folder).glob("trial-*.jsonl"),
                                                                     key=lambda p: int(p.stem.split("-")[1]))]
    (out / "trials.json").write_text(json.dumps(results, indent=2))
    for label, rows in results.items():
        print(f"\n== {label}: {len(rows)} trials")
        print(" trial | laps | crash | 1st contact | driving s | peak m/s | >1 over plan | wheel/true | err@5s")
        for i, r in enumerate(rows, 1):
            fc = "-" if r["first_contact_s"] is None else f"{r['first_contact_s']:.1f} s"
            print(f" {i:5d} | {r['laps']:4d} | {'YES' if r['crashed'] else 'no ':5s} | {fc:>11s} | {r['driving_s']:9.1f} |"
                  f" {r['peak_speed']:8.2f} | {100 * (r['share_over_1mps'] or 0):10.0f}% |"
                  f" {r['wheel_to_true_speed'] or float('nan'):10.2f} | {r['estimate_error_at_5s_m'] if r['estimate_error_at_5s_m'] is None else round(r['estimate_error_at_5s_m'], 2)}")
        n = len(rows)
        print(f" -> crashed {sum(r['crashed'] for r in rows)}/{n}; laps total {sum(r['laps'] for r in rows)};"
              f" driving {sum(r['driving_s'] for r in rows):.0f} s")

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 130})
    fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.6))
    colors = ["#d62728", "#2ca02c", "#1f77b4", "#ff7f0e"]
    labels = list(results)
    axes[0].bar(labels, [sum(r["crashed"] for r in results[l]) / max(len(results[l]), 1) * 100 for l in labels],
                color=colors[: len(labels)])
    axes[0].set_title("trials that hit a wall (%)", fontsize=10)
    axes[1].bar(labels, [np.mean([r["laps"] for r in results[l]]) for l in labels], color=colors[: len(labels)])
    axes[1].set_title("laps per trial (mean)", fontsize=10)
    axes[2].bar(labels, [np.mean([r["wheel_to_true_speed"] or 0 for r in results[l]]) for l in labels],
                color=colors[: len(labels)])
    axes[2].axhline(1.0, color="black", ls="--", lw=1)
    axes[2].set_title("speed the controller sees / true speed", fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "trials-comparison.png")


if __name__ == "__main__":
    main()
