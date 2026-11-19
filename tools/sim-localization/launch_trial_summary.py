"""Summarize run_trials.sh folders as launch trials: lead, wheel slip and whether the car got through.

    python tools/sim-localization/launch_trial_summary.py label=DIR [label=DIR ...]

Per trial (first N seconds after a respawn): the largest distance the estimate was ahead of the car
along its direction of travel, wheel speed over true speed 1 to 1.5 s after the start, and an outcome:
``crash`` (collision counter rose), ``stall`` (car stood still for 2 s after moving, no collision) or ``ok``.
"""
import json
import math
import statistics as st
import sys
from pathlib import Path

import numpy as np


def trial(path):
    rows = [json.loads(line) for line in Path(path).open()]
    truth = [r for r in rows if r["k"] == "truth"]
    wheel = [r for r in rows if r["k"] == "wheel"]
    diag = [r for r in rows if r["k"] == "diag"]
    col = [r for r in rows if r["k"] == "collision"]
    if len(truth) < 50 or len(diag) < 20:
        return None
    t0 = truth[0]["tr"]
    tt = np.array([r["tr"] - t0 for r in truth]); tv = np.array([r["v"] for r in truth])
    tx = np.array([r["x"] for r in truth]); ty = np.array([r["y"] for r in truth])
    # outcome
    crash_t = next((c["tr"] - t0 for c in col if c["n"] > 0), None)
    moved = np.where(tv > 1.0)[0]
    stall_t = None
    if len(moved):
        after = np.where((tt > tt[moved[0]]) & (np.abs(tv) < 0.1))[0]
        for i in after:
            window = (tt >= tt[i]) & (tt < tt[i] + 2.0)
            if window.sum() > 20 and np.all(np.abs(tv[window]) < 0.1):
                stall_t = float(tt[i]); break
    outcome = "crash" if crash_t is not None else "stall" if stall_t is not None else "ok"
    # lead: estimate minus truth, projected on the direction the car is travelling
    lead = []
    for r in diag:
        d = r["d"]
        if "estimate_x_m" not in d:
            continue
        t = r["tr"] - t0
        if t > 6:
            break
        i = int(np.searchsorted(tt, t)); j = max(0, i - 15)
        dx, dy = tx[min(i, len(tx) - 1)] - tx[j], ty[min(i, len(ty) - 1)] - ty[j]
        n = math.hypot(dx, dy)
        if n < 0.1:
            continue
        lead.append(((d["estimate_x_m"] - d["truth_x_m"]) * dx + (d["estimate_y_m"] - d["truth_y_m"]) * dy) / n)
    wt = np.array([r["tr"] - t0 for r in wheel]); wv = np.array([r["v"] for r in wheel])
    m = (tt > 1.0) & (tt < 1.5); wm = (wt > 1.0) & (wt < 1.5)
    ratio = float(wv[wm].mean() / tv[m].mean()) if m.sum() > 5 and wm.sum() > 3 and tv[m].mean() > 0.5 else None
    return {"outcome": outcome, "event_s": crash_t if crash_t is not None else stall_t,
            "max_lead_m": max(lead) if lead else None, "wheel_ratio": ratio}


for item in sys.argv[1:]:
    label, folder = item.split("=", 1)
    res = [t for t in (trial(p) for p in sorted(Path(folder).glob("trial-*.jsonl"))) if t]
    n = len(res)
    out = {k: sum(r["outcome"] == k for r in res) for k in ("ok", "crash", "stall")}
    leads = [r["max_lead_m"] for r in res if r["max_lead_m"] is not None]
    ratios = [r["wheel_ratio"] for r in res if r["wheel_ratio"] is not None]
    print(f"== {label}: {n} trials  ok {out['ok']}  crash {out['crash']}  stall {out['stall']}")
    if leads:
        print(f"   max estimate lead in the first 6 s: median {st.median(leads):+.2f} m, "
              f"worst {max(leads):+.2f} m  | wheel/true speed at 1-1.5 s: "
              f"{('median %.2f' % st.median(ratios)) if ratios else 'n/a'}")
    print("   per trial:", ", ".join(f"{r['outcome']}{'' if r['event_s'] is None else '@%.0fs' % r['event_s']}"
                                      f"/{'-' if r['max_lead_m'] is None else '%.1f' % r['max_lead_m']}m" for r in res))
