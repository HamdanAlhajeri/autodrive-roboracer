"""Analyze measure_pose.py recordings: crashes, speed against the plan, braking and localization.

    python analyze_pose_measure.py OUT_DIR icp=RECORDING.jsonl [groundtruth=RECORDING.jsonl]

Each NAME=FILE is one recording; the first is the main one (it gets the localization charts) and
the others are comparison runs. Writes results.json and PNG charts to OUT_DIR. Every number
comes from the recordings; the only model is the bicycle relation yaw_rate = v*tan(steer)/L.

Definitions
- run: the time between two respawns (a respawn is a reset the recorder issued after the car
  stood still for several seconds, or the start of the recording);
- contact: a step up of the simulator's collision counter; the first contact of a run is the crash;
- overspeed: true speed minus the planned speed at the nearest point of the racing line;
- lost: after a respawn the estimate is more than 1 m from the truth while the car stands still.
"""

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

WHEELBASE = 0.324
SLACK_STOP_M, SLACK_WALL_M = 0.08, 0.13  # estimated from planner margin 0.371 m, envelope 0.29 m, radius 0.24 m
COLORS = {"icp": "#d62728", "groundtruth": "#1f77b4"}


def load(path):
    rows = {}
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            rows.setdefault(r["k"], []).append(r)
    return rows


def trim_to_first_spawn(rec, spawn, tol=0.35):
    """Drop everything before the car is first seen at the spawn pose (earlier resets did nothing)."""
    for r in rec["truth"]:
        if math.hypot(r["x"] - spawn[0], r["y"] - spawn[1]) < tol:
            cut = r["tr"] - 0.5
            break
    else:
        return rec
    return {k: [r for r in v if r["tr"] >= cut] for k, v in rec.items() if k != "plan"} | {"plan": rec["plan"]}


def arr(records, *keys):
    return np.array([[r[k] for k in keys] for r in records], dtype=float)


class RacingLine:
    def __init__(self, points, velocity):
        self.p, self.v = np.asarray(points, float), np.asarray(velocity, float)
        seg = np.roll(self.p, -1, axis=0) - self.p
        self.len = np.hypot(*seg.T)
        self.s = np.r_[0, np.cumsum(self.len)][:-1]
        self.total = float(self.len.sum())
        self.t = seg / np.maximum(self.len[:, None], 1e-9)
        self.n = np.c_[-self.t[:, 1], self.t[:, 0]]

    def project(self, xy):
        xy = np.asarray(xy, float)
        d = np.hypot(xy[:, None, 0] - self.p[None, :, 0], xy[:, None, 1] - self.p[None, :, 1])
        i = d.argmin(axis=1)
        return i, self.s[i], self.t[i], self.n[i], self.v[i]


def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else None


def analyze(name, rec, spawn=(0.80, 3.158)):
    """Return (metrics dict, working arrays for charts) for one recording."""
    full_duration = rec["truth"][-1]["tr"] - rec["truth"][0]["tr"]
    rec = trim_to_first_spawn(rec, spawn)
    line = RacingLine(rec["plan"][0]["d"]["path"], rec["plan"][0]["d"]["velocity"])
    truth = arr(rec["truth"], "tr", "x", "y", "yaw", "v", "w")
    t0 = truth[0, 0]
    truth[:, 0] -= t0
    dur = truth[-1, 0]
    i_line, station, tan, nrm, planned = line.project(truth[:, 1:3])
    v = truth[:, 4]
    moving = v > 0.5
    drive_s = float(moving.sum() / max(len(truth) / dur, 1))

    # ---- simulator real-time factor: displacement per wall second over the reported speed ------
    rt_series = []
    for i in range(0, len(truth) - 15, 5):
        j = i + 15
        dt_w = truth[j, 0] - truth[i, 0]
        if dt_w > 0 and truth[i:j, 4].min() > 2:
            rt_series.append((truth[i, 0], float(np.hypot(*(truth[j, 1:3] - truth[i, 1:3])) / dt_w
                                                 / truth[i:j, 4].mean())))
    rt = float(np.median([x for _, x in rt_series])) if rt_series else float("nan")
    wheel = arr(rec["wheel"], "tr", "v")
    wheel[:, 0] -= t0
    # ---- diagnostics ------------------------------------------------------------------
    diag = [(r["tr"] - t0, r["d"]) for r in rec["diag"]]
    dt_ = np.array([t for t, _ in diag])
    target = np.array([np.nan if d.get("target_velocity_mps") is None else d["target_velocity_mps"]
                       for _, d in diag])
    has_est = [d.get("truth_x_m") is not None and d.get("estimate_x_m") is not None for _, d in diag]
    # ---- runs --------------------------------------------------------------------------
    resets = [r["tr"] - t0 for r in rec.get("reset", [])]
    starts = [0.0] + resets
    ends = resets + [dur]
    col = arr(rec["collision"], "tr", "n")
    col[:, 0] -= t0
    contacts = [float(col[i + 1, 0]) for i in np.flatnonzero(np.diff(col[:, 1]) > 0)]
    first_contacts = []
    for a, b in zip(starts, ends):
        c = [t for t in contacts if a + 0.5 <= t < b]
        if c:
            first_contacts.append(c[0])
    at = lambda t: int(min(np.searchsorted(truth[:, 0], t), len(truth) - 1))
    fc = []
    for t in first_contacts:
        j = at(t - 0.5)
        k = np.searchsorted(dt_, t - 0.5)
        k = int(min(k, len(dt_) - 1))
        entry = {"t": t, "station_m": float(station[j]), "x": float(truth[j, 1]), "y": float(truth[j, 2]),
                 "speed_mps": float(v[j]), "planned_speed_mps": float(planned[j]),
                 "wheel_speed_mps": float(np.interp(t - 0.5, wheel[:, 0], wheel[:, 1])),
                 "target_speed_mps": None if not np.isfinite(target[k]) else float(target[k])}
        if has_est[k]:
            d = diag[k][1]
            entry["estimate_error_m"] = float(d["localization_error_m"])
            entry["alignment"] = d.get("localization_alignment")
        fc.append(entry)

    lap_steps = int(np.sum(np.diff(arr(rec["lap"], "n")[:, 0]) > 0))
    m = {"realtime_factor": rt, "laps_completed": lap_steps, "recording_s": full_duration, "duration_s": dur, "driving_s": drive_s, "resets": len(resets), "contacts_total": len(contacts),
         "first_contacts": len(fc), "peak_speed_mps": float(v.max()),
         "first_contacts_per_driving_minute": len(fc) / max(drive_s / 60, 1e-9),
         "first_contact_details": fc}
    over = (v - planned)[moving]
    m["overspeed"] = {"median_mps": float(np.median(over)), "p95_mps": pct(over, 95),
                      "share_over_1mps": float((over > 1).mean()), "share_over_2mps": float((over > 2).mean())}
    if fc:
        m["at_first_contact"] = {
            "median_speed": float(np.median([e["speed_mps"] for e in fc])),
            "median_planned": float(np.median([e["planned_speed_mps"] for e in fc])),
            "over_plan_by_1mps": int(sum(e["speed_mps"] - e["planned_speed_mps"] > 1 for e in fc)),
            "median_estimate_error_m": (float(np.median([e["estimate_error_m"] for e in fc
                                                        if "estimate_error_m" in e]))
                                        if any("estimate_error_m" in e for e in fc) else None)}

    # ---- braking and acceleration ------------------------------------------------------
    cmd = arr(rec["cmd"], "tr", "a", "steer")
    cmd[:, 0] -= t0
    grid = np.arange(0, dur - 0.1, 0.1)
    vg = np.interp(grid, truth[:, 0], v)
    ag = np.gradient(np.convolve(vg, np.ones(3) / 3, "same"), 0.1)
    cag = np.interp(grid, cmd[:, 0], cmd[:, 1])
    brake = {}
    for lo, hi in ((2, 4), (4, 5.5), (5.5, 7), (7, 9)):
        s = (cag < -2.0) & (vg >= lo) & (vg < hi)
        if s.sum() >= 5:
            brake[f"{lo}-{hi}"] = {"seconds": float(s.sum() * 0.1), "commanded_median": float(np.median(cag[s])),
                                    "achieved_median_wall": float(np.median(ag[s])),
                                    "achieved_median_sim": float(np.median(ag[s]) / rt), "achieved_p25_sim": pct(ag[s] / rt, 25)}
    m["braking_by_speed_mps"] = brake
    acc = (cag > 1.0) & (vg > 0.3) & (vg < 6)
    m["acceleration"] = {"commanded_median": float(np.median(cag[acc])) if acc.any() else None,
                         "achieved_median_wall": float(np.median(ag[acc])) if acc.any() else None,
                         "achieved_median_sim": float(np.median(ag[acc]) / rt) if acc.any() else None}

    # ---- control dead time: steering command -> yaw rate ----------------------------------
    g = np.arange(max(cmd[0, 0], 0) + 1, min(cmd[-1, 0], dur) - 1, 0.01)
    steer = np.interp(g, cmd[:, 0], cmd[:, 2])
    vv = np.interp(g, truth[:, 0], v)
    yaw = np.interp(g, truth[:, 0], np.unwrap(truth[:, 3]))
    k = np.ones(10) / 10
    expected = np.convolve(vv * np.tan(steer) / WHEELBASE, k, "same")
    measured = np.convolve(np.gradient(yaw, 0.01), k, "same")
    use = vv > 1.0
    a_, b_ = np.where(use, expected, 0.0), np.where(use, measured, 0.0)
    a_, b_ = a_ - a_.mean(), b_ - b_.mean()
    curve = []
    for lag in range(0, 51):
        x, y = (a_[: len(a_) - lag], b_[lag:]) if lag else (a_, b_)
        curve.append((x * y).sum() / math.sqrt((x * x).sum() * (y * y).sum()))
    curve = np.array(curve)
    kk = int(curve.argmax())
    shift = 0.0
    if 0 < kk < len(curve) - 1:
        y0, y1, y2 = curve[kk - 1], curve[kk], curve[kk + 1]
        shift = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
    m["control_dead_time"] = {"lag_ms_wall": float((kk + shift) * 10), "lag_ms_sim": float((kk + shift) * 10 * rt),
                              "peak_correlation": float(curve.max())}
    arrays = {"rt_series": rt_series, "wheel": wheel, "truth": truth, "station": station, "planned": planned, "line": line, "fc": fc,
              "contacts": contacts, "starts": starts, "ends": ends, "curve": curve, "grid": grid,
              "vg": vg, "ag": ag, "cag": cag}

    # ---- localization (only when the recording has estimate and truth) ---------------------
    if any(has_est):
        sel = [i for i, h in enumerate(has_est) if h]
        td = dt_[sel]
        est = np.array([[diag[i][1]["estimate_x_m"], diag[i][1]["estimate_y_m"]] for i in sel])
        tru = np.array([[diag[i][1]["truth_x_m"], diag[i][1]["truth_y_m"]] for i in sel])
        err = np.hypot(*(est - tru).T)
        al = np.array([diag[i][1].get("localization_alignment") or 0 for i in sel])
        spd = np.interp(td, truth[:, 0], v)
        j_line, _, tn_, nn_, _ = line.project(tru)
        e = est - tru
        along, cross = (e * tn_).sum(1), (e * nn_).sum(1)
        # time since the last respawn
        since = np.array([t - max(s for s in starts if s <= t + 1e-9) for t in td])
        lost_cases = []
        for r in resets + [0.0]:
            k2 = np.searchsorted(td, r + 3.0)
            if k2 < len(td) and td[k2] < r + 5 and spd[k2] < 0.3:
                lost_cases.append({"t": float(r), "error_m": float(err[k2])})
        resets_after = [c for c in lost_cases if c["t"] > 0]
        m["lost_after_respawn"] = {"respawns_checked": len(resets_after),
                                    "lost_error_over_1m": int(sum(c["error_m"] > 1 for c in resets_after)),
                                    "cases": resets_after}
        healthy = ~((err > 1.5) & (al < 0.5))  # drop frames where the estimate is already lost
        mv_d = spd > 0.5
        for label, mask in (("launch_0_to_6s_after_respawn", mv_d & (since < 6) & healthy),
                            ("later_than_6s_after_respawn", mv_d & (since >= 6) & healthy)):
            if mask.sum() > 10:
                m.setdefault("error", {})[label] = {
                    "n": int(mask.sum()), "magnitude_median_m": float(np.median(err[mask])),
                    "magnitude_p95_m": pct(err[mask], 95), "magnitude_max_m": float(err[mask].max()),
                    "along_median_m": float(np.median(along[mask])), "along_p95_abs_m": pct(np.abs(along[mask]), 95),
                    "cross_p95_abs_m": pct(np.abs(cross[mask]), 95)}
        late = mv_d & (since >= 6) & healthy
        bins = []
        for lo, hi in ((0.5, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 9)):
            s = late & (spd >= lo) & (spd < hi)
            if s.sum() >= 20:
                bins.append({"speed": [lo, hi], "n": int(s.sum()), "median": float(np.median(err[s])),
                             "p95": pct(err[s], 95), "along_p95": pct(np.abs(along[s]), 95),
                             "cross_p95": pct(np.abs(cross[s]), 95)})
        m["error_by_speed_after_launch"] = bins
        # delay of the estimate against the received truth, after the launch transient
        lags = np.arange(0, 0.41, 0.01)
        curve2 = []
        for lag in lags:
            px = np.interp(td - lag, truth[:, 0], truth[:, 1])
            py = np.interp(td - lag, truth[:, 0], truth[:, 2])
            curve2.append(float(np.mean(np.hypot(est[late, 0] - px[late], est[late, 1] - py[late]))))
        b = int(np.argmin(curve2))
        m["estimate_lag_after_launch"] = {"best_lag_ms": float(lags[b] * 1000), "mean_error_at_0_ms_m": curve2[0],
                                           "mean_error_at_best_m": curve2[b]}
        arrays.update({"td": td, "err": err, "along": along, "cross": cross, "since": since, "spd": spd,
                       "al": al, "lags": lags, "lagcurve": curve2, "lost": lost_cases, "healthy": healthy})
        # odometry speed versus true speed
    wv = np.interp(truth[:, 0], wheel[:, 0], wheel[:, 1])
    mv = v > 1
    m["wheel_speed_ratio_median"] = float(np.median(wv[mv] / v[mv]))  # what the controller sees / true
    la = np.abs(v * truth[:, 5])
    m["wheel_speed_ratio_by_lateral_acc"] = {
        f"{lo}-{hi}": {"n": int(s.sum()), "median": float(np.median(wv[s] / v[s]))}
        for lo, hi in ((0, 1), (1, 3), (3, 99))
        for s in [mv & (la >= lo) & (la < hi)] if s.sum() > 30}
    # launch: wheel speed vs true speed while accelerating from a stop
    return m, arrays


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out")
    parser.add_argument("recordings", nargs="+", help="name=file.jsonl")
    parser.add_argument("--map", default="config/maps/practice.json")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = {}
    for item in args.recordings:
        name, file = item.split("=", 1)
        runs[name] = analyze(name, load(file))
    (out / "results.json").write_text(json.dumps({k: v[0] for k, v in runs.items()}, indent=2))

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 130})
    names = list(runs)
    main_name = names[0]
    m0, a0 = runs[main_name]
    line = a0["line"]
    bounds = json.loads(Path(args.map).read_text())
    left, right = np.array(bounds["LeftBound"]), np.array(bounds["RightBound"])
    RED, BLUE, GRAY = COLORS["icp"], COLORS["groundtruth"], "#888888"
    label = {"icp": "AVLite ICP localization", "groundtruth": "simulator ground truth"}

    # 1. speed against plan around the lap, every run
    fig, ax = plt.subplots(figsize=(9, 4))
    for name in reversed(names):
        m_, a_ = runs[name]
        tr, st = a_["truth"], a_["station"]
        mv = tr[:, 4] > 0.5
        ax.scatter(st[mv], tr[mv, 4], s=2, alpha=0.35, color=COLORS.get(name, GRAY), label=label.get(name, name))
    ax.plot(line.s, line.v, color="black", lw=2, label="planned speed")
    if a0["fc"]:
        ax.scatter([e["station_m"] for e in a0["fc"]], [e["speed_mps"] for e in a0["fc"]], marker="X", s=110,
                   color="black", edgecolor="white", zorder=5, label="first wall contact")
    ax.set_xlabel("position along the racing line (m)")
    ax.set_ylabel("true speed (simulator m/s)")
    ax.set_title("With ICP the car enters the bottom corner at about twice the planned speed")
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], marker="o", ls="", color=COLORS.get(n, GRAY), label=label.get(n, n)) for n in names]
    handles += [Line2D([], [], color="black", lw=2, label="planned speed"),
                Line2D([], [], marker="X", ls="", color="black", ms=9, label="first wall contact")]
    ax.legend(handles=handles, frameon=False, fontsize=8, loc="upper right")
    fig.tight_layout(); fig.savefig(out / "fig1-speed-vs-plan.png"); plt.close(fig)

    # 2. what the controller sees at each crash
    if a0["fc"]:
        fig, ax = plt.subplots(figsize=(8.5, 3.9))
        x = np.arange(len(a0["fc"]))
        ax.bar(x - 0.27, [e["speed_mps"] for e in a0["fc"]], 0.27, color=RED, label="true speed")
        ax.bar(x, [e["wheel_speed_mps"] for e in a0["fc"]], 0.27, color=GRAY, label="speed the controller sees (wheel odometry)")
        ax.bar(x + 0.27, [e["target_speed_mps"] or 0 for e in a0["fc"]], 0.27, color="black", label="speed it is aiming for")
        ax.set_xticks(x, [f"crash {i + 1}" for i in x])
        ax.set_ylabel("m/s (half a second before the first wall contact)")
        ax.set_title("The controller believes it is near its target speed while the car is far faster")
        ax.legend(frameon=False, fontsize=8, loc="upper right")
        fig.tight_layout(); fig.savefig(out / "fig2-controller-view-at-crash.png"); plt.close(fig)

    # 3. simulator real-time factor
    fig, ax = plt.subplots(figsize=(8.5, 3.6))
    for name in names:
        s = np.array(runs[name][1]["rt_series"])
        if len(s):
            ax.scatter(s[:, 0], s[:, 1], s=4, alpha=0.5, color=COLORS.get(name, GRAY),
                       label=f"{label.get(name, name)} (median {runs[name][0]['realtime_factor']:.2f})")
    ax.axhline(1.0, color="black", lw=1, ls="--")
    ax.set_ylim(0, 1.25)
    ax.text(ax.get_xlim()[1], 1.02, "real time", ha="right", va="bottom", fontsize=8)
    ax.set_xlabel("seconds into the recording")
    ax.set_ylabel("simulated seconds per wall-clock second")
    ax.set_title("The simulator runs at about half of real time")
    ax.legend(frameon=False, fontsize=8, loc="lower right", markerscale=3)
    fig.tight_layout(); fig.savefig(out / "fig3-realtime-factor.png"); plt.close(fig)

    # 4. comparison between recordings
    if len(names) > 1:
        fig, axes = plt.subplots(1, 4, figsize=(11.5, 3.5))
        cols = [COLORS.get(n, GRAY) for n in names]
        short = [{"icp": "ICP", "groundtruth": "ground truth"}.get(n, n) for n in names]
        for ax, key, title in zip(axes, ("laps_completed", "first_contacts", "peak_speed_mps", "driving_s"),
                                  ("laps completed", "crashes (first contacts)", "peak speed (m/s)",
                                   "seconds spent driving")):
            ax.bar(short, [runs[n][0][key] for n in names], color=cols)
            ax.set_title(title, fontsize=10)
        axes[2].axhline(float(line.v.max()), color="black", ls="--", lw=1)
        fig.suptitle("Same plan, same simulator, 5 minutes each: only the pose source differs", fontsize=11)
        fig.tight_layout(); fig.savefig(out / "fig4-comparison.png"); plt.close(fig)

    # 5. crash map
    fig, ax = plt.subplots(figsize=(5, 7))
    for ring in (left, right):
        ax.plot(ring[:, 0], ring[:, 1], color="#999999", lw=1.2)
    ax.plot(line.p[:, 0], line.p[:, 1], color=BLUE, lw=0.8, ls="--", label="racing line")
    if a0["fc"]:
        ax.scatter([e["x"] for e in a0["fc"]], [e["y"] for e in a0["fc"]], marker="X", s=110, color=RED,
                   edgecolor="white", zorder=5, label=f"first wall contact ({len(a0['fc'])})")
    ax.set_aspect("equal"); ax.set_axis_off()
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    ax.set_title("Where the car first hits a wall (ICP run)")
    fig.tight_layout(); fig.savefig(out / "fig5-crash-map.png"); plt.close(fig)

    # 6. braking: commanded against achieved, in simulator time
    fig, ax = plt.subplots(figsize=(7, 3.8))
    # only runs without wall contacts: a crash looks like enormous braking
    groups = [(n, runs[n][0]["braking_by_speed_mps"]) for n in names
              if runs[n][0]["braking_by_speed_mps"] and runs[n][0]["contacts_total"] == 0]
    keys = sorted({k for _, b in groups for k in b}, key=lambda k: float(k.split("-")[0]))
    x = np.arange(len(keys))
    ax.bar(x - 0.3, [3.5] * len(keys), 0.2, color="#bbbbbb", label="commanded / assumed by the plan")
    for gi, (n, b) in enumerate(groups):
        ax.bar(x + 0.1 + 0.2 * gi, [-b[k]["achieved_median_sim"] if k in b else 0 for k in keys], 0.2,
               color=COLORS.get(n, GRAY), label=f"achieved, {label.get(n, n)}")
    ax.set_xticks(x, [k + " m/s" for k in keys])
    ax.set_ylabel("deceleration (simulator m/s²)")
    ax.set_title("Braking when 3.5 m/s² is commanded (crash-free run)")
    ax.legend(frameon=False, fontsize=8)
    fig.tight_layout(); fig.savefig(out / "fig6-braking.png"); plt.close(fig)

    # 7. launch transient and loss after reset (ICP only)
    if "err" in a0:
        fig, axes = plt.subplots(2, 1, figsize=(8.5, 5.2), sharex=True)
        td, since, err, along, spd, al, healthy = (a0[k] for k in ("td", "since", "err", "along", "spd", "al", "healthy"))
        sel = healthy & (since < 12)
        sc = axes[0].scatter(since[sel], along[sel], c=spd[sel], s=4, cmap="viridis")
        axes[0].axhline(0, color="black", lw=0.8)
        axes[0].set_ylabel("along-track error (m)\n(+ = estimate ahead of car)")
        axes[0].set_title("After each launch the estimate runs 1-3 m ahead of the car, then snaps back")
        fig.colorbar(sc, ax=axes[0], label="true speed (m/s)")
        axes[1].scatter(since[sel], np.abs(err[sel]), s=4, color="#555555")
        axes[1].axhline(SLACK_WALL_M, color=RED, ls="--", lw=1)
        axes[1].text(11.8, SLACK_WALL_M, "~13 cm to a wall", color=RED, ha="right", va="bottom", fontsize=8)
        axes[1].set_ylabel("|error| (m)"); axes[1].set_xlabel("seconds since respawn")
        fig.tight_layout(); fig.savefig(out / "fig7-launch-error.png"); plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 3.4))
        cases = [c for c in a0["lost"] if c["t"] > 0]
        ax.bar(range(1, len(cases) + 1), [max(c["error_m"], 0.01) for c in cases],
               color=[RED if c["error_m"] > 1 else "#2ca02c" for c in cases])
        ax.set_yscale("log"); ax.axhline(1, color="black", lw=0.8, ls="--")
        ax.set_xlabel("respawn after a crash"); ax.set_ylabel("estimate error 3 s later (m)")
        n_lost = sum(c["error_m"] > 1 for c in cases)
        ax.set_title(f"After a reset the localizer starts lost in {n_lost} of {len(cases)} cases")
        fig.tight_layout(); fig.savefig(out / "fig8-lost-after-reset.png"); plt.close(fig)

    print(json.dumps({n: {k: r[0][k] for k in r[0] if k not in ("first_contact_details",)} for n, r in runs.items()},
                     indent=1)[:200])


if __name__ == "__main__":
    main()
