"""Score a closed-loop run: ICP pose from the car's own log against the simulated true path.

Usage: python tools/jazzy-sim/analyze_closed.py tools/jazzy-sim/closed-run1 [--png out.png]
Reads <run>/truth.csv (fake car ground truth) and <run>/data/sessions/*/icp_trace.csv (the
supervisor's ICP poses while racing), matches them by time and prints error statistics.
"""
import argparse
import glob
import math
import sys
from pathlib import Path

import numpy as np


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run")
    parser.add_argument("--png")
    parser.add_argument("--mapping", action="store_true", help="score the mapping lap (slam_trace.csv)")
    args = parser.parse_args()
    run = Path(args.run)
    trace_name = "slam_trace.csv" if args.mapping else "icp_trace.csv"
    traces = sorted(glob.glob(str(run / "data" / "sessions" / "*" / trace_name)))
    if not traces:
        sys.exit(f"no icp_trace.csv under {run}/data/sessions (did the run reach READY?)")
    truth, icp = load(run / "truth.csv"), load(traces[-1])
    if icp.size < 5:
        sys.exit("icp_trace.csv has too few rows")
    t = icp["stamp_s"]
    tx = np.interp(t, truth["t_s"], truth["x"])
    ty = np.interp(t, truth["t_s"], truth["y"])
    tyaw = np.interp(t, truth["t_s"], np.unwrap(truth["yaw"]))
    err = np.hypot(icp["x"] - tx, icp["y"] - ty)
    yaw_err = np.abs(np.arctan2(np.sin(icp["yaw"] - tyaw), np.cos(icp["yaw"] - tyaw)))
    # The map frame comes from SLAM and can sit a little off the simulator's true frame, so also
    # score after the best single rigid transform (what matters for racing is consistency).
    a, b = np.c_[icp["x"], icp["y"]], np.c_[tx, ty]
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1] *= -1
        rot = vt.T @ u.T
    fit_err = np.hypot(*(((a - ca) @ rot.T + cb) - b).T)
    alignment = icp["alignment"][np.isfinite(icp["alignment"])]
    print(f"ICP poses {len(t)} over {t[-1] - t[0]:.0f} s")
    print("position error m: median %.3f  p95 %.3f  max %.3f  final %.3f"
          % (np.median(err), np.percentile(err, 95), err.max(), err[-1]))
    print("after one rigid fit (map-frame offset removed): median %.3f  p95 %.3f  max %.3f"
          % (np.median(fit_err), np.percentile(fit_err, 95), fit_err.max()))
    print("heading error rad: median %.3f  p95 %.3f  max %.3f"
          % (np.median(yaw_err), np.percentile(yaw_err, 95), yaw_err.max()))
    if alignment.size:
        print("scan alignment: median %.2f  min %.2f  below 0.5 for %.1f%% of scans"
              % (np.median(alignment), alignment.min(), 100 * np.mean(alignment < 0.5)))
    print("recoveries:", int(icp["recoveries"][-1]))
    drift = np.polyfit(t - t[0], err, 1)[0] * 60
    print("error trend: %+.3f m per minute" % drift)
    if args.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
        axes[0].plot(t - t[0], err, color="#2a78d6", lw=1.2)
        axes[0].axhline(0.3, color="#888", ls=":")
        axes[0].set_xlabel("time racing (s)"); axes[0].set_ylabel("position error (m)")
        axes[0].set_title("ICP pose error while racing", loc="left")
        axes[1].plot(tx, ty, color="#eb6834", lw=1.4, label="true")
        axes[1].plot(icp["x"], icp["y"], color="#2a78d6", lw=0.8, label="ICP")
        axes[1].set_aspect("equal"); axes[1].legend(); axes[1].set_title("Path", loc="left")
        fig.tight_layout(); fig.savefig(args.png, dpi=110)
        print("chart:", args.png)


if __name__ == "__main__":
    main()
