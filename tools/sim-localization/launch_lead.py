"""Launch behaviour after each respawn: wheel speed against true speed, and how far the estimate leads.

    python tools/sim-localization/launch_lead.py name=file.jsonl [...]

Lead is measured along the first straight (the car drives toward negative y after the spawn), so
a positive lead means the estimate is further along the lap than the car.
"""
import json
import statistics as st
import sys

import numpy as np

for item in sys.argv[1:]:
    name, file = item.split("=", 1)
    rows = [json.loads(line) for line in open(file)]
    resets = [r["tr"] for r in rows if r["k"] == "reset"]
    truth = [r for r in rows if r["k"] == "truth"]
    wheel = [r for r in rows if r["k"] == "wheel"]
    diag = [r for r in rows if r["k"] == "diag"]
    tt = np.array([r["tr"] for r in truth]); tv = np.array([r["v"] for r in truth])
    wt = np.array([r["tr"] for r in wheel]); wv = np.array([r["v"] for r in wheel])
    print(f"== {name}: {len(resets)} launches")
    for lo, hi in ((0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, 4.0)):
        ratios, lead = [], []
        for t in resets:
            m = (tt > t + lo) & (tt < t + hi)
            wm = (wt > t + lo) & (wt < t + hi)
            if m.sum() > 5 and wm.sum() > 3 and tv[m].mean() > 0.5:
                ratios.append(wv[wm].mean() / tv[m].mean())
            d = [r["d"] for r in diag if t + lo < r["tr"] < t + hi and "estimate_y_m" in r["d"]]
            if d:
                lead.append(st.mean(x["truth_y_m"] - x["estimate_y_m"] for x in d))
        a = f"wheel/true {st.median(ratios):.2f} (n={len(ratios)})" if ratios else "wheel/true -"
        b = (f"lead median {st.median(lead):+.2f} m, max {max(lead):+.2f} m (n={len(lead)})" if lead else "lead -")
        print(f"  {lo}-{hi} s after respawn: {a} | {b}")
