"""Summarize gap_test.sh: localization error before the cut and 1, 2, 3 and 5 s after sensors return."""
import json
import sys
from pathlib import Path

folder, gap = Path(sys.argv[1]), float(sys.argv[2])
for path in sorted(folder.glob("trial-*.jsonl")):
    rows = [json.loads(line) for line in path.open()]
    rows = [r for r in rows if r["err"] is not None]
    if not rows:
        print(path.name, "no data"); continue
    # the cut starts about 2 s in; the diagnostics stop while the bridge is paused
    times = [r["t"] for r in rows]
    gaps = [(a, b) for a, b in zip(times, times[1:]) if b - a > 0.5]
    back = gaps[0][1] if gaps else None
    def near(t):
        c = [r for r in rows if abs(r["t"] - t) < 0.3]
        return round(c[0]["err"], 2) if c else None
    before = [r["err"] for r in rows if r["t"] < 1.8]
    if back is None:
        print(path.name, "no gap seen in the diagnostics"); continue
    after = {s: near(back + s) for s in (0.5, 1, 2, 3, 5)}
    print(f"{path.name}: error before {round(max(before), 2) if before else None} m, "
          f"gap {round(gaps[0][1] - gaps[0][0], 1)} s, after {after}, final {round(rows[-1]['err'], 2)} m")
