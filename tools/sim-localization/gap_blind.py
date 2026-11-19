"""Split gap_test.sh results by how far the car travelled blind (while sensors were cut).

    python gap_blind.py DIR [DIR ...]

A trial recovers when the localization error 5 s after sensors return is below 0.8 m.
"""
import json
import math
import sys
from pathlib import Path


def trial(path):
    rows = [json.loads(line) for line in Path(path).open()]
    rows = [r for r in rows if r["err"] is not None and r["tx"] is not None]
    times = [r["t"] for r in rows]
    cut = [i for i in range(1, len(times)) if times[i] - times[i - 1] > 0.5]
    if not cut:
        return None
    i = cut[0]
    a, b = rows[i - 1], rows[i]
    blind = math.hypot(b["tx"] - a["tx"], b["ty"] - a["ty"])
    back = times[i]
    late = [r["err"] for r in rows if r["t"] >= back + 4.5]
    return {"blind_m": blind, "gap_s": times[i] - times[i - 1], "before": a["err"],
            "final": late[-1] if late else rows[-1]["err"]}


for folder in sys.argv[1:]:
    results = [t for t in (trial(p) for p in sorted(Path(folder).glob("trial-*.jsonl"))) if t]
    print(f"== {folder}: {len(results)} trials")
    for name, test in (("blind under 3 m", lambda t: t["blind_m"] < 3.0),
                       ("blind 3 m or more", lambda t: t["blind_m"] >= 3.0)):
        group = [t for t in results if test(t)]
        ok = [t for t in group if t["final"] < 0.8]
        print(f"   {name}: {len(ok)} of {len(group)} within 0.8 m 5 s after "
              f"(final errors {[round(t['final'], 2) for t in group]}, "
              f"blind {[round(t['blind_m'], 1) for t in group]})")
