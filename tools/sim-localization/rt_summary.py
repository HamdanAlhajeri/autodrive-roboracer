"""One-line summary per recording: laps, respawns, wall contacts, speed against plan.

    python tools/sim-localization/rt_summary.py name=file.jsonl [name=file.jsonl ...]
Run from the repository root (the map path is relative to it).
"""
import sys

sys.path.insert(0, "tools/sim-localization")
from analyze_pose_measure import analyze, load  # noqa: E402

print(f"{'run':10s} {'laps':>4s} {'resets':>6s} {'1st contacts':>12s} {'per drive-min':>13s} {'drive s':>7s} "
      f"{'lap time s':>10s} {'peak m/s':>8s} {'over plan >1':>12s}")
for item in sys.argv[1:]:
    name, file = item.split("=", 1)
    m = analyze(name, load(file))[0]
    laps, drive = m["laps_completed"], m["driving_s"]
    print(f"{name:10s} {laps:4d} {m['resets']:6d} {m['first_contacts']:12d} "
          f"{m['first_contacts_per_driving_minute']:13.2f} {drive:7.0f} "
          f"{(drive / laps if laps else float('nan')):10.1f} {m['peak_speed_mps']:8.1f} "
          f"{100 * m['overspeed']['share_over_1mps']:11.1f}%")
