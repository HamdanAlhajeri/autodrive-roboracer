"""Write the recorded-data and localization-quality reports for an exported recording.

Runs without ROS, so the laptop can analyze an export copied from the Jetson:

    python -m avlite_roboracer.report log/jetson/run1/records.jsonl \
        --profile config/roboracer/hardware.yaml --map log/jetson/map1/map.yaml \
        --output log/jetson/run1/report
"""

import argparse
import json
from pathlib import Path

from .occupancy import OccupancyGrid
from .profile import HardwareProfile
from .quality import localization_report, read_reference, render_markdown
from .records import read_records
from .recording_check import check_recording


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("records", type=Path, help="JSONL exported by roboracer_record export")
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--map", type=Path, help="map_saver YAML for scan alignment")
    parser.add_argument("--reference", type=Path, help="CSV of t_s,x,y,yaw reference poses")
    parser.add_argument("--require-map-frame", action="store_true",
                        help="Fail if map -> odom is absent (localization/mapping replays)")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    profile = HardwareProfile.load(args.profile)
    records = read_records(args.records)
    grid = OccupancyGrid.load(args.map, profile.frame("map")) if args.map else None
    reference = read_reference(args.reference) if args.reference else None
    recording = check_recording(records, profile, require_map=args.require_map_frame)
    localization = None
    if args.require_map_frame or any(r["kind"] == "pose" for r in records):
        localization = localization_report(records, profile, grid, reference)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "recording_check.json").write_text(json.dumps(recording, indent=2) + "\n")
    if localization is not None:
        (args.output / "localization.json").write_text(json.dumps(localization, indent=2) + "\n")
    (args.output / "report.md").write_text(render_markdown(recording, localization))
    print(render_markdown(recording, localization))
    failed = recording["status"] == "fail" or (localization or {}).get("status") == "fail"
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
