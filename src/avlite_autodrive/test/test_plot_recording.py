import csv
import json
import math

import pytest

from avlite_autodrive.plot_recording import export_csv, plot_run, read_samples, series


def test_export_preserves_timestamp_and_fields_arriving_after_startup(tmp_path):
    rows = [
        {"elapsed_s": 0, "timestamp_utc": "2026-09-16T12:00:00.000+00:00"},
        {"elapsed_s": 0.1, "speed": 0.5, "steering": None},
    ]
    recording = tmp_path / "run.jsonl"
    recording.write_text("\n".join(json.dumps(row) for row in rows))
    loaded = read_samples(recording)
    destination = tmp_path / "run.csv"
    export_csv(loaded, destination)
    with destination.open(newline="") as stream:
        csv_rows = list(csv.DictReader(stream))
    assert csv_rows[0]["timestamp_utc"] == rows[0]["timestamp_utc"]
    assert csv_rows[0]["speed"] == ""
    assert csv_rows[1]["speed"] == "0.5"
    assert csv_rows[1]["steering"] == ""


def test_stale_or_missing_speed_is_a_gap_not_zero():
    rows = [
        {},
        {"speed": 0.5, "odom_age_s": 0.01},
        {"speed": 0.5, "odom_age_s": 0.8},
        {"speed": None},
        {"speed": float("nan")},
    ]
    values = series(rows, "speed", "odom_age_s")
    assert values[1] == 0.5
    assert all(math.isnan(values[i]) for i in (0, 2, 3, 4))


def test_render_legacy_recording_without_timestamp_or_freshness_fields(tmp_path):
    rows = [
        {"elapsed_s": 0.1 * i, "speed": 0.5, "x": 0.05 * i, "y": 0,
         "throttle_command": 0.02, "steering": 0.1, "avlite_steer_rad": 0.1,
         "lap_count": 0, "collision_count": int(i > 5), "resets": 0}
        for i in range(10)
    ]
    output = tmp_path / "graph.png"
    plot_run(rows, output)
    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_recording_without_sensors_does_not_produce_misleading_graph(tmp_path):
    with pytest.raises(ValueError, match="No odometry recorded"):
        plot_run([{"elapsed_s": 0.0}], tmp_path / "graph.png")


def test_empty_recording_reports_error(tmp_path):
    path = tmp_path / "empty.jsonl"
    path.write_text("")
    with pytest.raises(ValueError, match="recording is empty"):
        read_samples(path)
