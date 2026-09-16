import csv
import json
import math

import pytest

from avlite_autodrive.plot_recording import (
    export_csv, lap_report_title, load_track_map, measured_acceleration, plot_control_report,
    plot_lap_report, plot_run, read_samples,
    saved_speed_ceiling, series, track_map_points,
)


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


def test_lap_title_uses_final_feedback_without_claiming_a_full_lap():
    rows = [{"lap_count": 4, "collision_count": 12},
            {"lap_count": 5, "collision_count": 12}]
    summary = {"initial": rows[0], "final": {"lap_count": 5, "collision_count": 14},
               "started_utc": "2026-09-16T14:30:01+00:00"}
    title = lap_report_title(rows, summary)
    assert "lap count 4 → 5" in title
    assert "2 collisions" in title
    assert "16 September 2026" in title
    assert "one lap" not in title
    # Missing counters or counter resets must not become a zero-collision claim.
    assert "collision count unavailable" in lap_report_title([{}])
    assert "0 collisions" not in lap_report_title(
        [{"collision_count": 2}, {"collision_count": 0}]
    )


@pytest.mark.parametrize("direction", [1, -1])
def test_lap_report_preserves_missing_samples_resets_and_speed_overshoot(
    tmp_path, monkeypatch, direction
):
    import matplotlib.pyplot as plt
    from matplotlib.figure import Figure

    rows = [
        {"elapsed_s": 0, "x": 0, "y": 0, "speed": 0, "resets": 0},
        {"elapsed_s": 1, "x": 1, "y": 1, "speed": 0.6, "resets": 0},
        {"elapsed_s": 2, "x": 2, "y": 2, "speed": 0.4, "odom_age_s": 1, "resets": 0},
        {"elapsed_s": 3, "x": 3, "y": 3, "speed": 0.4, "resets": 0},
        {"elapsed_s": 4, "x": 10, "y": 10, "speed": 0.3, "resets": 1},
        {"elapsed_s": 5, "x": 11, "y": 11, "speed": 0.2, "resets": 1},
    ]
    for row in rows:
        row["speed"] *= direction
    captured = []
    original_savefig = Figure.savefig

    def capture(fig, *args, **kwargs):
        captured.append(fig)
        return original_savefig(fig, *args, **kwargs)

    monkeypatch.setattr(Figure, "savefig", capture)
    output = tmp_path / "lap.png"
    plot_lap_report(rows, output, speed_ceiling=0.5 if direction > 0 else None)
    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    fig = captured[0]
    path_ax, speed_ax = fig.axes
    assert math.isnan(path_ax.lines[0].get_xdata()[2])
    assert math.isnan(path_ax.lines[0].get_xdata()[4])
    assert math.isnan(speed_ax.lines[0].get_ydata()[2])
    if direction > 0:
        assert speed_ax.get_ylim()[1] > 0.6
        assert list(speed_ax.lines[1].get_ydata()) == [0.5, 0.5]
    else:
        assert speed_ax.get_ylim()[0] < -0.6
        assert speed_ax.get_ylim()[1] > 0
        assert len(speed_ax.lines) == 1  # No invented ceiling for an older recording.
    assert path_ax.collections[0].get_offsets().tolist() == [[0, 0]]
    assert path_ax.collections[1].get_offsets().tolist() == [[11, 11]]
    assert not plt.fignum_exists(fig.number)


def test_lap_report_rejects_recording_with_no_valid_positions(tmp_path):
    with pytest.raises(ValueError, match="No valid odometry positions"):
        plot_lap_report([{"elapsed_s": 0, "speed": 1}], tmp_path / "lap.png")


def test_speed_reference_only_comes_from_the_recordings_saved_profile(tmp_path):
    recording = tmp_path / "telemetry.jsonl"
    assert saved_speed_ceiling(recording) is None
    config = tmp_path / "config"
    config.mkdir()
    (config / "avlite.yaml").write_text(
        "shared_settings: driving.yaml\nc30_control:\n  c32_ego_max_velocity: ${speed_mps}\n"
    )
    (config / "driving.yaml").write_text("speed_mps: 0.5\nmax_throttle: 0.02\n")
    assert saved_speed_ceiling(recording) == 0.5


def test_acceleration_plot_does_not_treat_collision_or_reset_as_physical_braking():
    rows = [
        {"elapsed_s": 0, "speed": 2, "collision_count": 0, "resets": 0},
        {"elapsed_s": 0.1, "speed": 1.8, "collision_count": 0, "resets": 0},
        {"elapsed_s": 0.2, "speed": 0, "collision_count": 1, "resets": 0},
        {"elapsed_s": 0.3, "speed": 1, "collision_count": 1, "resets": 1},
        {"elapsed_s": 0.4, "speed": 2, "collision_count": 1, "resets": 1, "odom_age_s": 1},
    ]
    values = measured_acceleration(rows)
    assert values[1] == pytest.approx(-2)
    assert all(math.isnan(values[i]) for i in (0, 2, 3, 4))


def test_control_report_with_real_fields_and_missing_snapshots(tmp_path, monkeypatch):
    from matplotlib.figure import Figure

    rows = [
        {"elapsed_s": 0.1 * i, "speed": 2 - 0.1 * i, "throttle_command": 0,
         "target_velocity_mps": 1.5, "actuator_target_speed_mps": 1.4,
         "braking_requested": True, "lookahead_m": 1.3, "clearance_m": 2.0,
         "curvature_speed_limit_mps": 1.5, "clearance_speed_limit_mps": 2.1,
         "controller_loop_dt_s": 0.05, "controller_step_time_s": 0.003,
         "controller_lidar_age_s": 0.02, "avlite_acceleration": -1.5,
         "collision_count": 0, "resets": 0}
        for i in range(4)
    ]
    rows.append({"elapsed_s": 0.5, "speed": 0, "collision_count": 1, "resets": 1})
    captured = []
    original_savefig = Figure.savefig

    def capture(fig, *args, **kwargs):
        captured.append(fig)
        return original_savefig(fig, *args, **kwargs)

    monkeypatch.setattr(Figure, "savefig", capture)
    output = tmp_path / "control.png"
    plot_control_report(rows, output)
    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert len(captured[0].axes) == 6


def test_preview_report_separates_direction_pursuit_and_steering_feedback(tmp_path, monkeypatch):
    from matplotlib.figure import Figure

    rows = [
        {"elapsed_s": 0.1 * i, "speed": 1.0, "controller_diagnostics_age_s": 0.01,
         "gap_preview_requested_m": 1.5, "gap_preview_selected_m": 1.5 - 0.2 * i,
         "lookahead_m": 0.6, "clearance_m": 0.8, "gap_preview_fallback": i > 0,
         "target_bearing_raw_rad": -0.8, "target_bearing_rad": -0.4,
         "avlite_steer_rad": -0.5, "steering": -0.1}
        for i in range(3)
    ]
    rows[2]["controller_diagnostics_age_s"] = 0.8
    rows.append({"elapsed_s": 0.3, "speed": 1.0})
    captured = []
    original_savefig = Figure.savefig

    def capture(fig, *args, **kwargs):
        captured.append(fig)
        return original_savefig(fig, *args, **kwargs)

    monkeypatch.setattr(Figure, "savefig", capture)
    output = tmp_path / "control.png"
    plot_control_report(rows, output)
    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert len(captured[0].axes) == 8
    preview_ax, bearing_ax = captured[0].axes[6:]
    assert preview_ax.get_title() == "Gap preview and steering pursuit distance"
    preview_lines = {line.get_label(): line.get_ydata() for line in preview_ax.lines}
    assert list(preview_lines["Requested gap preview"][:2]) == [1.5, 1.5]
    assert list(preview_lines["Selected gap preview"][:2]) == [1.5, 1.3]
    assert list(preview_lines["Steering pursuit distance"][:2]) == [0.6, 0.6]
    assert all(math.isnan(value) for values in preview_lines.values() for value in values[2:])
    fallback = preview_ax.collections[0]
    assert fallback.get_label() == "Shorter-preview fallback"
    assert fallback.get_offsets().tolist() == [[0.1, 1.3]]
    assert bearing_ax.get_title() == "Chosen gap direction in vehicle frame"
    bearing_lines = {line.get_label(): line.get_ydata() for line in bearing_ax.lines}
    assert list(bearing_lines["Raw gap bearing"][:2]) == [-0.8, -0.8]
    assert list(bearing_lines["Filtered gap bearing"][:2]) == [-0.4, -0.4]
    assert all(math.isnan(value) for values in bearing_lines.values() for value in values[2:])
    steer_ax = captured[0].axes[4]
    assert list(steer_ax.lines[0].get_ydata()[:2]) == [-0.5, -0.5]
    assert list(steer_ax.lines[1].get_ydata()[:2]) == [-0.1, -0.1]


@pytest.mark.parametrize("fields, expected_panels", [
    ({"gap_preview_requested_m": 1.5}, 7),
    ({"target_bearing_raw_rad": -0.8}, 7),
    ({"gap_preview_requested_m": None, "target_bearing_raw_rad": None}, 6),
    ({"gap_preview_requested_m": 1.5, "controller_diagnostics_age_s": 0.8}, 6),
])
def test_partial_preview_recording_does_not_add_empty_panels(
    tmp_path, monkeypatch, fields, expected_panels
):
    from matplotlib.figure import Figure

    captured = []
    monkeypatch.setattr(Figure, "savefig", lambda fig, *args, **kwargs: captured.append(fig))
    plot_control_report([{"elapsed_s": 0, "speed": 1, **fields}], tmp_path / "control.png")
    assert len(captured[0].axes) == expected_panels


def observed_track(points=None):
    return {"version": 1, "frame_id": "world", "units": "m",
            "source": "lidar_hits_with_simulator_ground_truth", "resolution_m": 0.03,
            "points": points if points is not None else [[-2, -3], [4, 5]]}


@pytest.mark.parametrize("change", [
    {"version": 2}, {"version": True}, {"frame_id": "lidar"}, {"units": "cm"},
    {"points": [1, 2]}, {"points": [[1, 2, 3]]}, {"points": [[1, math.nan]]},
    {"points": [[1, math.inf]]}, {"points": [[True, 2]]}, {"points": None},
])
def test_track_map_rejects_invalid_coordinates_and_incompatible_frames(change):
    with pytest.raises(ValueError, match="track map"):
        track_map_points({**observed_track(), **change})


def test_missing_empty_and_invalid_track_maps_keep_older_runs_plottable(tmp_path, capsys):
    recording = tmp_path / "telemetry.jsonl"
    assert load_track_map(recording) is None
    sidecar = recording.with_suffix(".track.json")
    sidecar.write_text(json.dumps(observed_track([])))
    assert load_track_map(recording) is None
    sidecar.write_text('{"version":')
    assert load_track_map(recording) is None
    output = capsys.readouterr().out
    assert output.count("Track overlay unavailable") == 3
    assert "map contains no observed points" in output
    assert track_map_points(None) == []
    assert track_map_points(observed_track([])) == []


@pytest.mark.parametrize("report", ["lap", "debug"])
def test_track_overlay_preserves_world_coordinates_units_and_path_visibility(
    tmp_path, monkeypatch, report
):
    from matplotlib.figure import Figure

    rows = [
        {"elapsed_s": i, "x": i * 0.2, "y": i * 0.3, "speed": 0.5}
        for i in range(3)
    ]
    captured = []
    original_savefig = Figure.savefig

    def capture(fig, *args, **kwargs):
        captured.append(fig)
        return original_savefig(fig, *args, **kwargs)

    monkeypatch.setattr(Figure, "savefig", capture)
    output = tmp_path / f"{report}.png"
    renderer = plot_lap_report if report == "lap" else plot_run
    renderer(rows, output, track_map=observed_track())
    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    axis = captured[0].axes[0 if report == "lap" else 4]
    map_layer = axis.collections[0]
    assert map_layer.get_offsets().tolist() == [[-2, -3], [4, 5]]
    assert map_layer.get_label() == "Observed track / obstacles (LiDAR)"
    assert map_layer.get_zorder() < axis.lines[0].get_zorder()
    assert axis.get_aspect() == 1
    assert axis.get_xlim()[0] < -2 and axis.get_xlim()[1] > 4
    assert axis.get_ylim()[0] < -3 and axis.get_ylim()[1] > 5
    assert list(axis.lines[0].get_xdata()) == [0, 0.2, 0.4]


def test_incident_markers_use_pre_event_pose_and_do_not_invent_missing_locations(
    tmp_path, monkeypatch
):
    from matplotlib.figure import Figure

    rows = [
        {"elapsed_s": 0, "x": 1, "y": 2, "speed": 1, "collision_count": 0, "resets": 0},
        {"elapsed_s": 1, "x": 50, "y": 50, "speed": 0, "collision_count": 1, "resets": 1},
        {"elapsed_s": 2, "x": 51, "y": 51, "speed": 1, "collision_count": 1, "resets": 1,
         "odom_age_s": 1},
        {"elapsed_s": 3, "x": 60, "y": 60, "speed": 0, "collision_count": 2, "resets": 2},
        {"elapsed_s": 4, "x": 61, "y": 61, "speed": 0, "collision_count": 0, "resets": 2},
    ]
    captured = []
    monkeypatch.setattr(Figure, "savefig", lambda fig, *args, **kwargs: captured.append(fig))
    plot_lap_report(rows, tmp_path / "lap.png")
    axis = captured[0].axes[0]
    events = [layer for layer in axis.collections if layer.get_label().startswith("Before ")]
    assert len(events) == 2
    assert all(layer.get_offsets().tolist() == [[1, 2]] for layer in events)
    assert [text.get_text() for text in axis.texts] == ["collision / reset 1.0 s"]


def test_cli_explicit_map_overrides_sibling_for_both_graphs(tmp_path, monkeypatch, capsys):
    from avlite_autodrive import plot_recording

    recording = tmp_path / "telemetry.jsonl"
    recording.write_text(json.dumps({"elapsed_s": 0, "x": 0, "y": 0, "speed": 0}))
    recording.with_suffix(".track.json").write_text(json.dumps(observed_track([[1, 2]])))
    explicit = tmp_path / "earlier.track.json"
    selected = observed_track([[3, 4]])
    explicit.write_text(json.dumps(selected))
    captured = []
    monkeypatch.setattr("sys.argv", ["plot_recording", str(recording), "--lap-report",
                                     "--track-map", str(explicit)])
    monkeypatch.setattr(plot_recording, "plot_run",
                        lambda *args, **kwargs: captured.append(kwargs["track_map"]))
    monkeypatch.setattr(plot_recording, "plot_lap_report",
                        lambda *args, **kwargs: captured.append(kwargs["track_map"]))
    plot_recording.main()
    assert captured == [selected, selected]
    assert str(explicit) in capsys.readouterr().out
    assert load_track_map(recording)["points"] == [[1, 2]]
