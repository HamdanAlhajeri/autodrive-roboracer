import http.client
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from avlite_autodrive.pose_view import (
    PoseHub, PoseViewConfig, PoseViewServer, build_snapshot, geometry_from_map, pose_view_config,
)


def test_view_is_off_unless_enabled_and_settings_are_validated():
    assert pose_view_config({}) is None
    assert pose_view_config({"visualization": {"pose_view": False}}) is None
    settings = pose_view_config({"visualization": {"pose_view": True}})
    assert (settings.port, settings.error_alert_m) == (8765, 0.5)
    settings = pose_view_config({"visualization": {"pose_view": True, "port": 9000,
                                                   "error_alert_m": 0.25}})
    assert (settings.port, settings.error_alert_m) == (9000, 0.25)
    for block, message in [({"pose_view": "yes"}, "pose_view"),
                           ({"pose_view": True, "port": 80}, "port"),
                           ({"pose_view": True, "port": True}, "port"),
                           ({"pose_view": True, "error_alert_m": 0}, "error_alert_m"),
                           ({"pose_view": True, "error_alert_m": math.nan}, "error_alert_m"),
                           ({"pose_view": True, "prot": 8765}, "Unknown visualization setting")]:
        with pytest.raises(ValueError, match=message):
            pose_view_config({"visualization": block})
    with pytest.raises(ValueError, match="mapping"):
        pose_view_config({"visualization": [1]})


def test_snapshot_reports_position_and_signed_wrapped_yaw_error():
    shot = build_snapshot(2.0, (3.0, 4.0, -3.1), (0.0, 0.0, 3.1), alignment=0.9, hits=120,
                          speed=2.5)
    assert shot["err_m"] == pytest.approx(5.0)
    assert shot["yaw_err_rad"] == pytest.approx(2 * math.pi - 6.2, abs=1e-3)  # short way round
    assert shot["est"] == [3.0, 4.0, -3.1] and shot["truth"] == [0.0, 0.0, 3.1]
    assert (shot["alignment"], shot["hits"], shot["speed"]) == (0.9, 120.0, 2.5)
    json.dumps(shot, allow_nan=False)


def test_snapshot_without_truth_keeps_the_estimate_and_rejects_bad_numbers():
    shot = build_snapshot(1.0, (1.0, 2.0, 0.5), None)
    assert shot["truth"] is None and shot["err_m"] is None and shot["yaw_err_rad"] is None
    assert build_snapshot(1.0, (math.nan, 2.0, 0.5), (0, 0, 0)) is None
    assert build_snapshot(1.0, None, (0, 0, 0)) is None
    shot = build_snapshot(1.0, (1.0, 2.0, 0.5), (math.inf, 0, 0), alignment=math.nan)
    assert shot["truth"] is None and shot["alignment"] is None
    json.dumps(shot, allow_nan=False)


def test_geometry_comes_from_the_map_and_thins_recorded_hits(tmp_path):
    source = tmp_path / "map.json"
    source.write_text(json.dumps({"observed_hits": [[i * 0.01, 0.0] for i in range(9000)]}))
    race_map = SimpleNamespace(left_bound=np.array([[0, 0], [1, 0], [1, 1]]),
                               right_bound=np.array([[0, 2], [1, 2]]), source_path=str(source))
    geometry = geometry_from_map(race_map, np.array([[0.5, 0.5], [0.6, 0.6]]))
    assert geometry["left"] == [[0, 0], [1, 0], [1, 1]] and len(geometry["path"]) == 2
    assert 0 < len(geometry["hits"]) <= 3000
    race_map.source_path = str(tmp_path / "missing.json")
    assert geometry_from_map(race_map)["hits"] == []  # an unreadable map file only drops the hits


def test_hub_wakes_waiters_with_the_newest_snapshot_and_signals_resets():
    hub = PoseHub()
    assert hub.wait(0, 0, 0.01) == (0, 0, None)  # nothing yet: a timeout, not a hang
    hub.publish({"n": 1})
    hub.publish({"n": 2})
    seq, epoch, shot = hub.wait(0, 0, 1.0)
    assert (seq, epoch, shot) == (2, 0, {"n": 2})
    hub.reset()
    assert hub.wait(seq, 0, 1.0) == (2, 1, None)  # a reset arrives without a stale snapshot
    hub.close()
    assert hub.wait(2, 1, 5.0)[2] is None  # closing releases waiters at once


@pytest.fixture
def server():
    hub = PoseHub({"left": [[0, 0]], "right": [], "path": [], "hits": []}, error_alert_m=0.4)
    view = PoseViewServer(hub, PoseViewConfig(port=0, host="127.0.0.1"))
    view.start()
    yield view, hub
    view.close()


def _get(view, path):
    connection = http.client.HTTPConnection("127.0.0.1", view.port, timeout=5)
    connection.request("GET", path)
    return connection, connection.getresponse()


def test_server_serves_the_page_the_track_and_404(server):
    view, _ = server
    connection, response = _get(view, "/")
    body = response.read().decode()
    assert response.status == 200 and "Where the car thinks it is" in body
    connection.close()
    connection, response = _get(view, "/geometry")
    assert json.loads(response.read()) == {"left": [[0, 0]], "right": [], "path": [], "hits": [],
                                           "error_alert_m": 0.4}
    connection.close()
    connection, response = _get(view, "/nothing")
    assert response.status == 404
    connection.close()


def test_event_stream_delivers_snapshots_and_resets(server):
    view, hub = server
    connection, response = _get(view, "/events")
    assert response.status == 200
    assert response.getheader("Content-Type") == "text/event-stream"
    hub.publish({"t": 1.0, "est": [1.0, 2.0, 0.0]})
    assert response.readline() == b'data: {"t": 1.0, "est": [1.0, 2.0, 0.0]}\n'
    response.readline()  # blank line ending the event
    hub.reset()
    assert response.readline() == b"event: reset\n"
    connection.close()


def test_task_publishes_estimate_and_truth_and_survives_failures():
    pytest.importorskip("avlite")
    from avlite_autodrive.plugin.pose_view import PoseComparisonView

    ego = SimpleNamespace(x=1.0, y=2.0, theta=0.1, velocity=3.0)
    truth = {"pose": (1.0, 2.5, 0.0)}
    world = SimpleNamespace(true_pose=lambda: truth["pose"])
    executer = SimpleNamespace(ego_state=ego, world=world)
    task = PoseComparisonView(PoseViewConfig(port=0, host="127.0.0.1"),
                              localizer=SimpleNamespace(alignment=0.8, hits=50))
    try:
        task.execute(executer)
        _, _, shot = task.hub.wait(0, 0, 1.0)
        assert shot["err_m"] == pytest.approx(0.5) and shot["alignment"] == 0.8
        assert shot["speed"] == 3.0
        truth["pose"] = None  # no ground truth: the estimate is still published
        task.execute(executer)
        assert task.hub.wait(1, 0, 1.0)[2]["truth"] is None
        task.reset()
        assert task.hub.epoch == 1
        task.execute(SimpleNamespace())  # a broken executer disables the view, never raises
        task.execute(executer)  # and it stays disabled
        assert task.hub.wait(2, 1, 0.05)[2] is None
    finally:
        task.close()
