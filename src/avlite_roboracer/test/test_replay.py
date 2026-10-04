from types import SimpleNamespace

import pytest

from avlite_roboracer import recorder
from avlite_roboracer.slam import require_save_success
from roboracer_fixtures import ROOT, commissioned_profile


def test_replay_excludes_recorded_commands_and_slam_outputs():
    profile = commissioned_profile(**{"topics.imu": "/imu"})
    assert set(recorder.replay_topics(profile)) == {"/scan", "/odom", "/imu", "/tf", "/tf_static"}
    assert "/roboracer/command" in recorder.record_topics(profile)


def test_replay_refuses_live_domain(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "73")
    with pytest.raises(ValueError, match="differ"):
        recorder.replay_environment(73)
    assert recorder.replay_environment(74)["ROS_LOCALHOST_ONLY"] == "1"


def test_failed_slam_save_cannot_be_treated_as_success():
    for code in (1, 255):
        with pytest.raises(RuntimeError, match="failed with result"):
            require_save_success(SimpleNamespace(result=code), "save_map")
    assert require_save_success(SimpleNamespace(result=0), "save_map").result == 0


def test_replay_isolates_all_processes_and_cleans_up_failed_playback(tmp_path, monkeypatch):
    calls, stopped, slam_calls = [], [], []

    class Process:
        def __init__(self, command, **kwargs):
            self.command = command
            calls.append((command, kwargs))

        def wait(self):
            return 1  # playback fails; all launched processes still need cleanup

    class Slam:
        def __init__(self, directory):
            pass

        def start(self, *args, **kwargs):
            slam_calls.append(kwargs)

        def stop(self):
            stopped.append("slam")

    monkeypatch.setenv("ROS_DOMAIN_ID", "0")
    monkeypatch.setattr(recorder.subprocess, "Popen", Process)
    monkeypatch.setattr(recorder, "SlamProcess", Slam)
    monkeypatch.setattr(recorder, "git_revision", lambda: "test")
    monkeypatch.setattr(recorder, "interrupt", lambda p: stopped.append(p.command))
    monkeypatch.setattr(recorder.time, "sleep", lambda _: None)
    args = SimpleNamespace(session=str(tmp_path), mode="mapping", domain_id=73, rate=1.0,
                           slam_params=ROOT / "config/roboracer/slam_toolbox.yaml",
                           pose_graph=None, start_pose=[0, 0, 0], profile="profile.yaml")
    with pytest.raises(RuntimeError, match="bag play failed"):
        recorder.replay(args, commissioned_profile(**{"topics.imu": "/imu"}))
    for _, kwargs in calls:
        assert kwargs["env"]["ROS_DOMAIN_ID"] == "73"
        assert kwargs["env"]["ROS_LOCALHOST_ONLY"] == "1"
        assert kwargs["start_new_session"]
    assert slam_calls[0]["env"]["ROS_DOMAIN_ID"] == "73"
    play = next(cmd for cmd, _ in calls if cmd[:3] == ["ros2", "bag", "play"])
    assert set(play[play.index("--topics") + 1:play.index("--remap")]) == {
        "/scan", "/odom", "/imu", "/tf", "/tf_static"}
    assert "/tf_static:=/replay/tf_static" in play
    assert len(stopped) == 4  # playback, output recording, SLAM and TF relay
