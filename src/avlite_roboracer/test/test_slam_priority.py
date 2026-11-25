import subprocess

from avlite_roboracer.slam import SlamProcess


def test_slam_starts_at_a_lower_priority_only_when_asked(monkeypatch, tmp_path):
    seen = []

    class Fake:
        def poll(self):
            return None

    monkeypatch.setattr(subprocess, "Popen", lambda cmd, **kw: seen.append(kw) or Fake())
    SlamProcess(tmp_path, nice=5).start("mapping", "params.yaml")
    SlamProcess(tmp_path).start("mapping", "params.yaml")
    assert seen[0]["preexec_fn"] is not None
    assert seen[1]["preexec_fn"] is None
