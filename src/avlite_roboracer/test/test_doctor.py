"""The profile doctor lists what is still unmeasured and passes a commissioned profile."""

import yaml

from avlite_roboracer.doctor import HOW, inspect, main, render
from avlite_roboracer.profile import AUTONOMY_REQUIRED
from roboracer_fixtures import commissioned_data


def write(tmp_path, data):
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def test_every_required_measurement_has_a_how_to():
    assert set(AUTONOMY_REQUIRED) <= set(HOW)


def test_committed_profile_is_not_commissioned_and_every_gap_is_explained():
    report = inspect(__import__("avlite_roboracer.doctor", fromlist=["x"]).DEFAULT_PROFILE)
    assert report["loaded"] and not report["commissioned"]
    assert all(item["how"] for item in report["missing"])
    assert "NOT commissioned" in render(report)


def test_commissioned_profile_passes_and_exit_code_reflects_it(tmp_path, capsys):
    path = write(tmp_path, commissioned_data())
    assert main(["--profile", str(path)]) == 0
    assert "Commissioned" in capsys.readouterr().out
    data = commissioned_data()
    data["vehicle"]["wheelbase_m"] = None
    assert main(["--profile", str(write(tmp_path, data))]) == 1
    assert "vehicle.wheelbase_m" in capsys.readouterr().out


def test_unreadable_profile_is_reported_not_raised(tmp_path):
    report = inspect(tmp_path / "missing.yaml")
    assert not report["loaded"] and report["error"]
    assert "CANNOT LOAD" in render(report)
