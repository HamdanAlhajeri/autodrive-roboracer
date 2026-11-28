"""tools/car-run.sh against a fake supervisor CLI: command order, refusals and exit codes."""

import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "tools" / "car-run.sh"
pytestmark = pytest.mark.skipif(shutil.which("bash") is None or shutil.which("python3") is None
                                or not SCRIPT.exists(), reason="needs bash, python3 and the script")

FAKE_CLI = """#!/usr/bin/env bash
d="$FAKE_DIR"; echo "$*" >> "$d/calls"
state="$(cat "$d/state")"
case "$1" in
  status)
    if [[ "${2:-}" == "--json" ]]; then
      fmt='{"state":"%s","session":"s1","reasons":%s,"actuator":{"deadman_held":%s},'
      fmt+='"session_dir":"/tmp/session","finalize_step":null}\\n'
      printf "$fmt" \\
        "$state" "$(cat "$d/reasons")" "$(cat "$d/deadman")"
    else echo "State:   $state   session s1"; fi ;;
  map-start)
    [[ "$state" == IDLE || "$state" == STOPPED ]] || {
      echo "Rejected: map-start not allowed in $state" >&2; exit 1; }
    echo MAPPING > "$d/state"; echo "mapping started; session s1. Ctrl+C to stop."
    sleep 0.5; cat "$d/map_result" > "$d/state"
    echo "Authorization ended: state FINALIZING" >&2; exit 0 ;;
  race-start)
    [[ "$state" == READY ]] || { echo "Rejected: not ready" >&2; exit 1; }
    echo RACING > "$d/state"
    trap 'echo STOPPED > "$d/state"; echo Stopped >&2; exit 130' INT TERM
    if [[ -f "$d/race_fault" ]]; then sleep 0.3; echo STOPPED > "$d/state"; exit 0; fi
    while true; do sleep 0.1; done ;;
  stop) echo STOPPED > "$d/state"; echo "stopped (STOPPED)" ;;
esac
"""
FAKE_CAR_DAY_OK = "#!/usr/bin/env bash\necho all good\nexit 0\n"
FAKE_CAR_DAY_BAD = ("#!/usr/bin/env bash\necho '  [FAIL] hardware profile not commissioned'\n"
                    "exit 1\n")


@pytest.fixture
def car(tmp_path):
    fake = tmp_path / "fake"
    fake.mkdir()
    cli = tmp_path / "cli.sh"
    cli.write_text(FAKE_CLI)
    cli.chmod(cli.stat().st_mode | stat.S_IEXEC)
    car_day = tmp_path / "car-day.sh"
    car_day.write_text(FAKE_CAR_DAY_OK)
    for name, text in (("state", "IDLE"), ("reasons", "[]"), ("deadman", "true"),
                       ("map_result", "READY")):
        (fake / name).write_text(text)
    race_yaml = tmp_path / "race.yaml"
    race_yaml.write_text(
        "planning:\n  max_velocity_mps: 1.5\nmapping:\n  laps: 3\nracing:\n  laps: 9\n")

    class Car:
        dir, cli_path, car_day_path, yaml = fake, cli, car_day, race_yaml

        def set(self, name, text):
            (fake / name).write_text(text)

        def calls(self):
            path = fake / "calls"
            return path.read_text().splitlines() if path.exists() else []

        def motion_calls(self):
            return [c for c in self.calls() if c.split()[0] in ("map-start", "race-start")]

        def run(self, *args, tty=False):
            env = dict(os.environ, FAKE_DIR=str(fake), ROBORACER_CLI=str(cli), CAR_DAY=str(car_day),
                       ROBORACER_RACE_YAML=str(race_yaml), CAR_RUN_LOG=str(tmp_path / "log"))
            return subprocess.run(["bash", str(SCRIPT), *args], capture_output=True, text=True,
                                  env=env, timeout=60, stdin=None if tty else subprocess.DEVNULL)

    return Car()


def test_happy_path_maps_then_races_for_the_set_time_then_stops(car):
    done = car.run("--yes", "--race-seconds", "2")
    assert done.returncode == 0, done.stdout + done.stderr
    kinds = [c.split()[0] for c in car.calls()]
    assert kinds.index("map-start") < kinds.index("race-start")
    assert kinds[-1] in ("stop", "status")
    assert (car.dir / "state").read_text().strip() == "STOPPED"  # the race ended with a stop
    out = done.stdout
    for text in ("Mapping", "READY", "Racing for 2 s", "race time reached"):
        assert text in out


def test_without_yes_and_without_a_terminal_nothing_moves(car):
    done = car.run("--race-seconds", "2")
    assert done.returncode == 2
    assert "needs a person's confirmation" in done.stdout
    assert car.motion_calls() == []


def test_a_failed_preflight_blocks_everything(car):
    car.car_day_path.write_text(FAKE_CAR_DAY_BAD)
    done = car.run("--yes")
    assert done.returncode == 2 and "preflight failed" in done.stdout and "[FAIL]" in done.stdout
    assert car.calls() == []  # the supervisor was never even asked


def test_no_supervisor_prints_the_commands_to_start_it(car):
    car.cli_path.write_text("#!/usr/bin/env bash\necho 'cannot connect' >&2\nexit 2\n")
    done = car.run("--yes")
    assert done.returncode == 2
    assert "ros2 launch f1tenth_stack bringup_launch.py" in done.stdout
    assert "tools/jetson-run.sh launch" in done.stdout and "--start-stack" in done.stdout


def test_a_map_that_does_not_validate_stops_before_racing_and_says_why(car):
    car.set("map_result", "STOPPED")
    car.set("reasons", '["lap closure check failed"]')
    done = car.run("--yes")
    assert done.returncode == 1
    assert "mapping did not reach READY" in done.stdout
    assert "race-start" not in " ".join(car.calls())


def test_map_only_does_not_race_and_race_only_needs_a_ready_map(car):
    done = car.run("--yes", "--map-only")
    assert done.returncode == 0 and "map is READY" in done.stdout
    assert [c.split()[0] for c in car.motion_calls()] == ["map-start"]
    car.set("state", "IDLE")
    refused = car.run("--yes", "--race-only")
    assert refused.returncode == 1 and "not READY" in refused.stdout
    assert [c.split()[0] for c in car.motion_calls()] == ["map-start"]  # nothing new was sent


def test_an_already_ready_map_is_reused_unless_remap_is_given(car):
    car.set("state", "READY")
    done = car.run("--yes", "--race-seconds", "1")
    assert done.returncode == 0 and "skipping mapping" in done.stdout
    assert [c.split()[0] for c in car.motion_calls()] == ["race-start"]
    car.set("state", "READY")
    open(car.dir / "calls", "w").close()
    remap = car.run("--yes", "--remap", "--race-seconds", "1")
    assert remap.returncode == 0
    assert [c.split()[0] for c in car.motion_calls()] == ["map-start", "race-start"]


def test_the_car_waits_for_the_dead_man_button_and_gives_up_if_it_is_never_held(car, monkeypatch):
    monkeypatch.setenv("CAR_RUN_DEADMAN_WAIT", "2")
    car.set("deadman", "false")
    done = car.run("--yes", "--race-seconds", "1")
    assert done.returncode == 1
    assert "Hold the joystick dead-man button" in done.stdout
    assert "was not held within 2 s" in done.stdout
    assert car.motion_calls() == []  # never started mapping without the button


def test_laps_option_edits_only_the_mapping_laps(car):
    car.run("--yes", "--map-only", "--laps", "5")
    text = car.yaml.read_text()
    assert "mapping:\n  laps: 5\n" in text and "racing:\n  laps: 9\n" in text
    bad = car.run("--yes", "--map-only", "--laps", "0")
    assert bad.returncode == 2


def test_a_fault_during_the_race_is_reported_as_a_failure(car):
    car.set("state", "READY")
    (car.dir / "race_fault").write_text("1")
    done = car.run("--yes", "--race-seconds", "5")
    assert done.returncode == 1 and "stopped the race before the time was up" in done.stdout


def test_bad_options_are_refused_before_anything_is_sent(car):
    for args in (["--map-only", "--race-only"], ["--race-seconds", "abc"], ["--unknown"]):
        assert car.run("--yes", *args).returncode == 2
    assert car.calls() == []
