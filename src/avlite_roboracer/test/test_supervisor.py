import pytest

from avlite_roboracer.protocol import dispatch
from avlite_roboracer.supervisor import Action, State, Supervisor


class Gates:
    def __init__(self):
        self.race = []
        self.mapping = []


@pytest.fixture
def gates():
    return Gates()


@pytest.fixture
def sup(gates):
    return Supervisor("boot1", 1.0, lambda now: list(gates.race), lambda now: list(gates.mapping))


def to_ready(sup, now=0.0):
    assert sup.map_start(sup.session, now)["ok"]
    sup.lap_completed(now + 10)
    sup.vehicle_stopped(now + 11)
    sup.finalize_succeeded({"plan_sha256": "abc"}, now + 20)
    assert sup.state == State.READY
    sup.take_actions()


def test_full_sequence_requires_explicit_race_start(sup):
    reply = sup.map_start(sup.session, 0.0)
    assert reply["ok"] and reply["session"] == "boot1-1"
    assert sup.take_actions() == [Action.START_MAPPING]
    sup.heartbeat("boot1-1", 0.5)
    sup.lap_completed(1.0)
    assert sup.state == State.MAPPING and sup.take_actions() == [Action.STOP_AFTER_LAP]
    sup.heartbeat("boot1-1", 1.5)
    sup.vehicle_stopped(2.0)
    assert sup.state == State.FINALIZING and sup.take_actions() == [Action.FINALIZE]
    sup.finalize_succeeded({"plan_sha256": "abc"}, 30.0)
    assert sup.state == State.READY
    assert sup.take_actions() == []  # completing the lap never starts racing
    sup.tick(100.0)  # no authorization needed while stationary in READY
    assert sup.state == State.READY
    assert sup.race_start("boot1-1", 100.0)["ok"]
    assert sup.state == State.RACING and sup.take_actions() == [Action.START_RACING]


@pytest.mark.parametrize("stage", ["idle", "mapping", "finalizing"])
def test_premature_race_start_is_rejected_and_never_queued(sup, stage):
    if stage != "idle":
        sup.map_start(sup.session, 0.0)
    if stage == "finalizing":
        sup.lap_completed(0.5)
        sup.vehicle_stopped(0.6)
    reply = sup.race_start(sup.session, 0.7)
    assert not reply["ok"] and reply["reasons"]
    if stage == "finalizing":
        sup.finalize_succeeded({"plan_sha256": "abc"}, 1.0)
        assert sup.state == State.READY  # the earlier request did not start racing
    assert Action.START_RACING not in sup.take_actions()


def test_readiness_failures_are_reported_not_queued(sup, gates):
    to_ready(sup)
    gates.race = ["localization stable for only 0.5s"]
    reply = sup.race_start(sup.session, 30.0)
    assert not reply["ok"] and reply["reasons"] == ["localization stable for only 0.5s"]
    gates.race = []
    sup.tick(31.0)
    assert sup.state == State.READY  # becoming ready later does not start racing


def test_duplicate_race_start_does_not_restart(sup):
    to_ready(sup)
    assert sup.race_start(sup.session, 30.0)["ok"]
    sup.take_actions()
    reply = sup.race_start(sup.session, 30.1)
    assert not reply["ok"] and "already racing" in reply["message"]
    assert sup.take_actions() == []


def test_delayed_commands_cannot_act_on_a_later_session(sup):
    old = sup.session
    to_ready(sup)
    sup.stop(30.0)
    assert sup.map_start(sup.session, 31.0)["ok"]  # new session boot1-2
    assert not sup.race_start(old, 32.0)["ok"]
    assert not sup.map_start(old, 32.0)["ok"]
    assert not sup.heartbeat("boot1-1", 32.0)["ok"]


def test_laptop_authorization_expiry_stops_motion(sup):
    sup.map_start(sup.session, 0.0)
    sup.take_actions()
    for t in (0.4, 0.8, 1.2):
        assert sup.heartbeat(sup.session, t)["ok"]
        sup.tick(t)
    assert sup.state == State.MAPPING
    sup.tick(2.3)
    assert sup.state == State.STOPPED
    assert sup.reasons == ["laptop authorization expired"]
    assert sup.take_actions() == [Action.STOP_MOTION]
    assert not sup.heartbeat(sup.session, 2.4)["ok"]


def test_racing_authorization_expiry(sup):
    to_ready(sup)
    sup.race_start(sup.session, 30.0)
    sup.tick(31.5)
    assert sup.state == State.STOPPED


def test_failed_validation_requires_new_mapping_attempt(sup):
    sup.map_start(sup.session, 0.0)
    sup.lap_completed(1.0)
    sup.vehicle_stopped(2.0)
    sup.finalize_failed(["Unsupported right boundary gap"], 3.0)
    assert sup.state == State.STOPPED and not sup.status(3.0)["map_ready"]
    assert "run map-start" in sup.reasons[-1]
    reply = sup.race_start(sup.session, 4.0)  # cannot override a failed validation
    assert not reply["ok"] and "no validated map" in reply["message"]
    assert sup.map_start(sup.session, 5.0)["ok"]


def test_stop_is_repeatable_and_aborts_finalization(sup):
    sup.map_start(sup.session, 0.0)
    sup.lap_completed(1.0)
    sup.vehicle_stopped(2.0)
    sup.take_actions()
    assert sup.stop(3.0)["ok"]
    assert sup.take_actions() == [Action.STOP_MOTION, Action.ABORT_FINALIZE]
    assert sup.stop(3.1)["ok"] and sup.stop(3.2)["ok"]
    sup.finalize_succeeded({"late": True}, 4.0)  # late result after abort is ignored
    assert sup.state == State.STOPPED and sup.plan is None


def test_restart_after_stop_needs_command_and_readiness(sup, gates):
    to_ready(sup)
    sup.race_start(sup.session, 30.0)
    sup.fault("lost localization", 31.0)
    assert sup.state == State.STOPPED and sup.reasons == ["lost localization"]
    gates.race = ["no localization estimate"]
    assert not sup.race_start(sup.session, 32.0)["ok"]
    gates.race = []
    assert sup.race_start(sup.session, 33.0)["ok"]


def test_supervisor_restart_never_resumes(gates):
    first = Supervisor("boot1", 1.0, lambda now: [], lambda now: [])
    to_ready(first)
    first.race_start(first.session, 30.0)
    snapshot = first.to_dict()
    second = Supervisor("boot2", 1.0, lambda now: [], lambda now: [], previous=snapshot)
    assert second.state == State.IDLE
    assert "no session was resumed" in second.reasons[0]
    assert second.take_actions() == []
    assert not second.race_start(snapshot["session"], 1.0)["ok"]
    assert not second.heartbeat(snapshot["session"], 1.0)["ok"]


def test_mapping_preconditions(sup, gates):
    gates.mapping = ["hardware profile not commissioned: vehicle.width_m"]
    reply = sup.map_start(sup.session, 0.0)
    assert not reply["ok"] and reply["reasons"] == gates.mapping
    assert sup.state == State.IDLE


def test_dispatch_routes_commands(sup):
    status = dispatch(sup, {"command": "status"}, 0.0, lambda: {"localization": {}})
    assert status["ok"] and status["state"] == "IDLE" and "localization" in status
    assert dispatch(sup, {"command": "map-start", "session": sup.session}, 0.0)["ok"]
    assert dispatch(sup, {"command": "stop"}, 0.1)["state"] == "STOPPED"
    assert not dispatch(sup, {"command": "launch"}, 0.2)["ok"]
    assert not dispatch(sup, ["status"], 0.2)["ok"]


@pytest.mark.parametrize("event", ["stop", "fault"])
@pytest.mark.parametrize("stage", ["mapping", "finalizing", "racing"])
def test_stop_supersedes_unexecuted_actions_and_revokes_authorization(sup, event, stage):
    sup.map_start(sup.session, 0.0)
    if stage in ("finalizing", "racing"):
        sup.lap_completed(0.1)
        sup.vehicle_stopped(0.2)
    if stage == "racing":
        sup.finalize_succeeded({"plan": "valid"}, 0.3)
        sup.race_start(sup.session, 0.4)
    old_session = sup.session
    if event == "stop":
        sup.stop(0.5)
    else:
        sup.fault("test fault", 0.5)
    expected = [Action.STOP_MOTION]
    if stage == "finalizing":
        expected.append(Action.ABORT_FINALIZE)
    assert sup.take_actions() == expected
    assert sup.session != old_session
    assert not sup.race_start(old_session, 0.6)["ok"]
    assert not sup.heartbeat(old_session, 0.6)["ok"]
    if stage == "racing":
        assert sup.race_start(sup.session, 0.7)["ok"]
        assert not sup.heartbeat(old_session, 0.8)["ok"]
