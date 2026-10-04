from collections import deque
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from avlite_roboracer.command import CommandArbiter, CommandSource, SpeedDemand
from avlite_roboracer.runtime import Runtime, sensor_stamp_is_fresh
from avlite_roboracer.supervisor import State, Supervisor
from roboracer_fixtures import commissioned_profile


@pytest.mark.parametrize("stamp,previous,now,expected", [
    (100, None, 110, True), (100, 100, 110, False), (100, 101, 110, False),
    (100, None, 90, False), (100, None, 1000000100, False), (0, None, 0, False),
])
def test_sensor_source_timestamps_must_advance_and_be_fresh(stamp, previous, now, expected):
    assert sensor_stamp_is_fresh(stamp, previous, now, 0.5) is expected


def runtime_stub():
    runtime = Runtime.__new__(Runtime)
    runtime.lock = threading.RLock()
    runtime.profile = commissioned_profile()
    runtime.node = Mock()
    runtime.last_graph_check = time.monotonic()
    runtime.pending_scans = deque()
    runtime.supervisor = Supervisor("boot", 1.0, lambda now: [], lambda now: [])
    runtime.supervisor.state = State.RACING
    runtime.supervisor.last_heartbeat_s = time.monotonic()
    runtime.arbiter = CommandArbiter(0.25)
    runtime.arbiter.select(CommandSource.RACING)
    runtime.speed = SpeedDemand(1.0)
    runtime.slam = SimpleNamespace(mode="localization")
    runtime.check_faults = Mock()
    runtime._persist = Mock()
    runtime.Drive = lambda: SimpleNamespace(header=SimpleNamespace(), drive=SimpleNamespace())
    runtime.command_pub = Mock()
    return runtime


def test_control_fault_publishes_stop_in_the_same_tick():
    runtime = runtime_stub()
    runtime.arbiter.submit(CommandSource.RACING, 0.1, 1.0, time.monotonic())

    def control(now):
        runtime.supervisor.fault("outside corridor", now)
        return CommandSource.RACING, 0.1, 1.0

    runtime.control = control
    runtime.tick()
    assert runtime.supervisor.state == State.STOPPED
    assert runtime.command_pub.publish.call_args.args[0].drive.speed == 0


def test_scan_is_retried_until_its_async_transform_arrives():
    runtime = runtime_stub()
    scan = object()
    runtime.pending_scans.append((time.monotonic(), scan))
    runtime.on_new_scan = Mock(side_effect=[False, True])
    runtime.control = lambda now: None
    runtime.tick()
    assert len(runtime.pending_scans) == 1
    runtime.tick()
    assert not runtime.pending_scans
    assert runtime.on_new_scan.call_count == 2


def test_finalization_requires_all_readiness_gates():
    runtime = runtime_stub()
    runtime.supervisor.state = State.FINALIZING
    runtime._finalize_steps = lambda cancel: {"plan": "valid"}
    runtime.race_readiness = lambda now: ["actuator unavailable"]
    runtime.slam = Mock()
    runtime.finalize(threading.Event(), runtime.supervisor.session)
    assert runtime.supervisor.state == State.STOPPED
    assert runtime.supervisor.plan is None
    assert "actuator unavailable" in runtime.supervisor.reasons


def test_cancelled_worker_cannot_mark_a_later_session_ready():
    runtime = runtime_stub()
    runtime.supervisor.state = State.FINALIZING
    old_session = runtime.supervisor.session
    runtime.supervisor.stop(time.monotonic())
    runtime.supervisor.map_start(runtime.supervisor.session, time.monotonic())
    runtime._finalize_steps = lambda cancel: {"old": "plan"}
    runtime.finalize(threading.Event(), old_session)
    assert runtime.supervisor.state == State.MAPPING
    assert runtime.supervisor.plan is None
