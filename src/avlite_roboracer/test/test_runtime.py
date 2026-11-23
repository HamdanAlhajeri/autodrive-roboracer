from collections import deque
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from avlite_roboracer.command import CommandArbiter, CommandSource, SpeedDemand
from avlite_roboracer import runtime as runtime_module
from avlite_roboracer.profiling import StackSampler, TickStats
from avlite_roboracer.runtime import Runtime, sensor_stamp_is_fresh
from avlite_roboracer.supervisor import State, Supervisor
from roboracer_fixtures import commissioned_profile


@pytest.mark.parametrize("stamp,previous,now,expected", [
    (100, None, 110, True), (100, 100, 110, False), (100, 101, 110, False),
    (100, None, 1000000100, False), (0, None, 0, False),
    # The Hokuyo stamps scans about 1 ms ahead of the Jetson clock: that must be accepted.
    (10_001_000_000, None, 10_000_000_000, True),
    (10_001_000_000, 10_000_000_000, 10_000_000_000, True),
    # A stamp well ahead of the clock (a wrong clock) or too old is still refused.
    (10_500_000_000, None, 10_000_000_000, False),
    (9_000_000_000, None, 10_000_000_000, False),
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
    runtime.tick_stats = TickStats()
    runtime.sampler = StackSampler(threading.get_ident())
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


def test_warm_up_imports_what_is_installed_and_names_what_is_not(monkeypatch):
    # The first control tick must not pay for importing AVLite: that stalled the scan callbacks
    # for about 0.6 s on the Jetson and tripped the stale-LiDAR stop when mapping started.
    monkeypatch.setattr(runtime_module, "WARM_UP_MODULES", ("json", "no_such_module_for_warm_up"))
    messages = []
    skipped = runtime_module.warm_up_imports(messages.append)
    assert skipped == ["no_such_module_for_warm_up"]
    assert len(messages) == 1 and "warm-up imports took" in messages[0]
    assert "no_such_module_for_warm_up" in messages[0]
    assert runtime_module.warm_up_imports() == ["no_such_module_for_warm_up"]


def test_scan_stats_text_reports_what_the_stale_lidar_stop_needs():
    runtime = Runtime.__new__(Runtime)
    assert runtime.scan_stats_text() == "no scans seen since the session started"
    runtime.reset_scan_stats()
    assert runtime.scan_stats_text() == "no scans seen since the session started"
    runtime.scan_stats.update(seen=10, rejected=3, lag_last_s=0.21, lag_max_s=0.34)
    text = runtime.scan_stats_text()
    assert "seen 10" in text and "rejected as stale or out of order 3" in text
    assert "last 0.210 s" in text and "max 0.340 s" in text


def mapping_stub(session, mapping_session):
    runtime = runtime_stub()
    runtime.check_faults = Runtime.check_faults.__get__(runtime)
    runtime.supervisor.state = State.MAPPING
    runtime.supervisor.number = session
    runtime.ages = lambda now: (0.0, 0.0)
    runtime.actuator = (time.monotonic(), {"fault": None, "manual_override": False,
                                           "deadman_held": True})
    runtime.slam = SimpleNamespace(died=False, mode="mapping")
    runtime.pose = lambda *args, **kwargs: None  # no SLAM pose yet
    runtime.lap_started = True   # left over from the previous session
    runtime.mapping_session = mapping_session
    return runtime


def test_a_new_mapping_session_is_not_judged_by_the_previous_sessions_lap_state():
    # Second map-start on the same supervisor: tick() runs check_faults before the start action
    # has reset lap_started, so the old True must not count for the new session.
    runtime = mapping_stub(session=3, mapping_session="boot-1")
    assert not runtime.lap_active
    runtime.check_faults(time.monotonic())
    assert runtime.supervisor.state == State.MAPPING


def test_the_mapping_pose_check_still_stops_a_session_whose_lap_has_started():
    runtime = mapping_stub(session=3, mapping_session="boot-3")
    assert runtime.lap_active
    runtime.check_faults(time.monotonic())
    assert runtime.supervisor.state == State.STOPPED
