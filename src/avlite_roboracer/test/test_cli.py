import queue
import socket
import sys
import threading

import pytest

from avlite_roboracer.cli import authorize
from avlite_roboracer.protocol import dispatch
from avlite_roboracer.supervisor import State, Supervisor


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class LocalClient:
    """Calls the supervisor directly with the fake clock, as the socket server would."""

    def __init__(self, supervisor, clock):
        self.supervisor = supervisor
        self.clock = clock
        self.sent = []

    def request(self, command, **fields):
        self.sent.append(command)
        reply = dispatch(self.supervisor, {"command": command, **fields}, self.clock())
        self.supervisor.tick(self.clock())
        return reply


@pytest.fixture
def setup():
    clock = FakeClock()
    supervisor = Supervisor("b", 1.0, lambda now: [], lambda now: [])
    supervisor.map_start(supervisor.session, 0.0)
    return clock, supervisor, LocalClient(supervisor, clock)


def test_authorization_ends_when_supervisor_stops_session(setup):
    clock, supervisor, client = setup
    original = client.request

    def request(command, **fields):
        if clock() >= 2.0 and supervisor.state == State.MAPPING:
            supervisor.stop(clock())
        return original(command, **fields)

    client.request = request
    assert authorize(client, supervisor.session, None, clock=clock, sleep=clock.sleep) == 0
    assert supervisor.state == State.STOPPED


def test_silent_laptop_stops_the_car(setup):
    clock, supervisor, client = setup
    lines = queue.Queue()
    lines.put("hb\n")
    code = authorize(client, supervisor.session, lines, laptop_timeout_s=0.5,
                     clock=clock, sleep=clock.sleep)
    assert code == 3
    assert client.sent[-1] == "stop"
    assert supervisor.state == State.STOPPED
    assert clock() <= 0.8  # stopped within the laptop timeout, before supervisor expiry


def test_closed_laptop_stream_stops_the_car(setup):
    clock, supervisor, client = setup
    lines = queue.Queue()
    lines.put(None)
    assert authorize(client, supervisor.session, lines, clock=clock, sleep=clock.sleep) == 3
    assert supervisor.state == State.STOPPED


def test_keyboard_interrupt_sends_stop(setup):
    clock, supervisor, client = setup

    def interrupt(_):
        raise KeyboardInterrupt

    assert authorize(client, supervisor.session, None, clock=clock, sleep=interrupt) == 130
    assert supervisor.state == State.STOPPED


@pytest.mark.skipif(not hasattr(socket, "AF_UNIX") or sys.platform == "win32",
                    reason="Unix sockets are used on the Jetson")
def test_socket_roundtrip(tmp_path):
    from avlite_roboracer.protocol import Client, CommandServer
    supervisor = Supervisor("b", 1.0, lambda now: [], lambda now: [])
    lock = threading.Lock()

    def handler(request):
        with lock:
            return dispatch(supervisor, request, 0.0)

    server = CommandServer(tmp_path / "s" / "sock", handler)
    server.start()
    try:
        with pytest.raises(RuntimeError, match="already owns"):
            CommandServer(tmp_path / "s" / "sock", handler)
        client = Client(tmp_path / "s" / "sock")
        assert client.request("status")["state"] == "IDLE"
        assert client.request("map-start", session="b-0")["ok"]
        assert client.request("heartbeat", session="b-1")["ok"]
        client.close()
        assert (tmp_path / "s").stat().st_mode & 0o077 == 0
    finally:
        server.close()
