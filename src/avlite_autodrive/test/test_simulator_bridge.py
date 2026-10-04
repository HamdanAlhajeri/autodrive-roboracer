from types import SimpleNamespace

import pytest

from avlite_autodrive.simulator_bridge import (
    REQUIRED_FIELDS, CommandExpiry, install_command_expiry, install_startup_guard,
)


def test_incomplete_startup_packet_is_answered_stopped_without_fake_sensors():
    emitted, forwarded = [], []
    sio = SimpleNamespace(
        on=lambda event: lambda fn: fn, emit=lambda *a, **kw: emitted.append((a, kw))
    )
    stock = SimpleNamespace(sio=sio, bridge=lambda *a: forwarded.append(a))
    handler = install_startup_guard(stock)
    handler("client", {"V1 Throttle": "0", "V1 LIDAR Scan Rate": "40"})
    assert not forwarded
    assert emitted[0][1]["data"] == {"V1 Throttle": "0", "V1 Steering": "0", "V1 Reset": "False"}
    assert emitted[0][1]["room"] == "client"
    payload = dict.fromkeys(REQUIRED_FIELDS, "sample")
    handler("client", payload)
    assert forwarded == [("client", payload)]
    assert len(emitted) == 1


def test_commands_expire_without_any_ros_callback_or_timer():
    now = [10.0]
    guard = CommandExpiry(clock=lambda: now[0])
    payload = {"V1 Throttle": "0.9", "V1 Steering": "0.8", "V1 Reset": "False"}
    guard.receive("throttle", 0.2)
    assert guard.gate(payload)["V1 Throttle"] == "0.0"
    guard.receive("steering", 0.1)
    assert guard.gate(payload)["V1 Throttle"] == "0.2"
    now[0] += 0.501
    assert guard.gate(payload) == {**payload, "V1 Throttle": "0.0", "V1 Steering": "0.0"}
    guard.receive("throttle", 0.2)  # one surviving publisher is insufficient
    assert guard.gate(payload)["V1 Throttle"] == "0.0"
    assert payload["V1 Throttle"] == "0.9"


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), 1.1, -1.1])
def test_invalid_command_clears_both_axes(bad):
    guard = CommandExpiry()
    guard.receive("throttle", 0.2)
    guard.receive("steering", 0.1)
    assert not guard.receive("throttle", bad)
    assert guard.gate({})["V1 Steering"] == "0.0"


def test_final_emit_gate_covers_decoding_delay_reset_and_reconnect():
    emitted, handlers, now = [], {}, [1.0]

    def register(event):
        def decorate(fn):
            handlers[event] = fn
            return fn
        return decorate

    stock = SimpleNamespace(
        sio=SimpleNamespace(on=register, emit=lambda *a, **kw: emitted.append((a, kw))),
        callback_throttle_command=lambda msg: None, callback_steering_command=lambda msg: None,
        callback_reset_command=lambda msg: None, connect=lambda *args: None,
    )
    install_command_expiry(stock, clock=lambda: now[0])

    def command():
        stock.callback_throttle_command(SimpleNamespace(data=0.2))
        stock.callback_steering_command(SimpleNamespace(data=0.1))

    def emit():
        stock.sio.emit("Bridge", data={"V1 Throttle": "0.2", "V1 Steering": "0.1"})
        return emitted[-1][0][1]

    command()
    assert emit()["V1 Throttle"] == "0.2"
    now[0] += 1  # camera decoding could take time; gate happens after decoding
    assert emit()["V1 Throttle"] == "0.0"
    for invalidate in (lambda: stock.callback_reset_command(SimpleNamespace(data=True)),
                       lambda: handlers["connect"]("client", {}),
                       lambda: handlers["disconnect"]("client")):
        command()
        invalidate()
        assert emit()["V1 Throttle"] == "0.0"
