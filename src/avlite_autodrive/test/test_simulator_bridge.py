from types import SimpleNamespace

from avlite_autodrive.simulator_bridge import REQUIRED_FIELDS, install_startup_guard


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
