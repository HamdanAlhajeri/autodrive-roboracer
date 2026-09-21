import pytest

from avlite_autodrive.telemetry_health import ActuatorPublicationHealth


def test_stationary_startup_grace_requires_both_commands_before_passing():
    health = ActuatorPublicationHealth()
    assert health.check(0, {}, {}) is None
    assert not health.valid
    state = {"throttle_command": 0.0, "steering_command": 0.0}
    assert health.check(1, state, dict.fromkeys(state, 0.9)) is None
    assert health.valid


@pytest.mark.parametrize("moving,expired", [(True, False), (False, True)])
def test_missing_publication_cannot_be_credited_as_clean(moving, expired):
    health = ActuatorPublicationHealth()
    assert health.check(1, {}, {}, moving=moving, startup_expired=expired)
    assert not health.valid


@pytest.mark.parametrize("value", [None, float("nan"), float("inf"), 1.1, -1.1])
def test_invalid_command_is_latched(value):
    health = ActuatorPublicationHealth()
    state = {"throttle_command": value, "steering_command": 0.0}
    received = dict.fromkeys(state, 1)
    assert "Invalid" in health.check(1, state, received)
    state["throttle_command"] = 0
    health.check(1.1, state, received)
    assert not health.valid


def test_frozen_zero_commands_fail_even_with_fresh_odometry_and_counters():
    health = ActuatorPublicationHealth()
    state = {"throttle_command": 0.0, "steering_command": 0.0}
    received = dict.fromkeys(state, 1)
    assert health.check(1.5, state, received) is None
    assert "expired" in health.check(1.501, state, received)
    health.check(2, state, dict.fromkeys(state, 2))
    assert not health.valid
