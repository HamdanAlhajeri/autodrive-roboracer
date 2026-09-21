import pytest

from avlite_autodrive.response_test import ResponseExperiment


def test_one_bounded_attempt_per_circuit_and_completion():
    experiment = ResponseExperiment(1.5, 3)
    for trial in range(1, 4):
        now = 10 * trial
        assert not experiment.step(now, 5, 30, 1.5, True)
        assert experiment.step(now + 0.21, 5.3, 30, 1.5, True)
        assert experiment.trial == trial
        assert experiment.step(now + 0.4, 5.5, 30, 0.8, True)
        assert not experiment.step(now + 1.1, 6, 30, 0.8, True)
        assert not experiment.step(now + 2, 7, 30, 1.5, True)
        experiment.step(now + 3, 29, 30, 1.5, False)
        experiment.step(now + 4, 0, 30, 1.5, False)
    assert experiment.phase == experiment.FINISHED
    assert not experiment.step(50, 5, 30, 1.5, True)


@pytest.mark.parametrize("speed,eligible,safe", [
    (.6, True, True), (1.5, False, True), (1.5, True, False),
])
def test_coast_ends_on_low_speed_turn_obstacle_or_invalid_pose(speed, eligible, safe):
    experiment = ResponseExperiment(1.5, 3)
    experiment.step(1, 5, 30, 1.5, True)
    assert experiment.step(1.21, 5.3, 30, 1.5, True)
    assert not experiment.step(1.3, 5.4, 30, speed, eligible, safe)
    assert experiment.trial == 1


def test_interruption_aborts_until_new_controller():
    experiment = ResponseExperiment(1.5, 3)
    experiment.reset()  # initial sensor discovery is allowed
    experiment.step(0, 10, 30, 0, False)
    experiment.reset()
    assert not experiment.step(1, 10, 30, 1.5, True)
    assert experiment.diagnostics()["response_aborted"]


@pytest.mark.parametrize("speed,trials", [
    (0, 3), (3, 3), (float("nan"), 3), (1.5, 1), (1.5, 3.2), (1.5, True),
])
def test_reject_invalid_experiment(speed, trials):
    with pytest.raises(ValueError):
        ResponseExperiment(speed, trials)
