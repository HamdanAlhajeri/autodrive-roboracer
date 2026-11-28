"""The throttle ramp (config/f1tenth/throttle_interpolator.py): slow to speed up, quick to stop."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType

# The node file imports ROS; only its ramp() function is tested here, so stand-ins are enough.
for name, attrs in (("rclpy", {}), ("rclpy.node", {"Node": object}),
                    ("std_msgs", {}), ("std_msgs.msg", {"Float64": object})):
    if name not in sys.modules:
        stand_in = ModuleType(name)
        stand_in.__dict__.update(attrs)
        sys.modules[name] = stand_in

PATH = Path(__file__).resolve().parents[3] / "config" / "f1tenth" / "throttle_interpolator.py"
spec = importlib.util.spec_from_file_location("throttle_interpolator", PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ramp = module.ramp


def run(last, desired, up, down, steps):
    for _ in range(steps):
        last = ramp(last, desired, up, down)
    return last


def test_full_stick_is_reached_gradually_at_the_acceleration_limit():
    values = []
    last = 0.0
    for _ in range(5):
        last = ramp(last, 3000.0, 100.0, 500.0)
        values.append(last)
    assert values == [100.0, 200.0, 300.0, 400.0, 500.0]
    assert run(0.0, 3000.0, 100.0, 500.0, 100) == 3000.0


def test_stopping_uses_the_faster_deceleration_limit():
    assert ramp(3000.0, 0.0, 100.0, 500.0) == 2500.0
    assert run(3000.0, 0.0, 100.0, 500.0, 6) == 0.0


def test_lowering_the_speed_is_a_deceleration_not_an_acceleration():
    assert ramp(3000.0, 2000.0, 100.0, 500.0) == 2500.0


def test_reverse_demand_slows_through_zero_first_in_both_directions():
    assert ramp(300.0, -3000.0, 100.0, 500.0) == 0.0
    assert ramp(0.0, -3000.0, 100.0, 500.0) == -100.0
    assert ramp(-3000.0, 1000.0, 100.0, 500.0) == -2500.0


def test_equal_limits_behave_like_the_stock_single_limit():
    assert ramp(0.0, 1000.0, 200.0, 200.0) == 200.0
    assert ramp(1000.0, 0.0, 200.0, 200.0) == 800.0
    assert ramp(5.0, 5.0, 200.0, 200.0) == 5.0
