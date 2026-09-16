import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from avlite_autodrive.sensors import scan_cloud
from avlite_autodrive.ros_utils import odom_state, valid_scan


def scan(ranges, angle_min=-math.pi / 2, increment=math.pi / 2):
    return NS(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=increment,
        range_min=0.06,
        range_max=10.0,
    )


def test_cloud_axes_and_unknown_returns():
    cloud = scan_cloud(scan([2, 2, 2]))
    np.testing.assert_allclose(cloud[:, :2], [[0, -2], [2, 0], [0, 2]], atol=1e-6)
    assert cloud.shape == (3, 4)
    assert cloud.dtype == np.float32
    cloud = scan_cloud(scan([float("inf"), float("nan"), -1]))
    np.testing.assert_allclose(np.linalg.norm(cloud[:, :2], axis=1), [10, 0.06, 0.06])


def test_scan_validity_requires_observations():
    assert valid_scan(scan([2] * 1080, increment=0.00436))
    assert not valid_scan(scan([float("nan")] * 1080))
    assert not valid_scan(scan([float("inf")] * 1080))
    assert not valid_scan(scan([2] * 1080, increment=0))
    assert not valid_scan(scan([]))


def test_odometry_does_not_rotate_body_velocity_twice():
    msg = NS(
        pose=NS(
            pose=NS(
                position=NS(x=1, y=2), orientation=NS(x=0, y=0, z=math.sqrt(0.5), w=math.sqrt(0.5))
            )
        ),
        twist=NS(twist=NS(linear=NS(x=0.5, y=0))),
    )
    assert odom_state(msg) == pytest.approx((1, 2, math.pi / 2, 0.5))
    msg.pose.pose.orientation.w = 0
    with pytest.raises(ValueError):
        odom_state(msg)
