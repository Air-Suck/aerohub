import numpy as np
import pytest

from uav_pin_slam_ros2.contracts import PointCloudContract, xyz_fields_valid
from uav_pin_slam_ros2.motion import body_twist, quaternion_xyzw


def test_cloud_contract_rejects_frame_empty_and_non_monotonic_input():
    contract = PointCloudContract("drone0/Lidar")
    assert contract.check("wrong", 1, 100).reason == "unexpected_frame"
    assert contract.check("drone0/Lidar", 1, 9).reason == "insufficient_points"
    assert contract.check("drone0/Lidar", 10, 100).accepted
    assert contract.check("drone0/Lidar", 10, 100).reason == "non_monotonic_stamp"
    assert contract.check("drone0/Lidar", 9, 100).reason == "non_monotonic_stamp"


def test_body_twist_is_expressed_in_current_sensor_frame():
    previous = np.eye(4)
    current = np.eye(4)
    current[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    current[:3, 3] = [1.0, 0.0, 0.0]
    twist = body_twist(previous, current, 2.0)
    np.testing.assert_allclose(twist.linear, [0.0, -0.5, 0.0], atol=1.0e-8)
    np.testing.assert_allclose(twist.angular, [0.0, 0.0, np.pi / 4.0], atol=1.0e-8)


def test_motion_rejects_zero_dt_and_quaternion_is_normalized():
    with pytest.raises(ValueError):
        body_twist(np.eye(4), np.eye(4), 0.0)
    q = quaternion_xyzw(np.eye(3))
    np.testing.assert_allclose(q, [0.0, 0.0, 0.0, 1.0])


class Field:
    def __init__(self, name, datatype=7, count=1):
        self.name = name
        self.datatype = datatype
        self.count = count


def test_xyz_contract_requires_exact_float32_scalars():
    assert xyz_fields_valid([Field("x"), Field("y"), Field("z")])
    assert not xyz_fields_valid([Field("x"), Field("y")])
    assert not xyz_fields_valid([Field("x"), Field("x"), Field("y"), Field("z")])
    assert not xyz_fields_valid([Field("x", datatype=8), Field("y"), Field("z")])
