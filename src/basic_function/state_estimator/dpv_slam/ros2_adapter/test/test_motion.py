import numpy as np

from uav_dpv_slam_ros2.motion import quaternion_xyzw


def test_quaternion_identity():
    np.testing.assert_allclose(
        quaternion_xyzw(np.eye(3)), [0.0, 0.0, 0.0, 1.0])
