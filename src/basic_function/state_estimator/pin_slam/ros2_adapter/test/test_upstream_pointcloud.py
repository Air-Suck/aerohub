import numpy as np
from sensor_msgs_py import point_cloud2 as ros_point_cloud2
from std_msgs.msg import Header
from utils import point_cloud2 as upstream_point_cloud2


def test_ros2_xyz_cloud_is_consumed_by_upstream_reader():
    expected = np.array(
        [[1.0, 2.0, 3.0], [-4.0, 5.5, 6.25], [7.0, -8.0, 9.0]],
        dtype=np.float32,
    )
    message = ros_point_cloud2.create_cloud_xyz32(
        Header(frame_id="drone0/Lidar"), expected)
    points, point_time = upstream_point_cloud2.read_point_cloud(message)
    np.testing.assert_allclose(points, expected)
    assert point_time is None
