import math
import struct
import sys
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField


def stamp(message, sec, nanosec):
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec


def cloud(sec, nanosec, frame_id, points):
    message = PointCloud2()
    stamp(message, sec, nanosec)
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.data = b"".join(struct.pack("<fff", *point) for point in points)
    message.is_dense = True
    return message


class Probe(Node):
    def __init__(self):
        super().__init__("fast_livo2_adapter_integration_probe")
        self.livox = None
        self.output_odom = None
        self.output_cloud = None
        self.output_cloud_count = 0
        self.reliable_imu = None
        self.reliable_image = None
        self.lidar_pub = self.create_publisher(
            PointCloud2, "/drone0/sensor_measurements/Lidar/points", qos_profile_sensor_data
        )
        self.imu_pub = self.create_publisher(
            Imu, "/drone0/sensor_measurements/imu/data", qos_profile_sensor_data
        )
        self.image_pub = self.create_publisher(
            Image,
            "/drone0/sensor_measurements/FrontCamera/rgb/image",
            qos_profile_sensor_data,
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/LIVO2/imu_propagate", qos_profile_sensor_data
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/cloud_registered", qos_profile_sensor_data
        )
        self.create_subscription(
            CustomMsg,
            "/fast_livo2/airsim_lidar",
            lambda message: setattr(self, "livox", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/drone0/fast_livo2/odometry",
            lambda message: setattr(self, "output_odom", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/drone0/perception/fast_livo2/registered_points",
            self.receive_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/fast_livo2/imu",
            lambda message: setattr(self, "reliable_imu", message),
            10,
        )
        self.create_subscription(
            Image,
            "/fast_livo2/image",
            lambda message: setattr(self, "reliable_image", message),
            10,
        )

    def receive_cloud(self, message):
        self.output_cloud = message
        self.output_cloud_count += 1


def wait_for(predicate, node, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError("timed out waiting for adapter output")


def close(actual, expected, tolerance=1e-5):
    if not math.isclose(actual, expected, abs_tol=tolerance):
        raise AssertionError(f"expected {expected}, got {actual}")


def main():
    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if (
                node.lidar_pub.get_subscription_count() > 0
                and node.imu_pub.get_subscription_count() > 0
                and node.image_pub.get_subscription_count() > 0
                and node.odom_pub.get_subscription_count() > 0
                and node.cloud_pub.get_subscription_count() > 0
            ):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            raise AssertionError("adapter subscriptions were not discovered")

        source_points = [(1.25, -2.5, 3.75), (-4.0, 5.5, -6.25)]
        lidar = cloud(17, 123456789, "drone0/Lidar", source_points)
        for _ in range(3):
            node.lidar_pub.publish(lidar)
            rclpy.spin_once(node, timeout_sec=0.1)
        wait_for(lambda: node.livox is not None, node, 5.0)
        assert node.livox.header.stamp == lidar.header.stamp
        assert node.livox.header.frame_id == "fast_livo_lidar"
        assert node.livox.timebase == 17123456789
        assert node.livox.point_num == 2
        assert len(node.livox.points) == 2
        for actual, expected in zip(node.livox.points, source_points):
            assert actual.offset_time == 0
            close(actual.x, expected[0])
            close(actual.y, expected[1])
            close(actual.z, expected[2])

        imu = Imu()
        stamp(imu, 23, 500000000)
        imu.header.frame_id = "drone0/imu_link"
        imu.angular_velocity.x = 0.1
        imu.angular_velocity.y = -0.2
        imu.angular_velocity.z = 0.3
        image = Image()
        stamp(image, 23, 500000000)
        image.header.frame_id = "drone0/FrontCamera_optical"
        image.height = 1
        image.width = 1
        image.encoding = "rgb8"
        image.step = 3
        image.data = bytes((11, 22, 33))
        odom = Odometry()
        stamp(odom, 23, 510000000)
        odom.header.frame_id = "untrusted_world"
        odom.child_frame_id = "untrusted_body"
        odom.pose.pose.orientation.z = math.sqrt(0.5)
        odom.pose.pose.orientation.w = math.sqrt(0.5)
        odom.twist.twist.linear.x = 2.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = -1.0
        for _ in range(3):
            node.imu_pub.publish(imu)
            node.image_pub.publish(image)
            rclpy.spin_once(node, timeout_sec=0.05)
        wait_for(
            lambda: node.reliable_imu is not None and node.reliable_image is not None,
            node,
            5.0,
        )
        assert node.reliable_imu == imu
        assert node.reliable_image == image
        for _ in range(3):
            node.odom_pub.publish(odom)
            rclpy.spin_once(node, timeout_sec=0.05)
        wait_for(lambda: node.output_odom is not None, node, 5.0)
        assert node.output_odom.header.stamp == odom.header.stamp
        assert node.output_odom.header.frame_id == "camera_init"
        assert node.output_odom.child_frame_id == "fast_livo_imu"
        close(node.output_odom.twist.twist.linear.x, 0.0)
        close(node.output_odom.twist.twist.linear.y, -2.0)
        close(node.output_odom.twist.twist.linear.z, -1.0)
        close(node.output_odom.twist.twist.angular.x, 0.1)
        close(node.output_odom.twist.twist.angular.y, -0.2)
        close(node.output_odom.twist.twist.angular.z, 0.3)

        registered = cloud(99, 999, "upstream_cloud", [(8.0, 9.0, 10.0)])
        for _ in range(3):
            node.cloud_pub.publish(registered)
            rclpy.spin_once(node, timeout_sec=0.05)
        wait_for(lambda: node.output_cloud is not None, node, 5.0)
        assert node.output_cloud.header.stamp == odom.header.stamp
        assert node.output_cloud.header.frame_id == "drone0/livo_world"
        assert node.output_cloud.data == registered.data
        first_cloud_count = node.output_cloud_count
        for _ in range(3):
            node.cloud_pub.publish(registered)
            rclpy.spin_once(node, timeout_sec=0.05)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert node.output_cloud_count == first_cloud_count
        print("PASS: FAST-LIVO2 ROS 2 adapter integration semantics verified")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise
