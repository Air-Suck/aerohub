#!/usr/bin/env python3
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Probe(Node):
    def __init__(self):
        super().__init__("pin_slam_plugin_probe", namespace="drone0")
        self.pose = None
        self.twist = None
        self.publisher = self.create_publisher(
            Odometry, "pin_slam/odometry", qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "self_localization/pose", self._pose, 10)
        self.create_subscription(
            TwistStamped, "self_localization/twist", self._twist, 10)

    def _pose(self, message):
        self.pose = message

    def _twist(self, message):
        self.twist = message

    def publish(self, x):
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "pin_slam_map"
        message.child_frame_id = "pin_slam_lidar"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = 0.5
        message.twist.twist.angular.z = 0.1
        self.publisher.publish(message)


def main():
    rclpy.init()
    probe = Probe()
    deadline = time.monotonic() + 8.0
    x = 0.0
    while time.monotonic() < deadline and (probe.pose is None or probe.twist is None):
        probe.publish(x)
        x += 0.01
        rclpy.spin_once(probe, timeout_sec=0.05)
    if probe.pose is None or probe.twist is None:
        print("missing canonical AS2 pose or twist", file=sys.stderr)
        return 1
    if probe.pose.header.frame_id != "earth":
        print(f"unexpected pose frame: {probe.pose.header.frame_id}", file=sys.stderr)
        return 1
    if "base_link" not in probe.twist.header.frame_id:
        print(f"unexpected twist frame: {probe.twist.header.frame_id}", file=sys.stderr)
        return 1
    if abs(probe.twist.twist.linear.x - 0.5) > 1.0e-6:
        print("twist was not forwarded through the plugin", file=sys.stderr)
        return 1
    print("PIN-SLAM AS2 plugin published canonical pose and twist")
    probe.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
