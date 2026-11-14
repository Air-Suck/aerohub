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
        super().__init__("dpv_slam_plugin_probe", namespace="drone0")
        self.pose = None
        self.twist = None
        self.pose_count = 0
        self.pose_stamps = []
        self.pin_publisher = self.create_publisher(
            Odometry, "dpv_slam/odometry", qos_profile_sensor_data)
        self.local_publisher = self.create_publisher(
            Odometry, "sensor_measurements/odom", qos_profile_sensor_data)
        self.pose_subscription = self.create_subscription(
            PoseStamped, "self_localization/pose", self._pose, 10)
        self.twist_subscription = self.create_subscription(
            TwistStamped, "self_localization/twist", self._twist, 10)

    def _pose(self, message):
        self.pose = message
        self.pose_count += 1
        self.pose_stamps.append(
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)

    def _twist(self, message):
        self.twist = message

    def publish_local(self, x, stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "drone0/odom"
        message.child_frame_id = "drone0/base_link"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = 0.8
        message.twist.twist.angular.z = 0.2
        self.local_publisher.publish(message)

    def publish_visual(self, x, stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "dpv_slam_map"
        message.child_frame_id = "dpv_slam_camera"
        message.pose.pose.position.x = x
        # Inverse of the configured optical-camera <- FLU-base rotation, so
        # the resulting base orientation is identity in the visual map.
        message.pose.pose.orientation.x = -0.5
        message.pose.pose.orientation.y = 0.5
        message.pose.pose.orientation.z = -0.5
        message.pose.pose.orientation.w = 0.5
        self.pin_publisher.publish(message)


def spin_for(probe, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.01)


def wait_for_graph(probe, seconds=10.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if (
            probe.pin_publisher.get_subscription_count() > 0
            and probe.local_publisher.get_subscription_count() > 0
            and probe.count_publishers("self_localization/pose") > 0
            and probe.count_publishers("self_localization/twist") > 0
        ):
            return
        rclpy.spin_once(probe, timeout_sec=0.05)
    raise RuntimeError("timed out waiting for the plugin ROS graph")


def main():
    rclpy.init()
    probe = Probe()
    wait_for_graph(probe)

    # The paired samples define map->odom = +3 m.
    for index in range(20):
        stamp = probe.get_clock().now().to_msg()
        probe.publish_local(2.0 + index * 0.001, stamp)
        probe.publish_visual(5.0 + index * 0.001, stamp)
        spin_for(probe, 0.03)

    spin_for(probe, 2.0)

    if probe.pose is None or probe.twist is None:
        print("missing canonical AS2 pose or twist", file=sys.stderr)
        return 1

    probe.pose_count = 0
    probe.pose_stamps.clear()
    local_x = 2.0
    for _ in range(50):
        local_x += 0.01
        probe.publish_local(local_x, probe.get_clock().now().to_msg())
        spin_for(probe, 0.02)
    spin_for(probe, 0.2)

    if probe.pose.header.frame_id != "earth":
        print(f"unexpected pose frame: {probe.pose.header.frame_id}", file=sys.stderr)
        return 1
    if "base_link" not in probe.twist.header.frame_id:
        print(f"unexpected twist frame: {probe.twist.header.frame_id}", file=sys.stderr)
        return 1
    if abs(probe.twist.twist.linear.x - 0.8) > 1.0e-6:
        print("canonical twist did not come from local odometry", file=sys.stderr)
        return 1
    if abs(probe.pose.pose.position.x - (local_x + 3.0)) > 0.05:
        print(
            f"global correction was not composed with local pose: "
            f"{probe.pose.pose.position.x} vs {local_x + 3.0}; "
            f"canonical updates={probe.pose_count}; stamps={probe.pose_stamps[-5:]}",
            file=sys.stderr,
        )
        return 1
    if probe.pose_count < 20:
        print(
            f"canonical pose did not follow high-rate local odometry: "
            f"{probe.pose_count} updates",
            file=sys.stderr,
        )
        return 1
    if any(right <= left for left, right in zip(probe.pose_stamps, probe.pose_stamps[1:])):
        print("canonical pose timestamps are not strictly monotonic", file=sys.stderr)
        return 1

    print(
        "DPV-SLAM global correction composed with high-rate local pose/twist "
        f"({probe.pose_count} local updates)"
    )
    probe.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
