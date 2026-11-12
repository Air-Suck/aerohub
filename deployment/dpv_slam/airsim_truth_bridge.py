#!/usr/bin/env python3
"""Publish timestamped AirSim truth for DPV-SLAM scoring only, never control."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from uav_as2_sensor_adapter.rpc import Rpc


class TruthBridge(Node):
    def __init__(self):
        super().__init__("dpv_slam_airsim_truth_bridge")
        self.declare_parameter("rpc_host", "127.0.0.1")
        self.declare_parameter("rpc_port", 41471)
        self.declare_parameter("vehicle_name", "PX4")
        self.declare_parameter("publish_period_s", 0.005)
        period = float(self.get_parameter("publish_period_s").value)
        if not 0.001 <= period <= 0.2:
            raise ValueError("publish_period_s must be between 0.001 and 0.2")
        self.rpc = Rpc(
            str(self.get_parameter("rpc_host").value),
            int(self.get_parameter("rpc_port").value), timeout=1.0)
        self.vehicle = str(self.get_parameter("vehicle_name").value)
        self.publisher = self.create_publisher(
            PoseStamped, "/dpv_slam/validation/airsim_truth", 20)
        self.last_stamp = 0
        self.create_timer(period, self.publish_truth)

    def publish_truth(self):
        try:
            state = self.rpc.call("getMultirotorState", self.vehicle)
            stamp = int(state["timestamp"])
            if stamp <= self.last_stamp:
                return
            kinematics = state["kinematics_estimated"]
            position = kinematics["position"]
            orientation = kinematics["orientation"]
            yaw_ned = math.atan2(
                2 * (orientation["w_val"] * orientation["z_val"]
                     + orientation["x_val"] * orientation["y_val"]),
                1 - 2 * (orientation["y_val"] ** 2 + orientation["z_val"] ** 2),
            )
            yaw_enu = math.atan2(
                math.sin(math.pi / 2.0 - yaw_ned),
                math.cos(math.pi / 2.0 - yaw_ned))
            message = PoseStamped()
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                stamp, 1_000_000_000)
            message.header.frame_id = "airsim_truth_enu"
            message.pose.position.x = float(position["y_val"])
            message.pose.position.y = float(position["x_val"])
            message.pose.position.z = -float(position["z_val"])
            message.pose.orientation.z = math.sin(yaw_enu / 2.0)
            message.pose.orientation.w = math.cos(yaw_enu / 2.0)
            self.publisher.publish(message)
            self.last_stamp = stamp
        except Exception as error:
            self.get_logger().warning(f"AirSim truth RPC failed: {error!r}")


def main():
    rclpy.init()
    node = TruthBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
