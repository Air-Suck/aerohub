#!/usr/bin/env python3
"""Minimal Aerostack2 controller boundary for adapter smoke tests."""

import sys

import rclpy
from as2_msgs.msg import ControlMode, ControllerInfo
from as2_msgs.srv import SetControlMode
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MockAs2(Node):
    def __init__(self, vehicles):
        super().__init__("mock_as2")
        self._pose_publishers = []
        self._services = []
        self._mode_publishers = []
        self._modes = {}
        for vehicle in vehicles:
            prefix = f"/{vehicle}"
            self._modes[vehicle] = ControlMode.UNSET
            self._mode_publishers.append((vehicle, self.create_publisher(ControllerInfo, f"{prefix}/controller/info", 10)))
            self._pose_publishers.append(
                self.create_publisher(
                    Odometry, f"{prefix}/sensor_measurements/odom", 10
                )
            )
            self._services.append(
                self.create_service(
                    SetControlMode,
                    f"{prefix}/controller/set_control_mode",
                    lambda req, res, v=vehicle: self._set_mode(v, req, res),
                )
            )
        self.create_timer(0.1, self._publish_poses)

    def _set_mode(self, vehicle, request, response):
        response.success = (
            request.control_mode.control_mode == ControlMode.TRAJECTORY
            and request.control_mode.yaw_mode == ControlMode.YAW_ANGLE
        )
        if response.success:
            self._modes[vehicle] = ControlMode.TRAJECTORY
        return response

    def _publish_poses(self):
        for vehicle, publisher in self._mode_publishers:
            message = ControllerInfo()
            message.input_control_mode.control_mode = self._modes[vehicle]
            message.input_control_mode.yaw_mode = ControlMode.YAW_ANGLE
            publisher.publish(message)
        for publisher in self._pose_publishers:
            message = Odometry()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = publisher.topic_name.split("/")[1] + "/odom"
            message.pose.pose.orientation.w = 1.0
            publisher.publish(message)


rclpy.init()
node = MockAs2(sys.argv[1:])
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
