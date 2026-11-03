#!/usr/bin/env python3
"""Minimal Aerostack2 controller boundary for adapter smoke tests."""

import sys

import rclpy
from as2_msgs.msg import ControlMode
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MockAs2(Node):
    def __init__(self, vehicles):
        super().__init__("mock_as2")
        self._pose_publishers = []
        self._services = []
        for vehicle in vehicles:
            prefix = f"/{vehicle}"
            self._pose_publishers.append(
                self.create_publisher(
                    PoseStamped, f"{prefix}/self_localization/pose", 10
                )
            )
            self._services.append(
                self.create_service(
                    SetControlMode,
                    f"{prefix}/controller/set_control_mode",
                    self._set_mode,
                )
            )
        self.create_timer(0.1, self._publish_poses)

    def _set_mode(self, request, response):
        response.success = (
            request.control_mode.control_mode == ControlMode.POSITION
            and request.control_mode.yaw_mode == ControlMode.YAW_ANGLE
        )
        return response

    def _publish_poses(self):
        for publisher in self._pose_publishers:
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.orientation.w = 1.0
            publisher.publish(message)


rclpy.init()
node = MockAs2(sys.argv[1:])
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
