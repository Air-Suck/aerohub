"""Publish shared fleet setpoints as Aerostack2 position references."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from as2_msgs.msg import ControlMode
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from uav_planner_interfaces.msg import FleetSetpoint


def ned_to_enu(position):
    return float(position.y), float(position.x), -float(position.z)


def px4_yaw_to_enu(yaw):
    value = math.pi / 2.0 - float(yaw)
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class As2SetpointAdapter(Node):
    def __init__(self):
        super().__init__("as2_setpoint_adapter")
        self.declare_parameter("resolved_config", "")
        path = self.get_parameter("resolved_config").value
        if not path:
            raise RuntimeError("resolved_config parameter is required")
        self.config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        self.task_id = self.config["task"]["task_id"]
        self.vehicles = list(self.config["vehicles"])
        self.speed_limit = float(
            self.config["planner"]["parameters"].get("max_velocity", 1.0)
        )
        self.pose_publishers = {}
        self.twist_publishers = {}
        self.mode_clients = {}
        self.mode_ready = {vehicle: False for vehicle in self.vehicles}
        self.mode_pending = {vehicle: False for vehicle in self.vehicles}
        self.current_yaw = {}
        self._pose_subscriptions = []
        self.latest_setpoints = None

        for vehicle in self.vehicles:
            deployment = self.config["deployment"][vehicle]
            namespace = deployment["namespace"].strip("/")
            prefix = f"/{namespace}"
            self.pose_publishers[vehicle] = self.create_publisher(
                PoseStamped, f"{prefix}/motion_reference/pose", 10
            )
            self.twist_publishers[vehicle] = self.create_publisher(
                TwistStamped, f"{prefix}/motion_reference/twist", 10
            )
            self.mode_clients[vehicle] = self.create_client(
                SetControlMode, f"{prefix}/controller/set_control_mode"
            )
            pose_topic = deployment["topics"]["pose"]
            self._pose_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    pose_topic,
                    lambda message, vehicle_id=vehicle: self._on_pose(
                        vehicle_id, message
                    ),
                    qos_profile_sensor_data,
                )
            )

        self.create_subscription(FleetSetpoint, "planner/setpoints", self._on_setpoints, 10)
        self.create_timer(0.5, self._tick)

    def _on_pose(self, vehicle, message):
        self.current_yaw[vehicle] = quaternion_yaw(message.pose.orientation)
        self._publish_if_ready()

    def _on_setpoints(self, message):
        if message.task_id != self.task_id:
            return
        if {item.vehicle_id for item in message.setpoints} != set(self.vehicles):
            self.get_logger().error("dropping incomplete fleet setpoint")
            return
        self.latest_setpoints = message
        self._publish_if_ready()

    def _tick(self):
        for vehicle in self.vehicles:
            if self.mode_ready[vehicle] or self.mode_pending[vehicle]:
                continue
            client = self.mode_clients[vehicle]
            if not client.service_is_ready():
                continue
            request = SetControlMode.Request()
            request.control_mode.control_mode = ControlMode.POSITION
            request.control_mode.yaw_mode = ControlMode.YAW_ANGLE
            self.mode_pending[vehicle] = True
            future = client.call_async(request)
            future.add_done_callback(
                lambda result, vehicle_id=vehicle: self._mode_result(vehicle_id, result)
            )
        self._publish_if_ready()

    def _mode_result(self, vehicle, future):
        self.mode_pending[vehicle] = False
        try:
            self.mode_ready[vehicle] = bool(future.result().success)
        except Exception as exc:
            self.get_logger().warning(f"{vehicle} control mode request failed: {exc}")
        self._publish_if_ready()

    def _publish_if_ready(self):
        if self.latest_setpoints is None:
            return
        if not all(self.mode_ready.values()) or set(self.current_yaw) != set(self.vehicles):
            return
        by_vehicle = {item.vehicle_id: item for item in self.latest_setpoints.setpoints}
        prepared = []
        for vehicle in self.vehicles:
            item = by_vehicle[vehicle]
            position = ned_to_enu(item.position)
            if not all(math.isfinite(value) for value in position):
                self.get_logger().error(f"dropping invalid position for {vehicle}")
                self.latest_setpoints = None
                return
            namespace = self.config["deployment"][vehicle]["namespace"].strip("/")
            header_frame = f"{namespace}/odom"
            yaw = (
                px4_yaw_to_enu(item.yaw)
                if math.isfinite(item.yaw)
                else self.current_yaw[vehicle]
            )

            pose = PoseStamped()
            pose.header.stamp = self.latest_setpoints.sampled_at
            pose.header.frame_id = header_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            limits = TwistStamped()
            limits.header = pose.header
            limits.twist.linear.x = self.speed_limit
            limits.twist.linear.y = self.speed_limit
            limits.twist.linear.z = self.speed_limit
            prepared.append((vehicle, limits, pose))

        for vehicle, limits, pose in prepared:
            self.twist_publishers[vehicle].publish(limits)
            self.pose_publishers[vehicle].publish(pose)
        self.latest_setpoints = None


def main(args=None):
    rclpy.init(args=args)
    node = As2SetpointAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
