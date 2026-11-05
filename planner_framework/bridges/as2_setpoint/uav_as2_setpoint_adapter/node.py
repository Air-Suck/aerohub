"""Publish shared fleet setpoints as Aerostack2 single-sample trajectory references."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from as2_msgs.msg import ControlMode, ControllerInfo, TrajectoryPoint, TrajectorySetpoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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
        self.trajectory_publishers = {}
        self.mode_ready = {vehicle: False for vehicle in self.vehicles}
        self.held_yaw = {}
        self.current_yaw = {}
        self._pose_subscriptions = []
        self.latest_setpoints = None

        for vehicle in self.vehicles:
            deployment = self.config["deployment"][vehicle]
            namespace = deployment["namespace"].strip("/")
            prefix = f"/{namespace}"
            self.trajectory_publishers[vehicle] = self.create_publisher(
                TrajectorySetpoints, f"{prefix}/motion_reference/trajectory", 1
            )
            self._pose_subscriptions.append(self.create_subscription(
                ControllerInfo, f"{prefix}/controller/info",
                lambda message, vehicle_id=vehicle: self._on_mode(vehicle_id, message),
                qos_profile_sensor_data,
            ))
            pose_topic = f"{prefix}/sensor_measurements/odom"
            self._pose_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    pose_topic,
                    lambda message, vehicle_id=vehicle: self._on_odometry(
                        vehicle_id, message
                    ),
                    qos_profile_sensor_data,
                )
            )

        self.create_subscription(FleetSetpoint, "planner/setpoints", self._on_setpoints, 1)

    def _on_odometry(self, vehicle, message):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        expected = self.config["deployment"][vehicle]["namespace"].strip("/") + "/odom"
        if pose.header.frame_id == expected:
            self._on_pose(vehicle, pose)

    def _on_pose(self, vehicle, message):
        self.current_yaw[vehicle] = quaternion_yaw(message.pose.orientation)
        if self.mode_ready[vehicle] and self.held_yaw.get(vehicle) is None:
            self.held_yaw[vehicle] = self.current_yaw[vehicle]

    def _on_setpoints(self, message):
        if message.task_id != self.task_id:
            return
        if {item.vehicle_id for item in message.setpoints} != set(self.vehicles):
            self.get_logger().error("dropping incomplete fleet setpoint")
            return
        self.latest_setpoints = message
        self._publish_if_ready()

    def _on_mode(self, vehicle, message):
        mode = message.input_control_mode
        ready = (mode.control_mode == ControlMode.TRAJECTORY and
                 mode.yaw_mode == ControlMode.YAW_ANGLE)
        if ready and not self.mode_ready[vehicle]:
            self.held_yaw[vehicle] = self.current_yaw.get(vehicle)
        if ready != self.mode_ready[vehicle]:
            self.latest_setpoints = None
        self.mode_ready[vehicle] = ready

    def _publish_if_ready(self):
        if self.latest_setpoints is None:
            return
        if not all(self.mode_ready.values()) or any(self.held_yaw.get(v) is None for v in self.vehicles):
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
                else self.held_yaw[vehicle]
            )

            trajectory = TrajectorySetpoints()
            trajectory.header.stamp = self.latest_setpoints.sampled_at
            trajectory.header.frame_id = header_frame
            point = TrajectoryPoint()
            point.position.x, point.position.y, point.position.z = position
            point.twist.x, point.twist.y, point.twist.z = ned_to_enu(item.velocity)
            point.acceleration.x, point.acceleration.y, point.acceleration.z = ned_to_enu(item.acceleration)
            point.yaw_angle = yaw
            trajectory.setpoints = [point]
            prepared.append((vehicle, trajectory))

        for vehicle, trajectory in prepared:
            self.trajectory_publishers[vehicle].publish(trajectory)
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
