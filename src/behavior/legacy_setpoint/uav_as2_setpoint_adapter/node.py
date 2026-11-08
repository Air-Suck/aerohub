"""Publish shared fleet setpoints as Aerostack2 single-sample trajectory references."""

from __future__ import annotations

import math
import json
import os
import time
from pathlib import Path

import rclpy
import yaml
from as2_msgs.msg import ControlMode, ControllerInfo, TrajectoryPoint, TrajectorySetpoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

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


latest_state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class As2SetpointAdapter(Node):
    def __init__(self, node_name="as2_setpoint_adapter"):
        super().__init__(node_name)
        self.declare_parameter("resolved_config", "")
        path = self.get_parameter("resolved_config").value
        if not path:
            raise RuntimeError("resolved_config parameter is required")
        self.config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        self.task_id = self.config["task"]["task_id"]
        self.vehicles = list(self.config["vehicles"])
        self.expected_vehicles = list(self.vehicles)
        self.declare_parameter('vehicle_id','')
        owned=self.get_parameter('vehicle_id').value
        if owned:
            if owned not in self.vehicles:raise ValueError('vehicle_id is not in the task')
            self.vehicles=[owned]
        self.receipt_stats={'count':0,'max_wall_gap_s':0.0,'min_source_age_s':None,'max_source_age_s':None,
                            'duplicate_or_reordered':0}
        self.receipt_first=None;self.receipt_last=None;self.receipt_stamp=None
        self.trajectory_publishers = {}
        self.mode_ready = {vehicle: False for vehicle in self.vehicles}
        self.held_yaw = {}
        self.current_yaw = {}
        self._pose_subscriptions = []
        self.latest_setpoints = None
        self.mode_seen = set()
        self.setpoints_seen = 0
        self.published_counts = {vehicle:0 for vehicle in self.vehicles}
        self.state_publisher = self.create_publisher(String, 'planner/as2_bridge_state', 1)
        self.create_timer(0.1, self._publish_state)

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
                latest_state_qos,
            ))
            pose_topic = f"{prefix}/sensor_measurements/odom"
            self._pose_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    pose_topic,
                    lambda message, vehicle_id=vehicle: self._on_odometry(
                        vehicle_id, message
                    ),
                    latest_state_qos,
                )
            )

        self.create_subscription(FleetSetpoint, "planner/setpoints", self._on_setpoints, 1)

    def _publish_state(self):
        prepared = (set(self.current_yaw)==set(self.vehicles) and self.mode_seen==set(self.vehicles)
                    and all(math.isfinite(yaw) for yaw in self.current_yaw.values()) and self.setpoints_seen>0)
        message=String()
        state={'task_id':self.task_id,'prepared':prepared,
            'yaw_received':sorted(self.current_yaw),'mode_received':sorted(self.mode_seen),
            'mode_ready':self.mode_ready,'setpoints_received':self.setpoints_seen,
            'published':self.published_counts,'owned_vehicles':self.vehicles,
            'compute_id':os.environ.get('CATORA_COMPUTE_ID',self.get_name()),'receipt':dict(self.receipt_stats)}
        if self.receipt_first is not None and self.receipt_last>self.receipt_first:
            state['receipt']['hz']=(self.receipt_stats['count']-1)/(self.receipt_last-self.receipt_first)
        state.update(self.extra_state())
        message.data=json.dumps(state)
        self.state_publisher.publish(message)

    def extra_state(self):
        return {}

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
        if {item.vehicle_id for item in message.setpoints} != set(getattr(self,'expected_vehicles',self.vehicles)):
            self.get_logger().error("dropping incomplete fleet setpoint")
            return
        if hasattr(self,'receipt_stats'):
            now=time.monotonic();stamp=message.sampled_at.sec+message.sampled_at.nanosec/1e9
            age=self.get_clock().now().nanoseconds/1e9-stamp
            stats=self.receipt_stats;stats['count']+=1
            if self.receipt_last is not None:stats['max_wall_gap_s']=max(stats['max_wall_gap_s'],now-self.receipt_last)
            if self.receipt_stamp is not None and stamp<=self.receipt_stamp:stats['duplicate_or_reordered']+=1
            for key,fn in [('min_source_age_s',min),('max_source_age_s',max)]:stats[key]=age if stats[key] is None else fn(stats[key],age)
            if self.receipt_first is None:self.receipt_first=now
            self.receipt_last=now;self.receipt_stamp=stamp
        self.setpoints_seen += 1
        self.latest_setpoints = message
        self._publish_if_ready()

    def _on_mode(self, vehicle, message):
        self.mode_seen.add(vehicle)
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
            self.published_counts[vehicle] += 1
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
