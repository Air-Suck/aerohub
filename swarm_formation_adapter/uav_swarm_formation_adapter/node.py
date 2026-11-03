"""Publish rolling Swarm-Formation trajectories through the shared contract."""

from __future__ import annotations

import json
import math
import socket
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from uav_planner_interfaces.msg import (
    PlannerFailure,
    TimedTrajectoryPoint,
    TimedTrajectorySegment,
    VehicleTrajectory,
)


RELATIVE_ENU = {
    "drone0": (0.0, 0.0, 0.0),
    "drone1": (1.7321, -1.0, 0.0),
    "drone2": (0.0, -2.0, 0.0),
    "drone3": (-1.7321, -1.0, 0.0),
    "drone4": (-1.7321, 1.0, 0.0),
    "drone5": (0.0, 2.0, 0.0),
    "drone6": (1.7321, 1.0, 0.0),
}


def ned_to_enu(position):
    return float(position["y"]), float(position["x"]), -float(position["z"])


def enu_to_ned(position):
    return float(position[1]), float(position[0]), -float(position[2])


def local_enu_to_fleet_enu(position, origin_ned):
    return [
        float(position[0]) + float(origin_ned[1]),
        float(position[1]) + float(origin_ned[0]),
        float(position[2]) - float(origin_ned[2]),
    ]


def set_time(target, value):
    target.sec = int(value)
    target.nanosec = int(round((value - target.sec) * 1_000_000_000.0))
    if target.nanosec == 1_000_000_000:
        target.sec += 1
        target.nanosec = 0


def formation_center(config):
    centers = []
    for vehicle in config["vehicles"]:
        target = ned_to_enu(config["targets"][vehicle])
        relative = RELATIVE_ENU[vehicle]
        centers.append(tuple(target[index] - relative[index] for index in range(3)))
    center = tuple(sum(values) / len(values) for values in zip(*centers))
    if any(math.dist(candidate, center) > 0.05 for candidate in centers):
        raise ValueError("targets do not match the configured Swarm-Formation shape")
    return center


def validate_trajectory_starts(vehicles, poses, trajectories, max_distance):
    if len(trajectories) != len(vehicles):
        raise ValueError("Swarm-Formation returned an incomplete fleet trajectory")
    for vehicle, points in zip(vehicles, trajectories):
        if len(points) < 2:
            raise ValueError(f"Swarm-Formation returned too few points for {vehicle}")
        distance = math.dist(poses[vehicle], points[0])
        if not math.isfinite(distance) or distance > max_distance:
            raise ValueError(
                f"Swarm-Formation trajectory for {vehicle} starts {distance:.3f} m "
                f"from current position"
            )


class SwarmFormationAdapter(Node):
    def __init__(self):
        super().__init__("swarm_formation_adapter")
        self.declare_parameter("resolved_config", "")
        self.declare_parameter("proxy_host", "127.0.0.1")
        self.declare_parameter("proxy_port", 28501)
        path = self.get_parameter("resolved_config").value
        if not path:
            raise RuntimeError("resolved_config parameter is required")
        self.config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        self.vehicles = list(self.config["vehicles"])
        if self.vehicles != list(RELATIVE_ENU):
            raise RuntimeError("first deployment requires drone0..drone6")
        self.goal_center = formation_center(self.config)
        self.poses = {}
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            TimedTrajectorySegment, "planner/trajectory", qos
        )
        self.failure_publisher = self.create_publisher(
            PlannerFailure, "planner/failure", 10
        )
        self.pose_subscriptions = []
        for vehicle in self.vehicles:
            topic = self.config["deployment"][vehicle]["topics"]["pose"]
            self.pose_subscriptions.append(self.create_subscription(
                PoseStamped,
                topic,
                lambda message, vehicle_id=vehicle: self._on_pose(vehicle_id, message),
                qos_profile_sensor_data,
            ))
        period = float(self.config["planner"]["parameters"]["update_period"])
        self.create_timer(period, self._update)

    def _on_pose(self, vehicle_id, message):
        point = message.pose.position
        origin = self.config["deployment"][vehicle_id]["px4_origin_in_fleet_ned"]
        self.poses[vehicle_id] = local_enu_to_fleet_enu(
            [point.x, point.y, point.z], origin
        )

    def _update(self):
        if len(self.poses) != len(self.vehicles):
            return
        parameters = self.config["planner"]["parameters"]
        request = {
            "task_id": self.config["task"]["task_id"],
            "vehicles": [
                {"id": vehicle, "position_enu": self.poses[vehicle]}
                for vehicle in self.vehicles
            ],
            "goal_center_enu": self.goal_center,
            "sample_dt": parameters["sample_dt"],
            "horizon": parameters["horizon"],
            "lead": parameters["valid_from_lead"],
        }
        try:
            with socket.create_connection(
                (
                    self.get_parameter("proxy_host").value,
                    int(self.get_parameter("proxy_port").value),
                ),
                timeout=0.5,
            ) as connection:
                stream = connection.makefile("rwb")
                stream.write((json.dumps(request, separators=(",", ":")) + "\n").encode())
                stream.flush()
                response = json.loads(stream.readline().decode())
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Swarm-Formation proxy unavailable: {exc}")
            return
        if not response.get("ok"):
            if response.get("reason") != "WAITING_FOR_TRAJECTORIES":
                self.get_logger().warning(response.get("error", response.get("reason", "error")))
            return
        trajectories = response["trajectories_enu"]
        try:
            validate_trajectory_starts(
                self.vehicles,
                self.poses,
                trajectories,
                float(parameters["max_start_distance"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"rejecting unsafe Swarm-Formation trajectory: {exc}")
            failure = PlannerFailure()
            failure.task_id = self.config["task"]["task_id"]
            failure.reason = "INVALID_TRAJECTORY_SET"
            failure.detail = str(exc)
            self.failure_publisher.publish(failure)
            return
        self.publisher.publish(self._message(trajectories))

    def _message(self, trajectories):
        if len(trajectories) != len(self.vehicles) or any(len(points) < 2 for points in trajectories):
            raise RuntimeError("Swarm-Formation returned invalid trajectories")
        parameters = self.config["planner"]["parameters"]
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        message = TimedTrajectorySegment()
        message.interface_version = 1
        message.task_id = self.config["task"]["task_id"]
        set_time(message.generated_at, now)
        set_time(message.valid_from, now + float(parameters["valid_from_lead"]))
        message.is_final = False
        for vehicle, points in zip(self.vehicles, trajectories):
            trajectory = VehicleTrajectory()
            trajectory.vehicle_id = vehicle
            trajectory.valid_fields = 0
            for index, position in enumerate(points):
                point = TimedTrajectoryPoint()
                set_time(point.time_from_start, index * float(parameters["sample_dt"]))
                point.position.x, point.position.y, point.position.z = enu_to_ned(position)
                trajectory.points.append(point)
            message.vehicle_trajectories.append(trajectory)
        return message


def main(args=None):
    rclpy.init(args=args)
    node = SwarmFormationAdapter()
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
