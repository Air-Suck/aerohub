"""Convert CAT-ORA service output into a shared timed trajectory segment."""

from __future__ import annotations

import json
import math
import socket
from pathlib import Path

import rclpy
import yaml
from .state import checked_pose
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


def ned_to_enu(position):
    return [float(position["y"]), float(position["x"]), -float(position["z"])]


def enu_to_ned(position):
    return float(position[1]), float(position[0]), -float(position[2])


def local_enu_to_fleet_enu(position, origin_ned):
    return [
        float(position[0]) + float(origin_ned[1]),
        float(position[1]) + float(origin_ned[0]),
        float(position[2]) - float(origin_ned[2]),
    ]


def trajectories_reach_requested_goals(trajectories, goals, tolerance=1e-3):
    return len(trajectories) == len(goals) and all(
        points and math.dist(points[-1], goal) <= tolerance
        for points, goal in zip(trajectories, goals)
    )


def set_time(target, value):
    target.sec = int(value)
    target.nanosec = int(round((value - target.sec) * 1_000_000_000.0))
    if target.nanosec == 1_000_000_000:
        target.sec += 1
        target.nanosec = 0


latest_state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class CatOraAdapter(Node):
    def __init__(self):
        super().__init__("cat_ora_adapter")
        self.declare_parameter("resolved_config", "")
        self.declare_parameter("proxy_host", "127.0.0.1")
        self.declare_parameter("proxy_port", 28500)
        path = self.get_parameter("resolved_config").value
        if not path:
            raise RuntimeError("resolved_config parameter is required")
        self.config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        self.vehicles = list(self.config["vehicles"])
        self.poses = {}
        self.pose_stamps = {}
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
        self._pose_subscriptions = []
        for vehicle in self.vehicles:
            topic = self.config["deployment"][vehicle]["topics"]["pose"]
            self._pose_subscriptions.append(self.create_subscription(
                PoseStamped,
                topic,
                lambda message, vehicle_id=vehicle: self._on_pose(vehicle_id, message),
                latest_state_qos,
            ))
        self.published = False
        self.create_timer(1.0, self._try_generate)

    def _on_pose(self, vehicle_id, message):
        frame=self.config['deployment'][vehicle_id]['namespace'].strip('/')+'/odom'
        try:
            stamp=checked_pose(message,frame,self.get_clock().now().nanoseconds/1e9,self.pose_stamps.get(vehicle_id,0))
        except ValueError:
            return
        point = message.pose.position
        origin = self.config["deployment"][vehicle_id]["px4_origin_in_fleet_ned"]
        self.poses[vehicle_id] = local_enu_to_fleet_enu(
            [point.x, point.y, point.z], origin
        )
        self.pose_stamps[vehicle_id]=stamp

    def _try_generate(self):
        if self.published or len(self.poses) != len(self.vehicles):
            return
        now=self.get_clock().now().nanoseconds/1e9
        if any(not -.05<=now-stamp<=1.0 for stamp in self.pose_stamps.values()):
            return
        try:
            self._generate()
            self.published = True
        except Exception as exc:
            self.get_logger().warning(f"CAT-ORA generation unavailable: {exc}")

    def _generate(self):
        parameters = self.config["planner"]["parameters"]
        request = {
            "starts_enu": [self.poses[vehicle] for vehicle in self.vehicles],
            "goals_enu": [ned_to_enu(self.config["targets"][vehicle]) for vehicle in self.vehicles],
            "trajectory_dt": parameters["trajectory_dt"],
            "max_velocity": parameters["max_velocity"],
            "max_acceleration": parameters["max_acceleration"],
        }
        host = self.get_parameter("proxy_host").value
        port = int(self.get_parameter("proxy_port").value)
        with socket.create_connection((host, port), timeout=30.0) as connection:
            stream = connection.makefile("rwb")
            stream.write((json.dumps(request, separators=(",", ":")) + "\n").encode())
            stream.flush()
            response = json.loads(stream.readline().decode())
        now=self.get_clock().now().nanoseconds/1e9
        if any(not -.05<=now-stamp<=1.0 for stamp in self.pose_stamps.values()):
            raise RuntimeError('local state expired while waiting for CAT-ORA response')
        if not response.get("ok"):
            if response.get("reason") == "NO_SOLUTION":
                failure = PlannerFailure()
                failure.task_id = self.config["task"]["task_id"]
                failure.reason = "NO_SOLUTION"
                failure.detail = response.get("error", "CAT-ORA found no solution")
                self.get_logger().warning(failure.detail)
                self.failure_publisher.publish(failure)
                self.published = True
                return
            raise RuntimeError(response.get("error", "unknown CAT-ORA error"))
        trajectories = response["trajectories_enu"]
        if not trajectories_reach_requested_goals(trajectories, request["goals_enu"]):
            errors = [
                math.dist(points[-1], goal) if points else math.inf
                for points, goal in zip(trajectories, request["goals_enu"])
            ]
            failure = PlannerFailure()
            failure.task_id = self.config["task"]["task_id"]
            failure.reason = "NO_SOLUTION"
            failure.detail = "CAT-ORA reassigned a fixed per-vehicle goal; endpoint errors=" + ",".join(
                f"{error:.3f}" for error in errors
            )
            self.get_logger().warning(failure.detail)
            self.failure_publisher.publish(failure)
            self.published = True
            return
        self.publisher.publish(self._message(trajectories))
        self.get_logger().info("published CAT-ORA timed trajectory")

    def _message(self, trajectories):
        if len(trajectories) != len(self.vehicles) or any(len(points) < 2 for points in trajectories):
            raise RuntimeError("CAT-ORA returned invalid trajectories")
        longest = max(len(points) for points in trajectories)
        dt = float(self.config["planner"]["parameters"]["trajectory_dt"])
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        message = TimedTrajectorySegment()
        message.interface_version = 1
        message.task_id = self.config["task"]["task_id"]
        set_time(message.generated_at, now)
        set_time(message.valid_from, now + float(self.config["planner"]["parameters"]["valid_from_lead"]))
        message.is_final = True
        for vehicle, points in zip(self.vehicles, trajectories):
            trajectory = VehicleTrajectory()
            trajectory.vehicle_id = vehicle
            trajectory.valid_fields = 0
            padded = list(points) + [points[-1]] * (longest - len(points))
            for index, position in enumerate(padded):
                point = TimedTrajectoryPoint()
                set_time(point.time_from_start, index * dt)
                point.position.x, point.position.y, point.position.z = enu_to_ned(position)
                trajectory.points.append(point)
            message.vehicle_trajectories.append(trajectory)
        return message


def main(args=None):
    rclpy.init(args=args)
    node = CatOraAdapter()
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
