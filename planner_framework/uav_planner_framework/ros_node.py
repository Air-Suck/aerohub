"""ROS 2 wrapper for task resolution, lifecycle, and fleet setpoint sampling."""

from __future__ import annotations

import math
import os
import signal
import subprocess
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
    BehaviorResult,
    FleetSetpoint,
    PlannerFailure,
    TimedTrajectorySegment as TimedTrajectorySegmentMsg,
    VehicleSetpoint,
)
from uav_planner_interfaces.srv import ControlPlanner

from .behavior import BehaviorLifecycle, BehaviorStatus, TransitionError
from .completion import FleetCompletion
from .registry import ConfigError, PlannerRegistry
from .ros_conversions import fill_setpoint_message, seconds_to_time, segment_from_msg
from .trajectory import OutputGateway, Setpoint, Vector3


def _read_yaml(path: str) -> dict:
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ConfigError(f"{path} must contain a mapping")
    return value


class PlannerFrameworkNode(Node):
    def __init__(self) -> None:
        super().__init__("planner_framework")
        self.declare_parameter("registry_root", "")
        self.declare_parameter("resolved_dir", "resolved")
        self.declare_parameter("sample_rate_hz", 50.0)
        registry_root = self.get_parameter("registry_root").value
        if not registry_root:
            raise RuntimeError("registry_root parameter is required")
        self.registry = PlannerRegistry(registry_root)
        self.lifecycle = BehaviorLifecycle()
        self.gateway: OutputGateway | None = None
        self.resolved: dict | None = None
        self.trajectory_wait_started_at: float | None = None
        self.positions: dict[str, Vector3] = {}
        self.pose_subscriptions = []
        self.completion: FleetCompletion | None = None
        self.active_started_at: float | None = None
        self.active_elapsed = 0.0
        self.planner_process: subprocess.Popen | None = None
        self.output_process: subprocess.Popen | None = None
        self.process_logs = []
        self.status_publisher = self.create_publisher(BehaviorResult, "planner/status", 10)
        self.setpoint_publisher = self.create_publisher(FleetSetpoint, "planner/setpoints", 10)
        trajectory_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TimedTrajectorySegmentMsg,
            "planner/trajectory",
            self._on_trajectory,
            trajectory_qos,
        )
        self.create_subscription(PlannerFailure, "planner/failure", self._on_failure, 10)
        self.create_service(ControlPlanner, "planner/control", self._on_control)
        frequency = float(self.get_parameter("sample_rate_hz").value)
        self.create_timer(1.0 / frequency, self._tick)

    def _on_control(self, request, response):
        try:
            if request.command == request.START:
                self._start(request.task_config_path, request.deployment_config_path)
            elif request.command == request.PAUSE:
                self._freeze_active_time()
                self.lifecycle.pause()
                self._publish_hover()
                self._stop_planner_adapter()
                self.gateway.clear()
                self.completion.reset()
                self.trajectory_wait_started_at = None
            elif request.command == request.RESUME:
                self.lifecycle.resume()
                self.gateway.clear()
                self.completion.reset()
                self.active_started_at = self._now()
                self.trajectory_wait_started_at = self._now()
                self._start_planner_adapter()
            elif request.command == request.CANCEL:
                self._freeze_active_time()
                self._publish_hover()
                self._stop_planner_adapter()
                self.lifecycle.cancel()
                self.gateway.clear()
                self.completion.reset()
                self.trajectory_wait_started_at = None
            elif request.command != request.GET_STATUS:
                raise TransitionError("unknown command")
            response.accepted = True
            response.message = "ok"
        except (ConfigError, OSError, yaml.YAMLError, TransitionError) as exc:
            response.accepted = False
            response.message = str(exc)
        response.result = self._result_message()
        self.status_publisher.publish(response.result)
        return response

    def _start(self, task_path: str, deployment_path: str) -> None:
        try:
            resolved = self.registry.resolve(_read_yaml(task_path), _read_yaml(deployment_path))
        except (ConfigError, OSError, yaml.YAMLError):
            try:
                self.lifecycle.reject("", "INVALID_TASK")
            except TransitionError:
                pass
            raise
        resolved["time"] = {
            "use_sim_time": bool(self.get_parameter("use_sim_time").value),
            "clock_topic": "/clock",
        }
        task_id = resolved["task"]["task_id"]
        self.lifecycle.start(task_id)
        self._stop_runtime()
        origins = {
            vehicle: Vector3(*item["px4_origin_in_fleet_ned"])
            for vehicle, item in resolved["deployment"].items()
        }
        self.resolved = resolved
        self.gateway = OutputGateway(task_id, origins)
        now = self._now()
        self.trajectory_wait_started_at = now
        self.active_started_at = now
        self.active_elapsed = 0.0
        self.positions = {}
        targets = {
            vehicle: Vector3(
                float(target["x"]), float(target["y"]), float(target["z"])
            )
            for vehicle, target in resolved["targets"].items()
        }
        completion = resolved["completion"]
        self.completion = FleetCompletion(
            targets,
            float(completion["position_tolerance"]),
            float(completion["hold_duration"]),
        )
        for subscription in self.pose_subscriptions:
            self.destroy_subscription(subscription)
        self.pose_subscriptions = []
        for vehicle in resolved["vehicles"]:
            topic = resolved["deployment"][vehicle]["topics"]["pose"]
            self.pose_subscriptions.append(self.create_subscription(
                PoseStamped,
                topic,
                lambda message, vehicle_id=vehicle: self._on_pose(vehicle_id, message),
                qos_profile_sensor_data,
            ))
        output = Path(self.get_parameter("resolved_dir").value) / f"{task_id}.yaml"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.safe_dump(resolved, sort_keys=False), encoding="utf-8")
        self._start_runtime(output)

    def _on_trajectory(self, message: TimedTrajectorySegmentMsg) -> None:
        snapshot = self.lifecycle.snapshot
        if snapshot is None or snapshot.status != BehaviorStatus.RUNNING:
            return
        if message.task_id != snapshot.task_id:
            return
        try:
            if self.gateway.accept(segment_from_msg(message), self._now()):
                self.trajectory_wait_started_at = None
        except ConfigError as exc:
            self._fail("INVALID_TRAJECTORY_SET", str(exc))

    def _on_failure(self, message: PlannerFailure) -> None:
        snapshot = self.lifecycle.snapshot
        if (
            snapshot is None
            or snapshot.status != BehaviorStatus.RUNNING
            or message.task_id != snapshot.task_id
        ):
            return
        reason = message.reason or "PLANNER_ERROR"
        self._fail(reason, message.detail)

    def _tick(self) -> None:
        snapshot = self.lifecycle.snapshot
        if snapshot is None or snapshot.status != BehaviorStatus.RUNNING or self.gateway is None:
            return
        now = self._now()
        if self.planner_process is not None and self.planner_process.poll() is not None:
            self._fail("PLANNER_ERROR", "planner adapter exited")
            return
        if self.output_process is not None and self.output_process.poll() is not None:
            self._fail("OUTPUT_ADAPTER_ERROR", "output adapter exited")
            return
        task_timeout = float(self.resolved["completion"]["task_timeout"])
        if self._active_time(now) >= task_timeout:
            self._fail("TASK_TIMEOUT")
            return
        if self.completion.update(self.positions, now):
            self._freeze_active_time(now)
            self.lifecycle.succeed()
            self._stop_planner_adapter()
            self.trajectory_wait_started_at = None
            self.status_publisher.publish(self._result_message())
            return
        timeout = float(self.resolved["planner"]["parameters"]["trajectory_timeout"])
        if self.gateway.trajectory_timed_out(now, timeout, self.trajectory_wait_started_at):
            self._fail("TRAJECTORY_TIMEOUT")
            return
        if self.gateway.segment is None:
            return
        message = FleetSetpoint()
        message.task_id = snapshot.task_id
        seconds_to_time(now, message.sampled_at)
        for vehicle_id in self.gateway.origins:
            item = VehicleSetpoint()
            fill_setpoint_message(item, vehicle_id, self.gateway.sample(vehicle_id, now))
            message.setpoints.append(item)
        self.setpoint_publisher.publish(message)

    def _on_pose(self, vehicle_id: str, message: PoseStamped) -> None:
        point = message.pose.position
        origin = self.gateway.origins[vehicle_id]
        self.positions[vehicle_id] = Vector3(
            float(point.y) + origin.x,
            float(point.x) + origin.y,
            -float(point.z) + origin.z,
        )

    def _publish_hover(self) -> None:
        snapshot = self.lifecycle.snapshot
        if snapshot is None or self.gateway is None or set(self.positions) != set(self.gateway.origins):
            return
        message = FleetSetpoint()
        message.task_id = snapshot.task_id
        seconds_to_time(self._now(), message.sampled_at)
        nan = Vector3(math.nan, math.nan, math.nan)
        for vehicle_id, position in self.positions.items():
            origin = self.gateway.origins[vehicle_id]
            local = Vector3(
                position.x - origin.x,
                position.y - origin.y,
                position.z - origin.z,
            )
            item = VehicleSetpoint()
            fill_setpoint_message(item, vehicle_id, Setpoint(local, nan, nan, math.nan))
            message.setpoints.append(item)
        self.setpoint_publisher.publish(message)

    def _fail(self, reason: str, detail: str = "") -> None:
        self._publish_hover()
        self._stop_planner_adapter()
        self._freeze_active_time()
        self.lifecycle.fail(reason)
        self.trajectory_wait_started_at = None
        if self.completion is not None:
            self.completion.reset()
        self.status_publisher.publish(self._result_message(detail))

    def _start_runtime(self, resolved_path: Path) -> None:
        self._start_output_adapter(resolved_path)
        self._start_planner_adapter(resolved_path)

    def _start_planner_adapter(self, resolved_path: Path | None = None) -> None:
        if self.planner_process is not None and self.planner_process.poll() is None:
            return
        if resolved_path is None:
            task_id = self.lifecycle.snapshot.task_id
            resolved_path = Path(self.get_parameter("resolved_dir").value) / f"{task_id}.yaml"
        use_sim_time = str(bool(self.get_parameter("use_sim_time").value)).lower()
        command = [
            str(part).format(resolved_config=str(resolved_path), use_sim_time=use_sim_time)
            for part in self.resolved["planner"]["runtime"]["command"]
        ]
        self.planner_process = self._spawn(command, resolved_path.with_suffix(".planner.log"))

    def _start_output_adapter(self, resolved_path: Path) -> None:
        use_sim_time = str(bool(self.get_parameter("use_sim_time").value)).lower()
        command = [
            "ros2", "run", "uav_as2_setpoint_adapter", "as2_setpoint_adapter_node",
            "--ros-args", "-p", f"resolved_config:={resolved_path}",
            "-p", f"use_sim_time:={use_sim_time}",
        ]
        self.output_process = self._spawn(command, resolved_path.with_suffix(".output.log"))

    def _spawn(self, command: list[str], log_path: Path) -> subprocess.Popen:
        log = log_path.open("w", encoding="utf-8")
        self.process_logs.append(log)
        environment = os.environ.copy()
        environment.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
        return subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )

    def _stop_planner_adapter(self) -> None:
        self._terminate(self.planner_process)
        self.planner_process = None

    def _stop_runtime(self) -> None:
        self._stop_planner_adapter()
        self._terminate(self.output_process)
        self.output_process = None
        for log in self.process_logs:
            log.close()
        self.process_logs = []

    @staticmethod
    def _terminate(process: subprocess.Popen | None) -> None:
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)

    def _active_time(self, now: float | None = None) -> float:
        if self.active_started_at is None:
            return self.active_elapsed
        current = self._now() if now is None else now
        return self.active_elapsed + current - self.active_started_at

    def _freeze_active_time(self, now: float | None = None) -> None:
        if self.active_started_at is None:
            return
        current = self._now() if now is None else now
        self.active_elapsed += current - self.active_started_at
        self.active_started_at = None

    def _result_message(self, detail: str = "") -> BehaviorResult:
        message = BehaviorResult()
        message.behavior_type = "planner"
        snapshot = self.lifecycle.snapshot
        if snapshot is None:
            message.overall_status = BehaviorResult.REJECTED
            message.failure_reason = "NOT_STARTED"
            return message
        message.task_id = snapshot.task_id
        message.overall_status = int(snapshot.status)
        message.failure_reason = snapshot.failure_reason
        if detail:
            self.get_logger().warning(detail)
        return message

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerFrameworkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node._stop_runtime()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
