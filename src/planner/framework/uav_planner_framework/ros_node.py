"""ROS 2 wrapper for task resolution, lifecycle, and fleet setpoint sampling."""

from __future__ import annotations

import math
import json
import os
import subprocess
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
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
from .goal_assignment import match_goal_slots
from .registry import ConfigError, PlannerRegistry
from .ros_conversions import fill_setpoint_message, seconds_to_time, segment_from_msg
from .trajectory import OutputGateway, Setpoint, Vector3
from .processes import terminate_session
from .follow_path_client import FleetFollowPathClient


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
        self.declare_parameter("runtime_startup_timeout", 0.0)
        self.declare_parameter('output_vehicle_id','')
        self.declare_parameter("execution_backend", "follow_path")
        self.execution_backend = self.get_parameter("execution_backend").value
        if self.execution_backend not in ("follow_path", "legacy_setpoint"):
            raise RuntimeError("unknown execution_backend")
        self.execution = None
        self.retired_executions = []
        registry_root = self.get_parameter("registry_root").value
        if not registry_root:
            raise RuntimeError("registry_root parameter is required")
        self.registry = PlannerRegistry(registry_root)
        self.lifecycle = BehaviorLifecycle()
        self.gateway: OutputGateway | None = None
        self.resolved: dict | None = None
        self.trajectory_wait_started_at: float | None = None
        self.positions: dict[str, Vector3] = {}
        self.pose_stamps = {}
        self.pose_subscriptions = []
        self.goal_assignment = None
        self.completion: FleetCompletion | None = None
        self.active_started_at: float | None = None
        self.active_elapsed = 0.0
        self.planner_process: subprocess.Popen | None = None
        self.output_process: subprocess.Popen | None = None
        self.process_logs = []
        self.assignment_publisher = self.create_publisher(String, "planner/goal_assignment",
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.execution_publisher = self.create_publisher(String, "planner/execution_state", 10)
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
                self._notify_cat_stop()
                self._publish_hover()
                self._stop_planner_adapter()
                self.gateway.clear()
                self.completion.reset()
                self.trajectory_wait_started_at = None
            elif request.command == request.RESUME:
                self._require_clock()
                if self.execution is not None and not self.execution.quiescent:
                    raise TransitionError("waiting for FollowPath stop confirmation")
                self.lifecycle.resume()
                self.gateway.clear()
                self.completion.reset()
                self.active_started_at = self._now()
                self.trajectory_wait_started_at = self._now()
                if self.execution_backend == "follow_path":
                    self._new_execution()
                self._start_planner_adapter()
            elif request.command == request.CANCEL:
                self._freeze_active_time()
                self.lifecycle.cancel()
                self._notify_cat_stop()
                self._publish_hover()
                self._stop_planner_adapter()
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

    def _require_clock(self) -> None:
        if bool(self.get_parameter("use_sim_time").value) and self._now() <= 0:
            raise TransitionError("CLOCK_NOT_READY")

    def _start(self, task_path: str, deployment_path: str) -> None:
        # DDS service discovery can precede this process's first /clock sample.
        self._require_clock()
        if self.execution is not None and not self.execution.quiescent and self.execution.cancel_started:
            raise TransitionError("waiting for FollowPath stop confirmation")
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
        if self.execution_backend == "follow_path":
            self._new_execution()
        self.gateway = OutputGateway(task_id, origins)
        now = self._now()
        self.trajectory_wait_started_at = now
        self.active_started_at = now
        self.active_elapsed = 0.0
        self.positions = {}
        self.pose_stamps = {}
        self.goal_assignment = None
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
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
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
            segment = segment_from_msg(message)
            binding = None
            if self.resolved['planner']['id'] == 'cat_ora':
                if not segment.is_final:
                    raise ConfigError('CAT-ORA assignment requires a final batch plan')
                slots = self.resolved['targets']
                ids = [item.vehicle_id for item in segment.vehicle_trajectories]
                if len(ids)!=len(set(ids)) or any(not item.points for item in segment.vehicle_trajectories):
                    raise ConfigError('duplicate vehicle or empty trajectory in assignment')
                slot_ids = match_goal_slots(
                    {item.vehicle_id:(item.points[-1].position.x,item.points[-1].position.y,item.points[-1].position.z)
                     for item in segment.vehicle_trajectories},
                    {key:(value['x'],value['y'],value['z']) for key,value in slots.items()},
                    self.resolved['planner']['parameters'].get('goal_assignment','fixed'))
                binding = {v:{'slot_id':slot,'target':dict(slots[slot])} for v,slot in slot_ids.items()}
                if self.goal_assignment and binding != self.goal_assignment['bindings']:
                    raise ConfigError('goal assignment cannot change during an active batch task')
            if self.gateway.accept(segment, self._now()):
                if binding is not None:
                    self.completion.targets = {v:Vector3(**item['target']) for v,item in binding.items()}
                    self.completion.reset()
                    audit = {'interface_version':1,'task_id':message.task_id,
                             'frame':'LOCAL_NED','generated_at':segment.generated_at,'valid_from':segment.valid_from,
                             'policy':self.resolved['planner']['parameters'].get('goal_assignment','fixed'),
                             'bindings':binding}
                    path=Path(self.get_parameter('resolved_dir').value)/(message.task_id+'.assignment.json')
                    path.write_text(json.dumps(audit,indent=2)+'\n')
                    self.goal_assignment=audit
                    self.assignment_publisher.publish(String(data=json.dumps(audit)))
                self.trajectory_wait_started_at = None
                if self.execution is not None:
                    self.execution.submit(message)
        except (ConfigError, OSError) as exc:
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
        if (
            snapshot is not None
            and snapshot.status == BehaviorStatus.RUNNING
            and self.execution_backend == "follow_path"
            and self.execution is None
        ):
            # Creating the action and service clients may wait on DDS graph
            # resources under a loaded multi-vehicle deployment. Keep that
            # work out of START so its response is sent before discovery.
            self._new_execution()
        if self.execution is not None:
            confirmed = self.execution.quiescent if self.execution.cancel_started else False
            state = {"task_id": self.lifecycle.snapshot.task_id,
                     "status": int(self.lifecycle.snapshot.status),
                     "failure_reason": self.lifecycle.snapshot.failure_reason,
                     "backend": "follow_path", "prepared": self.execution.ready,
                     "stop_requested": self.execution.cancel_started,
                     "stop_confirmed": {v: confirmed for v in self.resolved["vehicles"]}}
            self.execution_publisher.publish(String(data=json.dumps(state)))
        snapshot = self.lifecycle.snapshot
        if snapshot is None or snapshot.status != BehaviorStatus.RUNNING or self.gateway is None:
            return
        now = self._now()
        if self.execution is not None:
            if self.planner_process is None and not self.execution.closed:
                self._start_planner_adapter()
            self.execution.tick(now)
            if self.lifecycle.snapshot.status != BehaviorStatus.RUNNING:
                return
        if self.planner_process is not None and self.planner_process.poll() is not None:
            self._fail("PLANNER_ERROR", "planner adapter exited")
            return
        if self.output_process is not None and self.output_process.poll() is not None:
            self._fail("OUTPUT_ADAPTER_ERROR", "output adapter exited")
            return
        task_timeout = float(self.resolved["completion"]["task_timeout"])
        if self.resolved['planner']['id'] in ('cat_ora','swarm_formation') and any(now-stamp>1.0 or now-stamp<-.05 for stamp in self.pose_stamps.values()):
            self._fail('STATE_TIMEOUT','Local state is stale or clock changed')
            return
        if self._active_time(now) >= task_timeout:
            self._fail("TASK_TIMEOUT")
            return
        assignment_ready = (self.resolved['planner']['id'] != 'cat_ora' or
                            self.resolved['planner']['parameters'].get('goal_assignment','fixed') != 'optimize' or
                            self.goal_assignment is not None)
        if assignment_ready and self.completion.update(self.positions, now) and (
            self.execution is None or self.execution.complete
        ):
            self._freeze_active_time(now)
            self.lifecycle.succeed()
            if self.execution is not None:
                self.execution.close()
            self._notify_cat_stop()
            self._stop_planner_adapter()
            self.trajectory_wait_started_at = None
            self.status_publisher.publish(self._result_message())
            return
        timeout = float(self.resolved["planner"]["parameters"]["trajectory_timeout"])
        if self.gateway.segment is None:
            timeout = max(timeout, float(self.get_parameter("runtime_startup_timeout").value))
        if self.gateway.trajectory_timed_out(now, timeout, self.trajectory_wait_started_at):
            self._fail("TRAJECTORY_TIMEOUT")
            return
        if self.gateway.segment is None:
            return
        if self.execution is not None:
            return  # Sampling and motion references belong to the Behavior.
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
        if self.resolved['planner']['id'] in ('cat_ora','swarm_formation'):
            stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
            frame=self.resolved['deployment'][vehicle_id]['namespace'].strip('/')+'/odom'
            if message.header.frame_id!=frame or stamp<=0 or not -.05<=self._now()-stamp<=1.0 or stamp<=self.pose_stamps.get(vehicle_id,0) or not all(math.isfinite(x) for x in (point.x,point.y,point.z)):
                return
            self.pose_stamps[vehicle_id]=stamp
        origin = self.gateway.origins[vehicle_id]
        self.positions[vehicle_id] = Vector3(
            float(point.y) + origin.x,
            float(point.x) + origin.y,
            -float(point.z) + origin.z,
        )

    def _publish_hover(self) -> None:
        if self.execution is not None:
            self.execution.close()
            return
        # CAT-ORA's own AS2 adapter requests real HOVER and confirms its result.
        # Keep the legacy path for other planners until their separate migration.
        if self.resolved and self.resolved['planner']['id'] in ('cat_ora','swarm_formation'):
            return
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
        self.get_logger().error(f"Planner failed: {reason}: {detail}")
        if self.resolved and self.resolved['planner']['id'] in ('cat_ora','swarm_formation'):
            self.lifecycle.fail(reason)
            self._notify_cat_stop()
        self._publish_hover()
        self._stop_planner_adapter()
        self._freeze_active_time()
        if not (self.resolved and self.resolved['planner']['id'] in ('cat_ora','swarm_formation')):
            self.lifecycle.fail(reason)
        self.trajectory_wait_started_at = None
        if self.completion is not None:
            self.completion.reset()
        self.status_publisher.publish(self._result_message(detail))

    def _notify_cat_stop(self):
        # Deliver the AS2 stop intent before waiting for a planner process to exit.
        if self.resolved and self.resolved['planner']['id'] in ('cat_ora','swarm_formation'):
            self.status_publisher.publish(self._result_message())

    def _start_runtime(self, resolved_path: Path) -> None:
        if self.execution_backend == "legacy_setpoint":
            self._start_output_adapter(resolved_path)
            self._start_planner_adapter(resolved_path)
        # FollowPath starts from _tick after all execution servers are ready.

    def _start_planner_adapter(self, resolved_path: Path | None = None) -> None:
        if self.execution is not None and not self.execution.ready:
            return
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
        package,executable={'cat_ora':('uav_cat_ora_adapter','cat_ora_as2_adapter_node'),
                            'swarm_formation':('uav_swarm_formation_adapter','swarm_as2_adapter_node')}.get(self.resolved['planner']['id'], ('uav_as2_setpoint_adapter','as2_setpoint_adapter_node'))
        command = [
            "ros2", "run", package, executable,
            "--ros-args", "-p", f"resolved_config:={resolved_path}",
            "-p", f"use_sim_time:={use_sim_time}",
        ]
        owned=self.get_parameter('output_vehicle_id').value
        if owned:command.extend(['-p','vehicle_id:='+owned])
        self.output_process = self._spawn(command, resolved_path.with_suffix(".output.log"))

    def _spawn(self, command: list[str], log_path: Path) -> subprocess.Popen:
        self.get_logger().info("Starting runtime: " + " ".join(command))
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
        if self.execution is not None:
            self.execution.close()
        self._stop_planner_adapter()
        self._terminate(self.output_process)
        self.output_process = None
        for log in self.process_logs:
            log.close()
        self.process_logs = []

    def _new_execution(self) -> None:
        if self.execution is not None:
            self.execution.close()
            self.retired_executions.append(self.execution)
        self.execution = FleetFollowPathClient(self, self.resolved, self._fail)

    @staticmethod
    def _terminate(process: subprocess.Popen | None) -> None:
        terminate_session(process)

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
