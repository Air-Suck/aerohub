"""Fleet coordinator for the extended AS2 FollowPath execution component.

This module never publishes motion references. All trajectories retain their
original execution epoch; delayed requests fail rather than silently retiming.
"""
from __future__ import annotations

import copy
import math

from action_msgs.msg import GoalStatus
from as2_msgs.msg import ControllerInfo, ControlMode
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal

from .ros_conversions import time_to_seconds


def make_goal(segment, item, deployment, revision, speed):
    goal = FollowTimedPath.Goal()
    path = goal.path
    path.interface_version = segment.interface_version
    path.task_id = segment.task_id
    path.vehicle_id = item.vehicle_id
    path.revision = revision
    path.frame_id = deployment["namespace"].strip("/") + "/odom"
    path.generated_at = copy.deepcopy(segment.generated_at)
    path.valid_from = copy.deepcopy(segment.valid_from)
    path.is_final = segment.is_final
    path.valid_fields = item.valid_fields
    path.max_speed = float(speed)
    origin = deployment["px4_origin_in_fleet_ned"]
    for source in item.points:
        target = copy.deepcopy(source)
        target.position.x = source.position.y - origin[1]
        target.position.y = source.position.x - origin[0]
        target.position.z = -(source.position.z - origin[2])
        for field in ("velocity", "acceleration"):
            src, dst = getattr(source, field), getattr(target, field)
            dst.x, dst.y, dst.z = src.y, src.x, -src.z
        if item.valid_fields & 4:
            target.yaw = (math.pi / 2 - source.yaw + math.pi) % (2 * math.pi) - math.pi
        path.points.append(target)
    return goal


class FleetFollowPathClient:
    def __init__(self, node, resolved, on_failure):
        self.node = node
        self.config = resolved
        self.on_failure = on_failure
        self.actions, self.modifiers, self.stoppers = {}, {}, {}
        self.handles = {}
        self.results = set()
        self.pending = set()
        self.revision = 0
        self.failed = False
        self.closed = False
        self.deadline = None
        self.generation = 0
        self.stop_futures = []
        self.stop_feedback = set()
        self.stop_subscriptions = []
        self.stop_complete = False
        self.cancel_started = False
        for vehicle in resolved["vehicles"]:
            ns = resolved["deployment"][vehicle]["namespace"].strip("/")
            self.stop_subscriptions.append(node.create_subscription(
                ControllerInfo, f"/{ns}/controller/info",
                lambda msg, v=vehicle: self._stop_mode(v, msg), qos_profile_sensor_data))
            name = f"/{ns}/follow_timed_path"
            self.actions[vehicle] = ActionClient(node, FollowTimedPath, name)
            self.modifiers[vehicle] = node.create_client(
                FollowTimedPath_SendGoal, name + "/_behavior/modify")
            self.stoppers[vehicle] = node.create_client(Trigger, name + "/_behavior/stop")

    @property
    def ready(self):
        return all(client.server_is_ready() for client in self.actions.values()) and all(
            client.service_is_ready() for client in self.modifiers.values()) and all(
            client.service_is_ready() for client in self.stoppers.values())

    @property
    def complete(self):
        return not self.failed and self.results == set(self.actions)

    def submit(self, segment):
        if self.closed or self.failed:
            return
        if self.pending:
            self._fail("EXECUTION_BUSY: previous fleet request is unacknowledged")
            return
        now = self.node.get_clock().now().nanoseconds / 1e9
        self.deadline = time_to_seconds(segment.valid_from)
        if self.deadline <= now:
            self._fail("LATE_TRAJECTORY")
            return
        if not all(c.server_is_ready() for c in self.actions.values()):
            self._fail("FOLLOW_PATH_UNAVAILABLE")
            return
        if self.revision and (set(self.handles) != set(self.actions) or self.results):
            self._fail("INVALID_EXECUTION_STATE")
            return
        if self.revision and not all(c.service_is_ready() for c in self.modifiers.values()):
            self._fail("FOLLOW_PATH_MODIFY_UNAVAILABLE")
            return
        self.revision += 1
        self.pending = set(self.actions)
        generation = self.generation
        for item in segment.vehicle_trajectories:
            vehicle = item.vehicle_id
            goal = make_goal(segment, item, self.config["deployment"][vehicle],
                             self.revision, self.config["planner"]["parameters"].get("max_velocity", 1.0))
            if self.revision == 1:
                future = self.actions[vehicle].send_goal_async(goal)
                future.add_done_callback(lambda f, v=vehicle, g=generation: self._accepted(v, g, f))
            else:
                request = FollowTimedPath_SendGoal.Request()
                request.goal = goal
                future = self.modifiers[vehicle].call_async(request)
                future.add_done_callback(lambda f, v=vehicle, g=generation: self._modified(v, g, f))

    def _accepted(self, vehicle, generation, future):
        try:
            handle = future.result()
            if self.closed or generation != self.generation or self.failed:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._fail(f"FOLLOW_PATH_REJECTED:{vehicle}")
                return
            self.handles[vehicle] = handle
            self.pending.discard(vehicle)
            handle.get_result_async().add_done_callback(
                lambda f, v=vehicle, g=generation: self._result(v, g, f))
        except Exception as exc:
            self._fail(f"FOLLOW_PATH_ERROR:{vehicle}:{exc}")

    def _modified(self, vehicle, generation, future):
        if self.closed or generation != self.generation or self.failed:
            return
        try:
            if not future.result().accepted:
                self._fail(f"FOLLOW_PATH_UPDATE_REJECTED:{vehicle}")
                return
            self.pending.discard(vehicle)
        except Exception as exc:
            self._fail(f"FOLLOW_PATH_UPDATE_ERROR:{vehicle}:{exc}")

    def _result(self, vehicle, generation, future):
        if self.closed or generation != self.generation or self.failed:
            return
        try:
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
                self._fail(f"FOLLOW_PATH_FAILED:{vehicle}:{result.result.reason}")
                return
            self.results.add(vehicle)
        except Exception as exc:
            self._fail(f"FOLLOW_PATH_RESULT_ERROR:{vehicle}:{exc}")

    def tick(self, now):
        if not self.closed and not self.failed and self.pending and now >= self.deadline:
            self._fail("FLEET_ACK_TIMEOUT")

    def _fail(self, reason):
        if self.closed or self.failed:
            return
        self.failed = True
        self.cancel()
        self.on_failure("EXECUTION_FAILED", reason)

    def cancel(self):
        if self.cancel_started:
            return
        self.cancel_started = True
        self.generation += 1
        for handle in self.handles.values():
            handle.cancel_goal_async()
        # Also stop a server whose acceptance response has not arrived yet.
        for client in self.stoppers.values():
            if client.service_is_ready():
                self.stop_futures.append(client.call_async(Trigger.Request()))
        self.handles.clear()
        self.pending.clear()
        self.results.clear()

    def _stop_mode(self, vehicle, message):
        if not self.cancel_started or self.stop_complete:
            return
        if message.input_control_mode.control_mode in (ControlMode.HOVER, ControlMode.POSITION):
            self.stop_feedback.add(vehicle)
        else:
            self.stop_feedback.discard(vehicle)

    @property
    def quiescent(self):
        if self.stop_complete:
            return True
        confirmed = self.cancel_started and self.stop_feedback == set(self.actions) and len(self.stop_futures) == len(self.actions) and all(
            future.done() and future.exception() is None and future.result().success
            for future in self.stop_futures)
        self.stop_complete = confirmed
        return confirmed

    def close(self):
        self.cancel()
        self.closed = True
        # Keep clients alive until outstanding cancellation/stop responses drain.
