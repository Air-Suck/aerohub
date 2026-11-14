#!/usr/bin/env python3
"""Minimal AS2 Takeoff -> GoTo -> Land backend acceptance mission."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import GoToWaypoint, Land, Takeoff
from as2_msgs.msg import PlatformInfo, YawMode
from geometry_msgs.msg import PointStamped, PoseStamped
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool


class Mission(Node):
    def __init__(self):
        super().__init__("simulator_basic_flight", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.platform = None
        self.pose = None
        self.clock_count = 0
        self.active_goal = None
        self.steps = []
        self.create_subscription(PlatformInfo, "/drone0/platform/info", self._platform, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/drone0/self_localization/pose", self._pose, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", lambda _: setattr(self, "clock_count", self.clock_count + 1), qos_profile_sensor_data)

    def _platform(self, msg): self.platform = msg
    def _pose(self, msg): self.pose = msg

    def wait(self, predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError("acceptance condition timed out")
            rclpy.spin_once(self, timeout_sec=0.05)

    def service(self, name, value):
        client = self.create_client(SetBool, name)
        self.wait(client.service_is_ready)
        future = client.call_async(SetBool.Request(data=value))
        self.wait(future.done)
        if not future.result().success:
            raise RuntimeError(f"{name} rejected")

    def action(self, action_type, name, goal, result_field, timeout=90):
        client = ActionClient(self, action_type, name)
        self.wait(client.server_is_ready)
        sent = client.send_goal_async(goal)
        self.wait(sent.done)
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError(f"{name} rejected")
        self.active_goal = handle
        result = handle.get_result_async()
        self.wait(result.done, timeout)
        envelope = result.result()
        if envelope.status != 4 or not getattr(envelope.result, result_field):
            raise RuntimeError(f"{name} failed")
        self.active_goal = None
        self.steps.append(name)

    def recover(self):
        errors = []
        if self.active_goal:
            try:
                handle = self.active_goal
                cancellation = handle.cancel_goal_async()
                self.wait(cancellation.done, 10)
                terminal = handle.get_result_async()
                self.wait(terminal.done, 10)
                self.active_goal = None
            except Exception as exc:
                errors.append(f"cancel: {exc}")
        # Landing is attempted independently of cancellation failure.
        if self.platform and self.platform.armed:
            try:
                self.action(Land, "/drone0/LandBehavior", Land.Goal(land_speed=0.5), "land_success")
                self.wait(lambda: not self.platform.armed, 20)
            except Exception as exc:
                errors.append(f"land: {exc}")
        return {"errors": errors, "landed_and_disarmed": bool(
            self.platform and not self.platform.armed)}

    def run(self):
        self.wait(lambda: self.platform is not None and self.platform.connected
                  and self.pose is not None and self.clock_count >= 10, 60)
        if self.platform.armed:
            raise RuntimeError("acceptance requires a disarmed vehicle")
        start = [self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z]
        self.service("/drone0/set_arming_state", True)
        self.wait(lambda: self.platform.armed)
        self.service("/drone0/set_offboard_mode", True)
        self.wait(lambda: self.platform.offboard)
        self.action(Takeoff, "/drone0/TakeoffBehavior", Takeoff.Goal(takeoff_height=2.0, takeoff_speed=0.5), "takeoff_success")
        self.wait(lambda: self.platform.status.state == 3, 10)
        goal = GoToWaypoint.Goal()
        goal.target_pose = PointStamped()
        goal.target_pose.header.frame_id = self.pose.header.frame_id
        goal.target_pose.point.x = start[0] + 3.0
        goal.target_pose.point.y = start[1]
        goal.target_pose.point.z = start[2] + 2.0
        goal.max_speed = 1.0
        goal.yaw = YawMode(mode=YawMode.KEEP_YAW, angle=0.0)
        self.action(GoToWaypoint, "/drone0/GoToBehavior", goal, "go_to_success")
        self.wait(lambda: sum((a - b) ** 2 for a, b in zip(
            (self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z),
            (goal.target_pose.point.x, goal.target_pose.point.y, goal.target_pose.point.z))) <= 0.5**2, 10)
        self.action(Land, "/drone0/LandBehavior", Land.Goal(land_speed=0.5), "land_success")
        self.wait(lambda: not self.platform.armed, 20)
        return {"success": True, "start_enu_m": start,
                "end_enu_m": [self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z],
                "clock_samples": self.clock_count, "landed_and_disarmed": True,
                "completed_actions": self.steps, "goal_tolerance_m": 0.5}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Mission()
    result = {"success": False, "landed_and_disarmed": False}
    try:
        result = node.run()
    except Exception as exc:
        result["error"] = str(exc)
        result["recovery"] = node.recover()
        result["landed_and_disarmed"] = result["recovery"]["landed_and_disarmed"]
    finally:
        (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result["success"] else 1


if __name__ == "__main__": raise SystemExit(main())
