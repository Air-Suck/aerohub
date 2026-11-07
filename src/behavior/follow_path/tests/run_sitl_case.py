#!/usr/bin/env python3
"""Run a Planner task against an already-started seven-UAV SITL stack."""
import argparse
import json
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from uav_planner_interfaces.msg import TimedTrajectorySegment
from uav_planner_interfaces.srv import ControlPlanner
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_FeedbackMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("planner", choices=["cat_ora", "swarm_formation"])
    parser.add_argument("--skip-takeoff", action="store_true")
    args = parser.parse_args()
    if not args.skip_takeoff:
        subprocess.run(["fleet_takeoff.py", "--vehicles", *[f"drone{i}" for i in range(7)],
                        "--height", "1.5", "--speed", "0.5", "--timeout", "90"], check=True)
    rclpy.init()
    node = Node("followpath_sitl_acceptance")
    client = node.create_client(ControlPlanner, "/planner/control")
    segments = []
    tracking_errors = {}
    for index in range(7):
        vehicle = f"drone{index}"
        node.create_subscription(FollowTimedPath_FeedbackMessage,
            f"/{vehicle}/follow_timed_path/_action/feedback",
            lambda message, v=vehicle: tracking_errors.__setitem__(v,
                max(tracking_errors.get(v, 0), message.feedback.tracking_error)), 10)
    node.create_subscription(TimedTrajectorySegment, "/planner/trajectory",
                             lambda message: segments.append(message.generated_at),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    assert client.wait_for_service(timeout_sec=15), "planner service unavailable"

    def call(request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done(), "planner service timeout"
        response = future.result()
        assert response.accepted, response.message
        return response.result

    share = "/opt/uav_planner_ws/install/share/uav_planner_framework/examples/"
    request = ControlPlanner.Request()
    request.command = request.START
    request.task_config_path = share + (
        "cat_ora_four_px4.yaml" if args.planner == "cat_ora" else "swarm_formation_seven_px4.yaml")
    request.deployment_config_path = share + (
        "deployment_four_px4.yaml" if args.planner == "cat_ora" else "deployment_seven_px4.yaml")
    result = call(request)
    deadline = time.monotonic()+300
    while result.overall_status not in (3, 4, 5) and time.monotonic()<deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        request = ControlPlanner.Request(); request.command=request.GET_STATUS
        result = call(request)
    evidence = {"planner": args.planner, "status": result.overall_status,
                "reason": result.failure_reason, "segments_received": len(segments),
                "max_tracking_errors": tracking_errors,
                "backend": "extended FollowPath Actions", "simulation": "PX4/Gazebo"}
    print(json.dumps(evidence), flush=True)
    assert result.overall_status == 3, evidence
    assert len(segments) >= (2 if args.planner == "swarm_formation" else 1), evidence
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
