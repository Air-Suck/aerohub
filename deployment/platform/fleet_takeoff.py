#!/usr/bin/env python3
"""Arm, enter offboard mode, and take off an AS2 fleet for integration tests."""

from __future__ import annotations

import argparse
import time

import rclpy
from as2_msgs.action import Takeoff
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool


class FleetTakeoff(Node):
    def __init__(self, vehicles: list[str]) -> None:
        super().__init__("fleet_takeoff")
        self.vehicles = vehicles
        self.arm_clients = {
            vehicle: self.create_client(SetBool, f"/{vehicle}/set_arming_state")
            for vehicle in vehicles
        }
        self.offboard_clients = {
            vehicle: self.create_client(SetBool, f"/{vehicle}/set_offboard_mode")
            for vehicle in vehicles
        }
        self.takeoff_clients = {
            vehicle: ActionClient(self, Takeoff, f"/{vehicle}/TakeoffBehavior")
            for vehicle in vehicles
        }
        self.localized = set()
        self.pose_subscriptions = [
            self.create_subscription(
                PoseStamped,
                f"/{vehicle}/self_localization/pose",
                lambda _message, vehicle_id=vehicle: self.localized.add(vehicle_id),
                qos_profile_sensor_data,
            )
            for vehicle in vehicles
        ]

    def wait_for_localization(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while self.localized != set(self.vehicles):
            if time.monotonic() >= deadline:
                missing = sorted(set(self.vehicles) - self.localized)
                raise RuntimeError("localization unavailable: " + ", ".join(missing))
            rclpy.spin_once(self, timeout_sec=0.1)

    def call_bool(self, clients, value: bool, timeout: float) -> None:
        for vehicle, client in clients.items():
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"{vehicle}: service unavailable")
            request = SetBool.Request()
            request.data = value
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            response = future.result()
            if response is None or not response.success:
                detail = response.message if response else "no response"
                raise RuntimeError(f"{vehicle}: command failed: {detail}")

    def takeoff(self, height: float, speed: float, timeout: float) -> None:
        for vehicle, client in self.takeoff_clients.items():
            if not client.wait_for_server(timeout_sec=timeout):
                raise RuntimeError(f"{vehicle}: takeoff action unavailable")
            goal = Takeoff.Goal()
            goal.takeoff_height = height
            goal.takeoff_speed = speed
            send_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError(f"{vehicle}: takeoff goal rejected")
            result_future = handle.get_result_async()
            deadline = time.monotonic() + timeout
            while not result_future.done():
                if time.monotonic() >= deadline:
                    handle.cancel_goal_async()
                    rclpy.spin_once(self, timeout_sec=0.5)
                    raise RuntimeError(f"takeoff timed out: {vehicle}")
                rclpy.spin_once(self, timeout_sec=0.1)
            if not result_future.result().result.takeoff_success:
                raise RuntimeError(f"takeoff failed: {vehicle}")
            print(f"Takeoff succeeded: {vehicle}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicles", nargs="+", default=[f"drone{i}" for i in range(4)])
    parser.add_argument("--height", type=float, default=1.5)
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = FleetTakeoff(args.vehicles)
    try:
        node.wait_for_localization(args.timeout)
        node.call_bool(node.arm_clients, True, args.timeout)
        node.call_bool(node.offboard_clients, True, args.timeout)
        node.takeoff(args.height, args.speed, args.timeout)
        print("Fleet takeoff succeeded: " + ", ".join(args.vehicles))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
