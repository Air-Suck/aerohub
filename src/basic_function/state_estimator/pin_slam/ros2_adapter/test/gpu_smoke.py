#!/usr/bin/env python3
"""Drive the real PIN-SLAM GPU node with deterministic moving LiDAR frames."""

import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def world_points():
    grid = np.linspace(-5.0, 5.0, 41, dtype=np.float32)
    height = np.linspace(-1.5, 2.5, 21, dtype=np.float32)
    points = []
    for x in grid:
        for y in grid:
            points.append((x, y, -1.5))
    for y in grid:
        for z in height:
            points.append((7.0, y, z))
            points.append((-5.0, y, z))
    for x in grid:
        for z in height:
            points.append((x, 5.0, z))
            points.append((x, -5.0, z))
    return np.asarray(points, dtype=np.float32)


def sensor_frame(points, x, yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    translated = points - np.array([x, 0.0, 0.0], dtype=np.float32)
    local = translated @ rotation
    ranges = np.linalg.norm(local, axis=1)
    return local[(ranges >= 0.8) & (ranges <= 25.0)]


class Probe(Node):
    def __init__(self):
        super().__init__("pin_slam_gpu_smoke_probe")
        self.publisher = self.create_publisher(
            PointCloud2,
            "/drone0/sensor_measurements/Lidar/points",
            qos_profile_sensor_data,
        )
        self.status_count = 0
        self.statuses = []
        self.odom = []
        self.neural = []
        self.create_subscription(
            DiagnosticArray,
            "/drone0/pin_slam/status",
            self._status,
            10,
        )
        self.create_subscription(
            Odometry,
            "/drone0/pin_slam/odometry",
            self._odom,
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/drone0/pin_slam/neural_points",
            self._neural,
            10,
        )

    def _status(self, message):
        for item in message.status:
            if item.name != "pin_slam/runtime":
                continue
            self.status_count += 1
            level = item.level
            if isinstance(level, (bytes, bytearray)):
                level = int.from_bytes(level, byteorder="little")
            self.statuses.append({
                "level": int(level),
                "message": item.message,
                "values": {value.key: value.value for value in item.values},
            })

    def _odom(self, message):
        self.odom.append(message)

    def _neural(self, message):
        if message.width > 0:
            self.neural.append(message)

    def spin_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(description)
            rclpy.spin_once(self, timeout_sec=0.1)

    def send(self, points, index):
        stamp = self.get_clock().now().to_msg()
        message = point_cloud2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id="drone0/Lidar"), points)
        previous = self.status_count
        self.publisher.publish(message)
        self.spin_until(
            lambda: self.status_count > previous,
            120.0,
            f"frame {index} was not processed",
        )
        status = self.statuses[-1]
        error_level = DiagnosticStatus.ERROR
        if isinstance(error_level, (bytes, bytearray)):
            error_level = int.from_bytes(error_level, byteorder="little")
        if status["level"] >= int(error_level):
            raise RuntimeError(f"frame {index} failed: {status}")
        if status["message"] in ("input_rejected", "input_dropped"):
            raise RuntimeError(f"frame {index} was not accepted: {status}")


def main():
    rclpy.init()
    probe = Probe()
    try:
        probe.spin_until(
            lambda: probe.publisher.get_subscription_count() > 0,
            30.0,
            "PIN-SLAM input subscriber was not discovered",
        )
        points = world_points()
        started = time.monotonic()
        for index in range(6):
            probe.send(sensor_frame(points, 0.08 * index, 0.004 * index), index)
        probe.spin_until(
            lambda: len(probe.odom) >= 3 and len(probe.neural) >= 3,
            10.0,
            "PIN-SLAM did not publish enough state and map samples",
        )
        odom_stamps = [
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            for message in probe.odom
        ]
        if any(current <= previous for previous, current in zip(odom_stamps, odom_stamps[1:])):
            raise RuntimeError("odometry stamps are not monotonic")
        if any(message.header.frame_id != "pin_slam_map" for message in probe.odom):
            raise RuntimeError("unexpected odometry frame")
        report = {
            "success": True,
            "input_frames": 6,
            "input_points": int(len(points)),
            "status_messages": probe.statuses,
            "odometry_messages": len(probe.odom),
            "neural_point_messages": len(probe.neural),
            "max_neural_points": max(message.width for message in probe.neural),
            "elapsed_wall_seconds": time.monotonic() - started,
        }
        output = os.environ.get("PIN_SLAM_SMOKE_RESULT")
        if output:
            Path(output).write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2), flush=True)
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
