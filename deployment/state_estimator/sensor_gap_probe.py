#!/usr/bin/env python3
"""Measure source-stamp and arrival gaps for one PointCloud2 stream."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def maximum_gap_s(stamps):
    ordered = sorted(set(stamps))
    return max(
        ((right - left) * 1.0e-9 for left, right in zip(ordered, ordered[1:])),
        default=None,
    )


class GapProbe(Node):
    def __init__(self, topic):
        super().__init__("state_estimation_sensor_gap_probe")
        self.source_stamps = []
        self.arrival_stamps = []
        self.create_subscription(
            PointCloud2, topic, self.callback, qos_profile_sensor_data)

    def callback(self, message):
        self.source_stamps.append(
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec)
        self.arrival_stamps.append(time.monotonic_ns())

    def report(self):
        return {
            "messages": len(self.source_stamps),
            "unique_source_stamps": len(set(self.source_stamps)),
            "non_monotonic_source_stamps": sum(
                right <= left for left, right in zip(
                    self.source_stamps, self.source_stamps[1:])),
            "source_maximum_gap_s": maximum_gap_s(self.source_stamps),
            "arrival_maximum_gap_s": maximum_gap_s(self.arrival_stamps),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")

    rclpy.init()
    node = GapProbe(args.topic)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        report = json.dumps(node.report(), indent=2) + "\n"
        if args.output:
            with open(args.output, "w") as stream:
                stream.write(report)
        print(report, end="")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
