#!/usr/bin/env python3
import argparse
import json
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2


TOPICS = {
    "imu": (Imu, "/drone0/sensor_measurements/imu/data"),
    "image": (Image, "/drone0/sensor_measurements/FrontCamera/rgb/image"),
    "lidar": (PointCloud2, "/drone0/sensor_measurements/Lidar/points"),
    "livox": (CustomMsg, "/fast_livo2/airsim_lidar"),
    "upstream_odometry": (Odometry, "/LIVO2/imu_propagate"),
    "adapted_odometry": (Odometry, "/drone0/fast_livo2/odometry"),
    "upstream_registered_points": (PointCloud2, "/cloud_registered"),
    "adapted_registered_points": (
        PointCloud2, "/drone0/perception/fast_livo2/registered_points"
    ),
    "canonical_pose": (PoseStamped, "/drone0/self_localization/pose"),
    "canonical_twist": (TwistStamped, "/drone0/self_localization/twist"),
}


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Probe(Node):
    def __init__(self):
        super().__init__("fast_livo2_real_stream_probe")
        self.samples = {name: [] for name in TOPICS}
        self.nonempty = {name: 0 for name in TOPICS}
        self._owned_subscriptions = []
        for name, (kind, topic) in TOPICS.items():
            self._owned_subscriptions.append(
                self.create_subscription(
                    kind,
                    topic,
                    lambda message, name=name: self.receive(name, message),
                    qos_profile_sensor_data,
                )
            )

    def receive(self, name, message):
        self.samples[name].append(stamp_ns(message))
        if hasattr(message, "width"):
            self.nonempty[name] += int(message.width > 0 and message.height > 0)
        elif hasattr(message, "point_num"):
            self.nonempty[name] += int(message.point_num > 0)
        else:
            self.nonempty[name] += 1

    def report(self):
        report = {}
        for name, stamps in self.samples.items():
            ordered = sorted(set(stamps))
            gaps = [right - left for left, right in zip(ordered, ordered[1:])]
            report[name] = {
                "messages": len(stamps),
                "unique_source_stamps": len(ordered),
                "nonempty_messages": self.nonempty[name],
                "non_monotonic_arrivals": sum(
                    right <= left for left, right in zip(stamps, stamps[1:])
                ),
                "maximum_source_gap_s": max(gaps) * 1.0e-9 if gaps else None,
            }
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output")
    parser.add_argument("--require-output", action="store_true")
    parser.add_argument("--require-plugin", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report()
        minimums = {
            "imu": 20,
            "image": 2,
            "lidar": 3,
            "livox": 3,
        }
        if args.require_output:
            minimums.update(
                upstream_odometry=1,
                adapted_odometry=1,
                upstream_registered_points=1,
                adapted_registered_points=1,
            )
        if args.require_plugin:
            minimums.update(canonical_pose=20, canonical_twist=20)
        monotonic_required = {
            "imu",
            "image",
            "lidar",
            "livox",
            "upstream_odometry",
            "adapted_odometry",
            "adapted_registered_points",
            "canonical_pose",
            "canonical_twist",
        }
        failures = []
        for name, minimum in minimums.items():
            sample = report[name]
            if sample["unique_source_stamps"] < minimum:
                failures.append(
                    f"{name}: {sample['unique_source_stamps']} unique stamps < {minimum}"
                )
            if sample["nonempty_messages"] < minimum:
                failures.append(
                    f"{name}: {sample['nonempty_messages']} nonempty messages < {minimum}"
                )
            if name in monotonic_required and sample["non_monotonic_arrivals"]:
                failures.append(
                    f"{name}: {sample['non_monotonic_arrivals']} non-monotonic arrivals"
                )
        document = {"duration_s": args.duration, "streams": report, "failures": failures}
        text = json.dumps(document, indent=2) + "\n"
        print(text, end="")
        if args.output:
            with open(args.output, "w") as stream:
                stream.write(text)
        return 1 if failures else 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
