#!/usr/bin/env python3
"""Record canonical pose freshness in a dedicated lightweight ROS process."""

import argparse
import json
import os
import signal
import time
from pathlib import Path

import rclpy
from as2_msgs.msg import PlatformInfo, PlatformStatus
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def maximum_gap_s(stamps):
    ordered = sorted(set(stamps))
    return max(
        ((right - left) * 1.0e-9 for left, right in zip(ordered, ordered[1:])),
        default=None,
    )


def rate_hz(stamps):
    ordered = sorted(set(stamps))
    if len(ordered) < 2:
        return 0.0
    duration = (ordered[-1] - ordered[0]) * 1.0e-9
    return (len(ordered) - 1) / duration if duration > 0.0 else 0.0


class FreshnessProbe(Node):
    def __init__(self, namespace, output):
        super().__init__("state_estimation_canonical_freshness")
        self.output = output
        self.samples = []
        self.windows = []
        self.active_start = None
        self.seen_platform = False
        self.last_write = 0.0
        prefix = "/" + namespace.strip("/")
        self.create_subscription(
            PoseStamped, prefix + "/self_localization/pose",
            self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(
            PlatformInfo, prefix + "/platform/info",
            self.platform_callback, qos_profile_sensor_data)
        self.create_timer(1.0, self.write)

    def pose_callback(self, message):
        if self.active_start is None:
            return
        self.samples.append({
            "source_stamp_ns": stamp_ns(message),
            "arrival_monotonic_ns": time.monotonic_ns(),
        })

    def platform_callback(self, message):
        self.seen_platform = True
        flying = message.status.state == PlatformStatus.FLYING
        if flying and self.active_start is None:
            self.active_start = len(self.samples)
        elif not flying and self.active_start is not None:
            self.windows.append((self.active_start, len(self.samples)))
            self.active_start = None
            self.write()

    def selected_samples(self):
        windows = list(self.windows)
        if self.active_start is not None:
            windows.append((self.active_start, len(self.samples)))
        return [
            sample
            for start, end in windows
            for sample in self.samples[start:end]
        ]

    def report(self):
        selected = self.selected_samples()
        source = [sample["source_stamp_ns"] for sample in selected]
        arrival = [sample["arrival_monotonic_ns"] for sample in selected]
        non_monotonic = sum(
            right <= left for left, right in zip(source, source[1:]))
        return {
            "platform_status_observed": self.seen_platform,
            "canonical_airborne_windows": len(self.windows),
            "canonical_window_open": self.active_start is not None,
            "canonical_pose_samples": len(selected),
            "canonical_pose_unique_source_stamps": len(set(source)),
            "canonical_pose_non_monotonic_count": non_monotonic,
            "canonical_pose_rate_hz": rate_hz(source),
            "canonical_pose_arrival_rate_hz": rate_hz(arrival),
            "canonical_pose_source_maximum_gap_s": maximum_gap_s(source),
            "canonical_pose_arrival_maximum_gap_s": maximum_gap_s(arrival),
        }

    def write(self):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output.with_suffix(self.output.suffix + ".tmp")
        temporary.write_text(json.dumps(self.report(), indent=2) + "\n")
        os.replace(temporary, self.output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="drone0")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = FreshnessProbe(args.namespace, args.output)
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.write()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
