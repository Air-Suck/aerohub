#!/usr/bin/env python3
"""Record actual ROS sensor delivery and sample timestamps; never command motion."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       DurabilityPolicy, ReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topics", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=10)
    args = parser.parse_args()
    specification = json.loads(args.topics.read_text())
    rclpy.init()
    node = Node("simulator_sensor_probe")
    records = {}
    clock = [0]
    subscriptions = []

    def receive(name, msg):
        record = records[name]
        record["count"] += 1
        if name == "clock":
            stamp = msg.clock
        elif hasattr(msg, "header"):
            stamp = msg.header.stamp
            record["frame_id"] = msg.header.frame_id
        else:
            return
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        if stamp_ns > 0:
            if record["last_stamp_ns"] and stamp_ns < record["last_stamp_ns"]:
                record["backwards"] += 1
            record["stamps"].add(stamp_ns)
            record["last_stamp_ns"] = stamp_ns
        if name == "clock":
            clock[0] = stamp_ns
        if hasattr(msg, "data"):
            record["payload_bytes"] = len(msg.data)
            record["nonempty_samples"] += int(bool(msg.data))
            record["empty_samples"] += int(not msg.data)
        if hasattr(msg, "k"):
            record["intrinsics"] = [float(v) for v in msg.k]
            record["projection"] = [float(v) for v in msg.p]
            record["calibration_valid"] = bool(msg.width > 0 and msg.height > 0
                and all(math.isfinite(v) for v in (*msg.k, *msg.p))
                and msg.k[0] > 0 and msg.k[4] > 0 and msg.k[8] == 1)
        if hasattr(msg, "width"):
            record["width"] = msg.width
            record["height"] = msg.height
        if hasattr(msg, "encoding"):
            record["encoding"] = msg.encoding

    for name, spec in specification.items():
        records[name] = {"topic": spec["topic"], "count": 0, "stamps": set(),
                         "last_stamp_ns": 0, "backwards": 0,
                         "nonempty_samples": 0, "empty_samples": 0}
        qos = qos_profile_sensor_data
        if spec.get("calibration"):
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        subscriptions.append(node.create_subscription(
            get_message(spec["type"]), spec["topic"],
            lambda msg, name=name: receive(name, msg), qos))
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        for name, record in records.items():
            record["unique_stamps"] = len(record.pop("stamps"))
            record["age_sim_s"] = (clock[0] - record["last_stamp_ns"]) / 1e9
            calibration = specification[name].get("calibration", False)
            record["passed"] = record["count"] > 0 and (
                calibration or (record["unique_stamps"] >= 2 and
                                -0.1 <= record["age_sim_s"] <= 1.0 and
                                record["backwards"] == 0))
            if "payload_bytes" in record:
                # Ray-cast LiDAR may have no returns in individual scans. It
                # must still provide actual returns in this acceptance window.
                if specification[name].get("allow_empty_frames", False):
                    record["passed"] &= record["nonempty_samples"] > 0
                else:
                    record["passed"] &= record["empty_samples"] == 0
            if calibration:
                record["passed"] &= record.get("calibration_valid", False)
                if specification[name].get("projection_from_k") and record["count"]:
                    k = record["intrinsics"]
                    expected = [*k[:3], 0.0, *k[3:6], 0.0, *k[6:9], 0.0]
                    record["passed"] &= record["projection"] == expected
            spec = specification[name]
            if "frame" in spec:
                record["passed"] &= record.get("frame_id") == spec["frame"]
            if "encoding" in spec:
                record["passed"] &= record.get("encoding") == spec["encoding"]
        clock_publishers = len(node.get_publishers_info_by_topic("/clock"))
        result = {"passed": all(r["passed"] for r in records.values()),
                  "scope": "ROS delivery and timestamps; coordinate semantics require separate validation",
                  "wall_seconds": args.seconds, "topics": records}
        result["clock_publishers"] = clock_publishers
        result["passed"] &= clock_publishers == 1
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return 0 if result["passed"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
