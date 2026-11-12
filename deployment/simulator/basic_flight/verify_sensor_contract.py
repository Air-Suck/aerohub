#!/usr/bin/env python3
"""Compare actual native and normalized ROS samples and configured static TF."""
import argparse
from collections import OrderedDict
import json
import hashlib
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=10)
    args = parser.parse_args()
    inventory = json.loads(args.inventory.read_text())
    rclpy.init()
    node = Node("verify_sensor_contract")
    records, pending, transforms, subscriptions = {}, {}, {}, []
    for index, stream in enumerate(inventory):
        records[index] = {"source": stream["source"], "target": stream["target"],
                          "paired": 0, "errors": [], "nonempty_pairs": 0,
                          "source_count": 0, "target_count": 0}
        pending[index] = (OrderedDict(), OrderedDict())

    def receive(index, side, msg):
        stream, record = inventory[index], records[index]
        key = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if key <= 0:
            return
        record["source_count" if side == 0 else "target_count"] += 1
        if isinstance(msg, PointCloud2):
            # Native LiDAR can publish distinct partial/empty scans with one
            # timestamp. Stamp alone does not identify the same measurement.
            key = (key, hashlib.blake2b(bytes(msg.data), digest_size=16).digest())
        cache, other = pending[index][side], pending[index][1-side]
        cache[key] = msg
        while len(cache) > 64:
            cache.popitem(last=False)
        if key not in other:
            return
        raw, output = (msg, other.pop(key)) if side == 0 else (other.pop(key), msg)
        cache.pop(key, None)
        record["paired"] += 1
        try:
            assert output.header.frame_id == stream["frame"], "frame mismatch"
            if isinstance(raw, Imu):
                assert output.orientation_covariance[0] == -1, "orientation advertised as available"
                assert raw.angular_velocity == output.angular_velocity, "gyro changed"
                assert raw.linear_acceleration == output.linear_acceleration, "accel changed"
            else:
                record["nonempty_pairs"] += int(bool(output.data))
                assert (raw.width, raw.height, raw.is_bigendian) == (
                    output.width, output.height, output.is_bigendian), "layout changed"
                if isinstance(raw, Image) and raw.encoding == "bgr8":
                    assert output.encoding == "rgb8", "RGB encoding"
                    a = np.frombuffer(bytes(raw.data), dtype=np.uint8).reshape(raw.height, raw.step)
                    b = np.frombuffer(bytes(output.data), dtype=np.uint8).reshape(output.height, output.step)
                    assert np.array_equal(a[:, :raw.width*3].reshape(raw.height, raw.width, 3)[:, :, ::-1],
                                          b[:, :raw.width*3].reshape(raw.height, raw.width, 3)), "RGB pixels"
                else:
                    assert raw.data == output.data, "measurement payload changed"
                if isinstance(raw, Image) and raw.encoding == "32FC1":
                    dtype = ">f4" if raw.is_bigendian else "<f4"
                    values = np.frombuffer(bytes(output.data), dtype=dtype)
                    valid = values[np.isfinite(values) & (values > 0)]
                    record["depth_valid_pixels"] = int(valid.size)
                    if valid.size:
                        record["depth_range_m"] = [float(valid.min()), float(valid.max())]
        except AssertionError as exc:
            if len(record["errors"]) < 10:
                record["errors"].append(str(exc))

    for index, stream in enumerate(inventory):
        msg_type = {"image": Image, "imu": Imu, "lidar": PointCloud2}[stream["kind"]]
        for side, topic in enumerate((stream["source"], stream["target"])):
            subscriptions.append(node.create_subscription(msg_type, topic,
                lambda msg, index=index, side=side: receive(index, side, msg), qos_profile_sensor_data))

    def tf_receive(msg):
        for tf in msg.transforms:
            transforms[tf.child_frame_id] = tf
    subscriptions.append(node.create_subscription(TFMessage, "/tf_static", tf_receive,
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE)))
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        for index, stream in enumerate(inventory):
            record = records[index]
            tf = transforms.get(stream["frame"])
            expected = [*stream["transform"][0], *stream["transform"][1]]
            actual = [] if tf is None else [tf.transform.translation.x, tf.transform.translation.y,
                tf.transform.translation.z, tf.transform.rotation.x, tf.transform.rotation.y,
                tf.transform.rotation.z, tf.transform.rotation.w]
            record["tf_valid"] = bool(tf and tf.header.frame_id == stream["parent"]
                and all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(actual, expected)))
            record["passed"] = record["paired"] >= 5 and not record["errors"] and record["tf_valid"]
            record["paired_fraction"] = record["paired"] / max(1, min(
                record["source_count"], record["target_count"]))
            record["passed"] &= record["paired_fraction"] >= 0.95
            if stream["kind"] != "imu":
                record["passed"] &= record["nonempty_pairs"] > 0
        result = {"passed": all(r["passed"] for r in records.values()),
                  "scope": "real ROS sample preservation, configured static TF and RGB conversion; not downstream algorithm acceptance",
                  "streams": list(records.values())}
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return 0 if result["passed"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
