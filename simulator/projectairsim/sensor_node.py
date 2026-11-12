#!/usr/bin/env python3
"""Normalize upstream ROS messages; SDK access remains in the native bridge."""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from sensor_contract import streams

CALIBRATION_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SensorAdapter(Node):
    def __init__(self, inventory):
        super().__init__("projectairsim_sensor_adapter")
        self.subscriptions_owned = []
        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = {}
        for stream in inventory:
            frame = stream["frame"]
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = stream["parent"], frame
            xyz, quat = stream["transform"]
            (transform.transform.translation.x, transform.transform.translation.y,
             transform.transform.translation.z) = xyz
            (transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w) = quat
            transforms[frame] = transform
            msg_type = {"image": Image, "imu": Imu, "lidar": PointCloud2}[stream["kind"]]
            publisher = self.create_publisher(msg_type, stream["target"], qos_profile_sensor_data)
            self.subscribe(msg_type, stream["source"], publisher, stream)
            if stream["kind"] == "image":
                target = stream["target"].removesuffix("/image") + "/camera_info"
                publisher = self.create_publisher(CameraInfo, target, CALIBRATION_QOS)
                self.subscribe(CameraInfo, stream["source"] + "/camera_info", publisher, stream,
                               calibration=True)
        self.broadcaster.sendTransform(list(transforms.values()))

    def subscribe(self, msg_type, source, publisher, stream, calibration=False):
        def receive(message):
            if not calibration and message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
                return
            msg = deepcopy(message)
            # Native camera pixels are already optical; native IMU/cloud vectors
            # already have FRD->FLU conversion. Attach the corresponding real TF.
            msg.header.frame_id = stream["frame"]
            if isinstance(msg, Imu):
                # Native orientation is in a different world convention. Do not
                # advertise it as an ENU estimate or manufacture covariance.
                msg.orientation.x = msg.orientation.y = msg.orientation.z = 0.0
                msg.orientation.w = 1.0
                msg.orientation_covariance[0] = -1.0
            elif isinstance(msg, CameraInfo):
                capture = stream["capture"]
                focal = capture["width"] / (2 * math.tan(math.radians(capture["fov-degrees"]) / 2))
                if (msg.width != capture["width"] or msg.height != capture["height"]
                    or not math.isclose(msg.k[0], focal, rel_tol=1e-5)
                    or not math.isclose(msg.k[4], focal, rel_tol=1e-5)
                    or any(msg.d) or list(msg.r) != [1, 0, 0, 0, 1, 0, 0, 0, 1]):
                    raise ValueError("native calibration differs from supported pinhole configuration")
                # Native camera.cpp emits identity P. For this verified zero-
                # distortion, unrectified monocular model P is [K | 0].
                msg.p = [float(msg.k[0]), float(msg.k[1]), float(msg.k[2]), 0.0,
                         float(msg.k[3]), float(msg.k[4]), float(msg.k[5]), 0.0,
                         float(msg.k[6]), float(msg.k[7]), float(msg.k[8]), 0.0]
                msg.distortion_model = "plumb_bob"
            elif isinstance(msg, Image) and msg.encoding == "bgr8":
                # Match the existing AirSim rgb8 contract, preserving row padding.
                data = bytearray(msg.data)
                for row in range(msg.height):
                    start, end = row * msg.step, row * msg.step + msg.width * 3
                    data[start:end:3], data[start+2:end:3] = data[start+2:end:3], data[start:end:3]
                msg.data, msg.encoding = bytes(data), "rgb8"
            publisher.publish(msg)
        self.subscriptions_owned.append(self.create_subscription(
            msg_type, source, receive, CALIBRATION_QOS if calibration else qos_profile_sensor_data))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--topic-root", default="/ProjectAirsim")
    args = parser.parse_args()
    inventory = streams(json.loads(args.scene.read_text()), json.loads(args.runtime.read_text()),
                        args.topic_root)
    rclpy.init()
    node = SensorAdapter(inventory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
