#!/usr/bin/env python3
import os
import struct
import subprocess
import time
import unittest

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from uav_rog_map_interfaces.msg import MapStatus
from uav_rog_map_interfaces.srv import CheckLine, QueryVoxel


def stamp(seconds):
    value = rclpy.time.Time(seconds=seconds).to_msg()
    return value


class Harness:
    def __init__(self, test):
        self.test = test
        self.node = rclpy.create_node("rog_map_contract_test")
        self.clock_pub = self.node.create_publisher(
            Clock, "/clock", QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE))
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, "/test/points", qos_profile_sensor_data)
        self.status = None
        self.occupied = None
        self.inflated = None
        self.esdf = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(MapStatus, "/test/map/status", self._status, qos)
        self.node.create_subscription(PointCloud2, "/test/map/occupied", self._occupied, qos)
        self.node.create_subscription(
            PointCloud2, "/test/map/inflated_occupied", self._inflated, qos)
        self.node.create_subscription(PointCloud2, "/test/map/esdf_slice", self._esdf, qos)
        self.query = self.node.create_client(QueryVoxel, "/test/map/query_voxel")
        self.line = self.node.create_client(CheckLine, "/test/map/check_line")
        self.tf = StaticTransformBroadcaster(self.node)
        transform = TransformStamped()
        transform.header.frame_id = "test/world"
        transform.child_frame_id = "test/lidar"
        transform.transform.translation.z = 1.0
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)
        self.sim_time = 100.0
        self.process = None

    def _status(self, message):
        self.status = message

    def _occupied(self, message):
        self.occupied = message

    def _inflated(self, message):
        self.inflated = message

    def _esdf(self, message):
        self.esdf = message

    def tick(self, duration=0.02, clock=None):
        if clock is not None:
            self.sim_time = clock
        end = time.monotonic() + duration
        while time.monotonic() < end:
            message = Clock()
            message.clock = stamp(self.sim_time)
            self.clock_pub.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def wait(self, predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.tick()
            if predicate():
                return
        output = ""
        if self.process and self.process.poll() is not None:
            output = self.process.stdout.read()
        self.test.fail(f"condition timed out; process output:\n{output}")

    def start(self):
        executable = os.environ["ROG_MAP_EXECUTABLE"]
        config = os.environ["ROG_MAP_TEST_CONFIG"]
        self.process = subprocess.Popen([
            executable, "--ros-args", "-r", "__ns:=/test",
            "-p", "use_sim_time:=true", "-p", f"config_path:={config}",
            "-p", "global_frame:=test/world", "-p", "sensor_frame:=test/lidar",
            "-p", "cloud_topic:=/test/points", "-p", "max_input_age:=1.0",
            "-p", "transform_wait:=0.25", "-p", "stale_timeout:=1.0",
            "-p", "publish_rate:=10.0", "-p", "publication_range:=[6.0,6.0,3.0]",
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.wait(lambda: self.query.service_is_ready() and self.line.service_is_ready())
        self.wait(lambda: self.status is not None)

    def stop(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.process = None

    def cloud(self, at, valid=True, overlapping_xyz=False):
        message = PointCloud2()
        message.header.frame_id = "test/world"
        message.header.stamp = stamp(at) if valid else stamp(0.0)
        message.height = 1
        message.width = 3
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="y", offset=0 if overlapping_xyz else 4,
                datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = b"".join(struct.pack("<fff", *point) for point in (
            (2.0, 0.0, 1.0), (2.0, 0.05, 1.0), (2.0, -0.05, 1.0)))
        message.is_dense = True
        self.cloud_pub.publish(message)

    def call(self, client, request):
        future = client.call_async(request)
        self.wait(future.done)
        return future.result()

    def close(self):
        self.stop()
        self.node.destroy_node()


class ROGMapContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_real_core_queries_staleness_clock_reset_and_restart(self):
        harness = Harness(self)
        try:
            harness.start()
            self.assertFalse(harness.status.ready)
            self.assertEqual(harness.status.accepted_clouds, 0)
            harness.cloud(99.7, valid=False)
            harness.cloud(99.8, overlapping_xyz=True)
            harness.cloud(98.0)
            harness.cloud(102.0)
            for offset in (0.0, 0.02, 0.04):
                harness.cloud(100.0 + offset)
                harness.tick(0.08, 100.0 + offset)
            harness.cloud(100.04)
            harness.wait(lambda: harness.status.ready and harness.status.accepted_clouds >= 3)
            harness.wait(lambda: all(
                cloud is not None and cloud.width > 0
                for cloud in (harness.occupied, harness.inflated, harness.esdf)))
            harness.tick(0.35, 100.1)
            harness.wait(lambda: harness.status.rejected_malformed >= 2)
            harness.wait(lambda: harness.status.rejected_time >= 2)

            query = QueryVoxel.Request()
            query.point = Point(x=2.0, y=0.0, z=1.0)
            result = harness.call(harness.query, query)
            self.assertTrue(result.ready)
            self.assertEqual(result.occupancy, QueryVoxel.Response.OCCUPIED)
            self.assertEqual(result.inflated_occupancy, QueryVoxel.Response.OCCUPIED)
            self.assertTrue(result.has_esdf)
            self.assertEqual(result.frame_id, "test/world")

            line = CheckLine.Request()
            line.start = Point(x=0.0, y=0.0, z=1.0)
            line.end = Point(x=3.0, y=0.0, z=1.0)
            line.use_inflated = True
            line.unknown_as_occupied = False
            checked = harness.call(harness.line, line)
            self.assertTrue(checked.ready)
            self.assertFalse(checked.clear)

            harness.tick(0.2, 101.5)
            harness.wait(lambda: not harness.status.ready)
            self.assertFalse(harness.call(harness.query, query).ready)
            harness.cloud(101.5)
            harness.wait(lambda: harness.status.ready)
            harness.tick(0.1, 50.0)
            harness.wait(lambda: not harness.status.epoch_valid)
            self.assertFalse(harness.call(harness.query, query).ready)

            harness.stop()
            harness.status = harness.occupied = harness.inflated = harness.esdf = None
            harness.sim_time = 200.0
            harness.start()
            self.assertFalse(harness.status.ready)
            self.assertEqual(harness.status.accepted_clouds, 0)
            harness.cloud(200.0)
            harness.tick(0.08, 200.0)
            harness.cloud(200.02)
            harness.tick(0.08, 200.02)
            harness.wait(lambda: harness.status.ready and harness.status.accepted_clouds >= 2)
        finally:
            harness.close()


if __name__ == "__main__":
    unittest.main()
