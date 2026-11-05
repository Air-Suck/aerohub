#!/usr/bin/env python3
"""Offline ROS integration: real node, DDS, TF, clocks and serialized messages."""
import math
import os
import struct
import subprocess
import time
import unittest

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


class NavigationInputsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('navigation_input_test')
        cls.clouds, cls.odometry = [], []
        cls.subscriptions = [
            cls.node.create_subscription(PointCloud2, '/drone0/navigation/points',
                                         cls.clouds.append, qos_profile_sensor_data),
            cls.node.create_subscription(Odometry, '/drone0/navigation/odom',
                                         cls.odometry.append, qos_profile_sensor_data)]
        cls.cloud_pub = cls.node.create_publisher(PointCloud2, '/drone0/cloud', 10)
        cls.odom_pub = cls.node.create_publisher(Odometry, '/drone0/odom_input', 10)
        cls.clock_pub = cls.node.create_publisher(Clock, '/clock', 10)
        cls.static_tf = StaticTransformBroadcaster(cls.node)
        cls.dynamic_tf = TransformBroadcaster(cls.node)
        executable = os.environ.get('NAV_INPUT_EXECUTABLE')
        command = [executable] if executable else ['ros2', 'run', 'as2_navigation_inputs', 'navigation_inputs_node']
        cls.process = subprocess.Popen(command + ['--ros-args',
            '-r', '__ns:=/drone0', '-p', 'use_sim_time:=true',
            '-p', 'target_frame:=test/world', '-p', 'body_frame:=test/base',
            '-p', 'cloud_topic:=cloud', '-p', 'odom_topic:=odom_input',
            '-p', 'transform_wait:=0.25'])
        deadline = time.monotonic() + 10
        while cls.cloud_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
        if cls.cloud_pub.get_subscription_count() < 1:
            cls.process.terminate()
            raise RuntimeError('navigation node did not subscribe')
        cls.transforms = [cls.tf('sensor', (10., 20., 30.), math.pi / 2),
                          cls.tf('odom_source', (100., 200., 0.), math.pi / 2),
                          cls.tf('test/base', (1., 2., 3.))]
        cls.static_tf.sendTransform(cls.transforms)

    @classmethod
    def tearDownClass(cls):
        cls.process.terminate()
        try:
            cls.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            cls.process.kill()
            cls.process.wait()
        cls.node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def tf(child, translation, yaw=0., at=1.):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'test/world', child
        tf.header.stamp = stamp(at)
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = translation
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return tf

    def spin(self, duration=0.1):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def clock(self, seconds):
        for _ in range(3):
            self.clock_pub.publish(Clock(clock=stamp(seconds)))
            self.spin(0.02)

    def cloud(self, at, frame='sensor'):
        msg = PointCloud2()
        msg.header.frame_id, msg.header.stamp = frame, stamp(at)
        msg.height, msg.width, msg.point_step, msg.row_step = 1, 1, 16, 16
        msg.fields = [PointField(name=name, offset=i * 4,
                                datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        msg.data = struct.pack('<ffff', 1., 0., 0., 42.)
        return msg

    def test_input_contract(self):
        self.clock(100.)
        self.spin(0.2)
        msg = self.cloud(99.9)
        self.cloud_pub.publish(msg)
        odom = Odometry()
        odom.header.frame_id, odom.header.stamp = 'odom_source', stamp(99.9)
        odom.child_frame_id = 'test/base'
        odom.pose.pose.position.x = 1.
        odom.pose.pose.orientation.w = 1.
        odom.pose.covariance[0], odom.pose.covariance[7] = 1., 4.
        odom.twist.twist.linear.x = 2.
        odom.twist.covariance[0] = 3.
        self.odom_pub.publish(odom)
        self.spin(0.2)
        self.assertEqual(len(self.clouds), 1)
        self.assertEqual(len(self.odometry), 1)
        output = self.clouds[-1]
        self.assertEqual(output.header.frame_id, 'test/world')
        self.assertEqual(output.header.stamp, msg.header.stamp)
        self.assertEqual(output.fields, msg.fields)
        for actual, expected in zip(struct.unpack('<ffff', bytes(output.data)), (10., 21., 30., 42.)):
            self.assertAlmostEqual(actual, expected, places=5)
        state = self.odometry[-1]
        self.assertEqual(state.header.stamp, odom.header.stamp)
        self.assertEqual(state.child_frame_id, 'test/base')
        self.assertAlmostEqual(state.pose.pose.position.x, 100.)
        self.assertAlmostEqual(state.pose.pose.position.y, 201.)
        self.assertAlmostEqual(state.pose.covariance[0], 4.)
        self.assertAlmostEqual(state.pose.covariance[7], 1.)
        self.assertEqual(state.twist, odom.twist)

        # A transform arriving later is used only within the bounded waiting period.
        self.cloud_pub.publish(self.cloud(99.91, 'delayed'))
        self.spin(0.05)
        self.assertEqual(len(self.clouds), 1)
        self.transforms.append(self.tf('delayed', (5., 0., 0.)))
        self.static_tf.sendTransform(self.transforms)
        self.spin(0.15)
        self.assertEqual(len(self.clouds), 2)
        self.assertAlmostEqual(struct.unpack('<ffff', bytes(self.clouds[-1].data))[0], 6.)

        # Missing TF expires even while simulation time is paused.
        self.cloud_pub.publish(self.cloud(99.92, 'missing'))
        self.spin(0.35)
        self.transforms.append(self.tf('missing', (9., 0., 0.)))
        self.static_tf.sendTransform(self.transforms)
        self.spin(0.1)
        self.assertEqual(len(self.clouds), 2)

        # Explicit timestamp interpolation, rather than silently using latest TF.
        self.dynamic_tf.sendTransform(self.tf('moving', (0., 0., 0.), at=100.))
        self.dynamic_tf.sendTransform(self.tf('moving', (2., 0., 0.), at=100.2))
        self.clock(100.3)
        self.cloud_pub.publish(self.cloud(100.1, 'moving'))
        self.spin(0.2)
        self.assertEqual(len(self.clouds), 3)
        self.assertAlmostEqual(struct.unpack('<ffff', bytes(self.clouds[-1].data))[0], 2., places=5)

        # Future input is not executed before ROS time reaches its measurement time.
        self.cloud_pub.publish(self.cloud(100.4))
        self.spin(0.05)
        self.assertEqual(len(self.clouds), 3)
        self.clock(100.4)
        self.spin(0.1)
        self.assertEqual(len(self.clouds), 4)

        # Stale, duplicate, zero-time, malformed and wrong-child inputs are rejected.
        for bad in (self.cloud(98.), self.cloud(100.1), self.cloud(0.)):
            self.cloud_pub.publish(bad)
        self.clock(100.5)
        bad = self.cloud(100.41)
        bad.data = b'bad'
        self.cloud_pub.publish(bad)
        odom.header.stamp = stamp(100.4)
        odom.child_frame_id = 'wrong/base'
        self.odom_pub.publish(odom)
        self.spin(0.2)
        self.assertEqual(len(self.clouds), 4)
        self.assertEqual(len(self.odometry), 1)

        # A simulation reset clears monotonic watermarks; old queued samples do not leak.
        self.cloud_pub.publish(self.cloud(100.6, 'never'))
        self.clock(50.)
        self.static_tf.sendTransform(self.transforms)
        self.spin(0.1)
        self.cloud_pub.publish(self.cloud(49.9))
        self.spin(0.2)
        self.assertEqual(len(self.clouds), 5)
        self.assertEqual(self.clouds[-1].header.stamp, stamp(49.9))
        self.assertIsNone(self.process.poll())


if __name__ == '__main__':
    unittest.main()
