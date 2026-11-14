#!/usr/bin/env python3
"""Run in AS2 image; loads real AS2 host/plugin in an isolated ROS domain."""
import math
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_msgs.msg import TFMessage

rclpy.init()
node = Node('fast_livo2_contract_test', namespace='smoke')
received = {'pose': [], 'twist': [], 'tf': []}
subscriptions = [
    node.create_subscription(PoseStamped, 'self_localization/pose', lambda m: received['pose'].append(m), qos_profile_sensor_data),
    node.create_subscription(TwistStamped, 'self_localization/twist', lambda m: received['twist'].append(m), qos_profile_sensor_data),
    node.create_subscription(TFMessage, '/tf', lambda m: received['tf'].extend(m.transforms), qos_profile_sensor_data)]
pub = node.create_publisher(Odometry, 'fast_livo2/odometry', qos_profile_sensor_data)
proc = subprocess.Popen([
    'ros2', 'run', 'as2_state_estimator', 'as2_state_estimator_node', '--ros-args',
    '-r', '__ns:=/smoke', '-p', 'plugin_name:=fast_livo2', '-p', 'cloud_frame:=smoke/livo_world',
    '-p', 'imu_from_base:=[1.0,0.0,0.0,0.0,0.0,0.0,1.0]',
    '-p', 'earth_from_livo_world:=[10.0,0.0,0.0,0.0,0.0,0.7071067811865476,0.7071067811865476]'])

def spin(duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        assert proc.poll() is None, 'AS2 host exited'
        rclpy.spin_once(node, timeout_sec=0.02)

def message():
    m = Odometry()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = 'camera_init'
    m.child_frame_id = 'fast_livo_imu'
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = 1., 2., 3.
    m.pose.pose.orientation.z = m.pose.pose.orientation.w = math.sqrt(0.5)
    m.twist.twist.linear.x = 1.
    m.twist.twist.angular.z = 2.
    return m

try:
    deadline = time.monotonic() + 15
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        spin(0.1)
    assert pub.get_subscription_count() == 1, 'plugin failed to subscribe'
    for _ in range(10):
        good = message()
        pub.publish(good)
        spin(0.1)
    assert received['pose'] and received['twist'] and received['tf'], 'missing state/TF output'
    pose, twist = received['pose'][-1], received['twist'][-1]
    for actual, expected in zip((pose.pose.position.x,pose.pose.position.y,pose.pose.position.z), (7.,1.,3.)):
        assert abs(actual-expected) < 1e-6, (actual, expected)
    for actual, expected in zip((twist.twist.linear.x,twist.twist.linear.y,twist.twist.angular.z), (1.,2.,2.)):
        assert abs(actual-expected) < 1e-6, (actual, expected)
    assert pose.header.frame_id == 'earth'
    assert twist.header.frame_id == 'smoke/base_link'
    assert pose.header.stamp == good.header.stamp, 'measurement stamp changed'
    assert any(t.header.frame_id == 'smoke/odom' and t.child_frame_id == 'smoke/base_link' for t in received['tf'])
    cases = []
    bad = message(); bad.header.frame_id = 'wrong'; cases.append(bad)
    bad = message(); bad.child_frame_id = 'wrong'; cases.append(bad)
    bad = message(); bad.header.stamp.sec -= 2; cases.append(bad)
    bad = message(); bad.header.stamp.sec += 2; cases.append(bad)
    bad = message(); bad.pose.pose.orientation.w = 0.; cases.append(bad)
    bad = message(); bad.twist.twist.linear.x = float('nan'); cases.append(bad)
    cases.append(good)  # duplicate / out-of-order
    baseline = len(received['pose'])
    for bad in cases:
        pub.publish(bad); spin(0.12)
        assert len(received['pose']) == baseline, 'invalid state accepted'
    spin(0.7)
    assert len(received['pose']) == baseline, 'stale state republished'
    pub.publish(message()); spin(0.2)
    assert len(received['pose']) > baseline, 'valid stream did not recover'
    print('PASS: native plugin load, pose/velocity lever arm, frames, measurement stamps, TF, seven rejection cases, silence and recovery')
finally:
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill(); proc.wait()
    node.destroy_node(); rclpy.shutdown()
