#!/usr/bin/env python3
"""Synthetic moving-platform input check; never arms or publishes flight commands.

Source both installs. Pass --navigation-executable and --fsm-executable.
"""
import argparse
import math
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--navigation-executable', required=True)
    parser.add_argument('--fsm-executable', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('super_as2_inputs_test')
    clock = node.create_publisher(Clock, '/clock', 10)
    odometry = node.create_publisher(Odometry, '/drone0/sensor_measurements/odom', 10)
    clouds = node.create_publisher(PointCloud2, '/drone0/sensor_measurements/Lidar/points', 10)
    direct_clouds = node.create_publisher(PointCloud2, '/drone0/navigation/points', 10)
    maps = []
    subscription = node.create_subscription(PointCloud2, '/rog_map/occ', maps.append,
                                            qos_profile_sensor_data)
    dynamic_tf, static_tf = TransformBroadcaster(node), StaticTransformBroadcaster(node)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)

    def set_clock(seconds):
        clock.publish(Clock(clock=stamp(seconds)))
        spin(0.03)

    def cloud(at, xyz, frame):
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = stamp(at), frame
        msg.height, msg.width, msg.point_step, msg.row_step = 1, 1, 12, 12
        msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
        msg.data = struct.pack('<fff', *xyz)
        return msg

    def points(msg):
        offsets = {f.name: f.offset for f in msg.fields}
        return [tuple(struct.unpack_from('<f', bytes(msg.data), i * msg.point_step + offsets[k])[0]
                      for k in ('x', 'y', 'z')) for i in range(msg.width * msg.height)]

    processes = []
    with tempfile.TemporaryDirectory(prefix='super-as2-inputs-') as temporary:
        config = yaml.safe_load((Path(__file__).parents[1] / 'config/as2_inputs_ros2.yaml').read_text())
        config['rog_map']['map_size'] = [10, 10, 6]
        config_path = Path(temporary) / 'inputs.yaml'
        config_path.write_text(yaml.safe_dump(config))
        log = open(Path(temporary) / 'nodes.log', 'w+')
        try:
            processes.append(subprocess.Popen([args.navigation_executable, '--ros-args',
                '-r', '__ns:=/drone0', '-p', 'use_sim_time:=true'], stdout=log, stderr=log))
            processes.append(subprocess.Popen([args.fsm_executable, '--ros-args',
                '-p', 'use_sim_time:=true', '-p', 'config_name:=' + str(config_path)],
                stdout=log, stderr=log))
            until = time.monotonic() + 15
            while (clouds.get_subscription_count() < 1 or direct_clouds.get_subscription_count() < 1):
                if time.monotonic() > until or any(p.poll() is not None for p in processes):
                    raise AssertionError('Input nodes failed to start')
                set_clock(100.)
            extrinsic = TransformStamped()
            extrinsic.header.frame_id, extrinsic.child_frame_id = 'drone0/base_link', 'drone0/Lidar'
            extrinsic.header.stamp = stamp(100.)
            extrinsic.transform.translation.x = 0.2
            extrinsic.transform.rotation.w = 1.
            static_tf.sendTransform(extrinsic)
            spin(0.2)
            # The aircraft translates and turns; the obstacle stays at (3, 0, 1).
            for i in range(12):
                at, x, yaw = 100. + i * 0.05, i * 0.08, i * 0.05
                set_clock(at)
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = 'drone0/odom', 'drone0/base_link'
                tf.header.stamp = stamp(at)
                tf.transform.translation.x, tf.transform.translation.z = x, 1.
                tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                dynamic_tf.sendTransform(tf)
                odom = Odometry()
                odom.header = tf.header
                odom.child_frame_id = tf.child_frame_id
                odom.pose.pose.position.x, odom.pose.pose.position.z = x, 1.
                odom.pose.pose.orientation = tf.transform.rotation
                odometry.publish(odom)
                # Inverse world->body, then subtract lidar mounting translation.
                local = (math.cos(yaw) * (3 - x) - 0.2, -math.sin(yaw) * (3 - x), 0.)
                clouds.publish(cloud(at, local, 'drone0/Lidar'))
                spin(0.08)
            spin(0.6)
            assert maps, 'ROG-Map did not publish an occupied cloud'
            occupied = points(maps[-1])
            assert maps[-1].header.frame_id == 'drone0/odom'
            assert occupied, 'ROG-Map received no XYZ obstacle'
            assert all(math.dist(p, (3., 0., 1.)) < 0.2 for p in occupied), occupied
            # Wrong frames and a cloud with no historical pose must not add an obstacle.
            set_clock(100.8)
            direct_clouds.publish(cloud(100.7, (0., 3., 1.), 'wrong/frame'))
            direct_clouds.publish(cloud(100.75, (0., 3., 1.), 'drone0/odom'))
            spin(0.6)
            assert all(math.dist(p, (3., 0., 1.)) < 0.2 for p in points(maps[-1]))
            assert all(p.poll() is None for p in processes)
            print('PASS: moving-platform XYZ -> navigation inputs -> time-matched ROG-Map; invalid clouds rejected')
        except BaseException:
            log.flush()
            log.seek(0)
            print(log.read()[-12000:])
            raise
        finally:
            for process in reversed(processes):
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            log.close()
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
