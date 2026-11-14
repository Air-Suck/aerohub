#!/usr/bin/env python3
"""Loopback-only ROS1 -> ROS2 output bridge; no inbound flight commands."""
import base64
import json
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from rosgraph_msgs.msg import Clock

MAX_BYTES = 32 * 1024 * 1024

def read_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        block = sock.recv(size - len(data))
        if not block:
            raise ConnectionError('stream closed')
        data.extend(block)
    return bytes(data)

def stamp(target, value):
    sec, nsec = value
    if not isinstance(sec, int) or not isinstance(nsec, int) or not 0 <= nsec < 1000000000:
        raise ValueError('invalid timestamp')
    target.sec, target.nanosec = sec, nsec

class Bridge(Node):
    def __init__(self):
        super().__init__('fast_livo2_output_bridge')
        self.port = self.declare_parameter('port', 18761).value
        self.cloud_frame = self.declare_parameter('cloud_frame', 'drone0/livo_world').value
        self.forward_clock = self.declare_parameter('forward_clock', False).value
        self.stop = threading.Event()
        self.odom = self.create_publisher(Odometry, 'fast_livo2/odometry', qos_profile_sensor_data)
        self.cloud = self.create_publisher(PointCloud2, 'perception/fast_livo2/registered_points', qos_profile_sensor_data)
        self.clock = self.create_publisher(Clock, '/clock', 10) if self.forward_clock else None
        self.threads = []
        for offset, kind in enumerate(('odom', 'cloud', 'clock')):
            if kind == 'clock' and not self.forward_clock:
                continue
            thread = threading.Thread(target=self.receive, args=(self.port + offset, kind), daemon=True)
            thread.start()
            self.threads.append(thread)

    def receive(self, port, kind):
        last_warning = 0
        while not self.stop.is_set():
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=2) as sock:
                    while not self.stop.is_set():
                        size = struct.unpack('!I', read_exact(sock, 4))[0]
                        if not 0 < size <= MAX_BYTES:
                            raise ValueError('invalid packet size')
                        obj = json.loads(read_exact(sock, size))
                        if obj.get('version') != 1 or obj.get('kind') != kind:
                            raise ValueError('protocol mismatch')
                        self.publish_packet(obj)
            except (OSError, ValueError, KeyError, TypeError, AssertionError) as exc:
                if time.monotonic() - last_warning > 5:
                    self.get_logger().warning(f'{kind} reconnect: {exc}')
                    last_warning = time.monotonic()
                self.stop.wait(0.5)

    def publish_packet(self, obj):
        kind = obj['kind']
        if kind == 'clock':
            msg = Clock()
            stamp(msg.clock, obj['stamp'])
            self.clock.publish(msg)
            return
        if kind == 'odom':
            msg = Odometry()
            stamp(msg.header.stamp, obj['stamp'])
            msg.header.frame_id = obj['frame']
            msg.child_frame_id = obj['child']
            p, q, v, w = obj['position'], obj['orientation'], obj['linear'], obj['angular']
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, p)
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = map(float, q)
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, v)
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, w)
            msg.pose.covariance = list(map(float, obj['pose_covariance']))
            msg.twist.covariance = list(map(float, obj['twist_covariance']))
            self.odom.publish(msg)
        elif kind == 'cloud':
            if obj['frame'] != 'camera_init':
                raise ValueError('cloud must be in camera_init world coordinates')
            # Upstream emits empty clouds during LIO/VIO startup; these are not maps.
            if obj['width'] == 0 or obj['height'] == 0:
                return
            if not {'x', 'y', 'z'}.issubset({f[0] for f in obj['fields']}):
                raise ValueError('registered points lack XYZ fields')
            msg = PointCloud2()
            stamp(msg.header.stamp, obj['stamp'])
            # Identity alias only: the AS2 plugin publishes the corresponding static TF.
            msg.header.frame_id = self.cloud_frame
            for attr in ('height', 'width', 'is_bigendian', 'point_step', 'row_step', 'is_dense'):
                setattr(msg, attr, obj[attr])
            msg.fields = [PointField(name=f[0], offset=f[1], datatype=f[2], count=f[3]) for f in obj['fields']]
            msg.data = base64.b64decode(obj['data'], validate=True)
            if len(msg.data) != msg.height * msg.row_step or msg.row_step < msg.width * msg.point_step:
                raise ValueError('inconsistent cloud layout')
            self.cloud.publish(msg)

def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        for thread in node.threads:
            thread.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
