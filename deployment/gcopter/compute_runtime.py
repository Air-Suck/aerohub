"""Compute lifecycle for live local planning and the retained snapshot baseline."""
import json
from pathlib import Path
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Runtime(Node):
    def __init__(self):
        super().__init__('gcopter_compute_runtime', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.out = Path('/evidence')
        self.odom = self.cloud = None
        self.children = []
        self.target = None
        self.stopping = False
        self.local = os.environ.get('GCOPTER_CASE') in ('local', 'forest')
        self.create_subscription(Odometry, '/drone0/sensor_measurements/odom', lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/drone0/navigation/points', lambda m: setattr(self, 'cloud', m), qos_profile_sensor_data)
        self.map_pub = self.create_publisher(PointCloud2, '/drone0/gcopter/map', 1)
        self.state_pub = self.create_publisher(String, '/drone0/gcopter/runtime_state', 10)
        self.create_service(Trigger, '/drone0/gcopter/snapshot', self.snapshot)
        self.create_timer(.1, self.tick)

    def start(self, name, command):
        stream = (self.out/(name+'.log')).open('w')
        child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        self.children.append((name, child, stream))

    def snapshot(self, request, response):
        if not self.cloud or not self.odom or self.cloud.header.frame_id != 'drone0/odom':
            response.message = 'registered input unavailable'
            return response
        if self.local:
            (self.out/'input-lidar.cdr').write_bytes(serialize_message(self.cloud))
            (self.out/'input-state.cdr').write_bytes(serialize_message(self.odom))
            response.success = True
            response.message = 'local planner consumes live registered LiDAR directly'
            return response
        age = (self.get_clock().now()-rclpy.time.Time.from_msg(self.cloud.header.stamp)).nanoseconds/1e9
        if not 0 <= age <= 1 or self.map_pub.get_subscription_count() < 2:
            response.message = 'input stale or consumers undiscovered'
            return response
        cloud = self.cloud
        self.map_pub.publish(cloud)
        (self.out/'input-map.cdr').write_bytes(serialize_message(cloud))
        (self.out/'input-state.cdr').write_bytes(serialize_message(self.odom))
        response.success = True
        response.message = 'snapshot published; wait for Function health'
        return response

    def tick(self):
        for name, child, _ in self.children:
            if child.poll() is not None:
                raise RuntimeError(name+' exited '+str(child.returncode))
        if self.target is None and self.odom and self.cloud:
            p = self.odom.pose.pose.position
            self.target = [p.x, p.y+4., p.z+2.]
            low, high = [p.x-10., p.y-10., p.z-2.], [p.x+10., p.y+10., p.z+7.]
            extra = []
            executable = 'gcopter_function'
            if self.local:
                executable = 'gcopter_local_function'
                self.target = [-37.57, 231.76, 6.] if os.environ.get('GCOPTER_CASE') == 'forest' else [p.x, p.y+20., p.z+2.]
                extra = ['-p', 'map_config:=/workspace/src/tool/navigation/planner/gcopter/ros2_adapter/native/config/local_map.yaml']
            self.start('gcopter', ['ros2', 'run', 'uav_gcopter_core', executable, '--ros-args',
                '-r', '__ns:=/drone0', '-p', 'use_sim_time:=true', '-p', 'goal:='+str(self.target),
                '-p', 'map_low:='+str(low), '-p', 'map_high:='+str(high)]+extra)
            self.start('adapter', ['ros2', 'run', 'uav_gcopter_adapter', 'gcopter_adapter', '--ros-args',
                '-r', '__ns:=/drone0', '-p', 'use_sim_time:=true', '-p', 'local_mode:='+str(self.local).lower()])
        msg = String()
        msg.data = json.dumps({'ready': self.target is not None, 'target': self.target})
        self.state_pub.publish(msg)

    def run(self):
        self.start('as2', ['python3', '/workspace/deployment/airsim/as2/run.py'])
        self.start('navigation', ['ros2', 'run', 'as2_navigation_inputs', 'navigation_inputs_node', '--ros-args',
            '-r', '__ns:=/drone0', '--params-file', '/workspace/src/basic_function/perception/navigation_inputs/config/drone0.yaml'])
        try:
            while not self.stopping:
                rclpy.spin_once(self, timeout_sec=.1)
        finally:
            for _, child, _ in self.children:
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGTERM)
            for _, child, stream in self.children:
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=5)
                stream.close()


def main():
    rclpy.init()
    node = Runtime()
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'stopping', True))
    signal.signal(signal.SIGINT, lambda *_: setattr(node, 'stopping', True))
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
