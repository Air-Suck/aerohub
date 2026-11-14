"""Synthetic ROS algorithm test, NOT AirSim or FollowTimedPath acceptance.

Runs the real ROG-Map and GCOPTER node. An explicitly fake executor acknowledges
plans and feeds their PVA back as odometry, to check rolling boundary semantics.
"""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_planner_interfaces.msg import TimedPath


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class Probe(Node):
    def __init__(self):
        super().__init__('gcopter_local_probe')
        self.paths = []
        self.invalidated = False
        self.status = {}
        self.odom = self.create_publisher(Odometry, '/drone0/navigation/odom', qos_profile_sensor_data)
        self.cloud = self.create_publisher(PointCloud2, '/drone0/navigation/points', qos_profile_sensor_data)
        self.execution = self.create_publisher(String, '/drone0/gcopter/state', 10)
        self.create_subscription(TimedPath, '/drone0/gcopter/plan', self.on_plan, 10)
        self.create_subscription(String, '/drone0/gcopter/local_status',
                                 lambda m: setattr(self, 'status', json.loads(m.data)), 10)
        # Dense spherical returns; synthetic geometry is used only in this test.
        self.rays = []
        for elevation in range(-75, 76, 2):
            e = math.radians(elevation)
            for azimuth in range(0, 360, 2):
                a = math.radians(azimuth)
                self.rays.append((12 * math.cos(e) * math.cos(a),
                                  12 * math.cos(e) * math.sin(a), 12 * math.sin(e)))
        self.scan_enabled = True
        self.last_scan = 0
        self.create_timer(.02, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_plan(self, path):
        if not path.task_id:
            self.invalidated = True
            return
        assert path.revision == len(self.paths) + 1
        assert seconds(path.valid_from) > self.now_s() + .1
        assert len(path.points) > 2
        assert all(seconds(b.time_from_start) > seconds(a.time_from_start)
                   for a, b in zip(path.points, path.points[1:]))
        if self.paths:
            previous = self.paths[-1]
            t = seconds(path.valid_from) - seconds(previous.valid_from)
            point = self.sample(previous, t)
            # Nearest 20ms sample permits at most 60mm at the 3m/s bound.
            for field, tolerance in [('position', .08), ('velocity', .12), ('acceleration', .3)]:
                left, right = getattr(path.points[0], field), getattr(point, field)
                assert math.dist([left.x, left.y, left.z], [right.x, right.y, right.z]) < tolerance
            assert abs(math.remainder(path.points[0].yaw - point.yaw, 2*math.pi)) < .15
        moving = [point for point in path.points
                  if math.hypot(point.velocity.x, point.velocity.y) > .3]
        assert moving
        assert max(abs(math.remainder(point.yaw-
            math.atan2(point.velocity.y, point.velocity.x), 2*math.pi)) for point in moving) < .08
        self.paths.append(path)

    @staticmethod
    def sample(path, t):
        return min(path.points, key=lambda p: abs(seconds(p.time_from_start) - t))

    def tick(self):
        now = self.now_s()
        state = Odometry()
        state.header.frame_id = 'drone0/odom'
        state.child_frame_id = 'drone0/base_link'
        state.header.stamp = self.get_clock().now().to_msg()
        state.pose.pose.orientation.w = 1.
        state.pose.pose.position.z = 2.
        eligible = [p for p in self.paths if seconds(p.valid_from) <= now]
        if eligible:
            path = eligible[-1]
            p = self.sample(path, now - seconds(path.valid_from))
            state.pose.pose.position = p.position
            state.twist.twist.linear = p.velocity
        self.odom.publish(state)
        execution = String()
        execution.data = json.dumps(dict(enabled=True, error='',
            task=self.paths[-1].task_id if self.paths else '', accepted_revision=len(self.paths)))
        self.execution.publish(execution)
        if self.scan_enabled and now - self.last_scan > .3:
            self.last_scan = now
            origin = state.pose.pose.position
            cloud = PointCloud2()
            cloud.header = state.header
            cloud.height, cloud.width = 1, len(self.rays)
            cloud.fields = [PointField(name=name, offset=i * 4, datatype=7, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.point_step, cloud.row_step = 12, 12 * len(self.rays)
            cloud.data = b''.join(struct.pack('<fff', x+origin.x, y+origin.y, z+origin.z)
                                  for x, y, z in self.rays)
            self.cloud.publish(cloud)

    def wait(self, predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError(str(self.status))
            rclpy.spin_once(self, timeout_sec=.02)


def main():
    rclpy.init()
    node = Probe()
    config = Path(__file__).resolve().parents[1] / 'config/local_map.yaml'
    out = Path(os.environ.get('GCOPTER_TEST_OUTPUT', '/workspace/log/gcopter/local-ros-check'))
    out.mkdir(parents=True, exist_ok=True)
    with (out / 'node.log').open('w') as log:
        child = subprocess.Popen(['ros2', 'run', 'uav_gcopter_core', 'gcopter_local_function',
            '--ros-args', '-r', '__ns:=/drone0', '-p', 'goal:=[0.0, 20.0, 2.0]',
            '-p', 'map_config:='+str(config)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            health = node.create_client(Trigger, '/drone0/gcopter/health')
            plan = node.create_client(Trigger, '/drone0/gcopter/plan_once')
            node.wait(lambda: health.service_is_ready() and plan.service_is_ready())
            ready = False
            ready_deadline = time.monotonic()+30
            while not ready:
                if time.monotonic()>ready_deadline:
                    raise TimeoutError('local inputs unavailable: '+str(node.status))
                future = health.call_async(Trigger.Request())
                node.wait(future.done)
                ready = future.result().success
                if not ready:
                    until = time.monotonic()+.2
                    node.wait(lambda: time.monotonic() > until)
            future = plan.call_async(Trigger.Request())
            node.wait(future.done)
            assert future.result().success
            node.wait(lambda: len(node.paths) >= 3, 45)
            assert not node.paths[0].is_final
            node.scan_enabled = False
            node.wait(lambda: node.invalidated, 5)
            result = dict(success=True, evidence='synthetic-ROS-real-algorithm-fake-executor',
                          revisions=len(node.paths), stale_lidar_stopped=True,
                          local_status=node.status)
            (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result))
        finally:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=10)
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
