"""GCOPTER batch or rolling plans into the shared FollowTimedPath coordinator."""
import copy
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from uav_planner_interfaces.msg import TimedPath, TimedTrajectorySegment, VehicleTrajectory
from uav_planner_framework.follow_path_client import FleetFollowPathClient
from uav_planner_framework.ros_conversions import seconds_to_time, time_to_seconds


class Adapter(Node):
    def __init__(self):
        super().__init__('gcopter_execution_adapter')
        self.vehicle = self.declare_parameter('vehicle_id', 'drone0').value
        self.frame = self.declare_parameter('frame_id', self.vehicle + '/odom').value
        self.local_mode = self.declare_parameter('local_mode', False).value
        self.last_path = None
        self.accepted_revision = 0
        self.enabled = False
        self.client = None
        self.odom = None
        self.error = ''
        self.task = ''
        self.seen = set()
        self.create_subscription(Odometry, 'sensor_measurements/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(TimedPath, 'gcopter/plan', self.on_plan, 1)
        self.create_subscription(String, 'gcopter/failure',
            lambda m: self.fail('LOCAL_PLANNER', m.data) if self.local_mode and self.enabled else None, 10)
        self.create_subscription(PointCloud2, 'gcopter/map', self.on_map, qos_profile_sensor_data)
        self.create_service(SetBool, 'gcopter/enable', self.enable)
        self.publisher = self.create_publisher(String, 'gcopter/state', 10)
        self.create_timer(0.02, self.tick)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.odom = msg if msg.header.frame_id == self.frame and all(
            math.isfinite(x) for x in (p.x, p.y, p.z, v.x, v.y, v.z)) else None

    def on_map(self, msg):
        if self.local_mode:
            return  # Local planner checks evolving map against its committed curve.
        if self.client and self.client.revision and not self.client.complete and not self.client.cancel_started:
            self.fail('MAP_CHANGED', 'static-map task invalidated; wait for stop before replanning')

    def enable(self, request, response):
        if request.data:
            if self.enabled or (self.client and not self.client.quiescent):
                response.message = 'stop previous execution first'
                return response
            config = {'vehicles': [self.vehicle], 'deployment': {self.vehicle: {
                'namespace': self.vehicle, 'px4_origin_in_fleet_ned': [0., 0., 0.]}},
                'planner': {'parameters': {'max_velocity': 4.0}}}
            self.client = FleetFollowPathClient(self, config, self.fail)
            self.enabled = True
            self.error = ''
            self.task = ''
            self.last_path = None
            self.accepted_revision = 0
        else:
            self.enabled = False
            if self.client:
                self.client.cancel()
        response.success = True
        response.message = 'enabled' if request.data else 'stop requested; confirm gcopter/state'
        return response

    def fail(self, code, reason):
        self.error = code + ':' + reason
        self.enabled = False
        if self.client:
            self.client.cancel()

    def on_plan(self, path):
        if not self.enabled:
            return
        if self.local_mode and not path.task_id and not path.points:
            return  # The reliable failure topic carries the actual stop cause.
        try:
            if not self.client.ready or self.client.pending or (self.client.revision and not self.local_mode):
                raise ValueError('execution unavailable or task already committed')
            if path.interface_version != 1 or path.frame_id != self.frame or path.vehicle_id != self.vehicle:
                raise ValueError('identity/frame/interface mismatch')
            if ((not path.is_final and not self.local_mode) or path.valid_fields != 7 or
                    len(path.points) < 2 or not path.task_id or
                    (not self.local_mode and path.task_id in self.seen)):
                raise ValueError('invalid/repeated batch')
            if self.local_mode:
                if path.revision != self.client.revision + 1 or (self.task and path.task_id != self.task):
                    raise ValueError('task/revision mismatch')
                if time_to_seconds(path.valid_from) <= self.now_seconds() + .1:
                    raise ValueError('late local trajectory')
            if not math.isfinite(path.max_speed) or not 0 < path.max_speed <= 4.0 or path.revision < 1:
                raise ValueError('unsupported speed/revision')
            age = self.now_seconds() - time_to_seconds(path.generated_at)
            if not 0 <= age <= 0.5:
                raise ValueError('stale/future plan')
            state = self.odom
            if state is None or state.header.frame_id != self.frame:
                raise ValueError('state unavailable')
            if not 0 <= self.now_seconds() - time_to_seconds(state.header.stamp) <= 0.2:
                raise ValueError('stale/future state')
            position = state.pose.pose.position
            velocity = state.twist.twist.linear
            start = path.points[0]
            first = self.client.revision == 0
            if (first or not self.local_mode) and any(abs(getattr(field, axis)) > 1e-5 for field in (start.velocity, start.acceleration)
                   for axis in ('x', 'y', 'z')):
                raise ValueError('nonstationary planned start')
            values = (position.x, position.y, position.z, velocity.x, velocity.y, velocity.z)
            if not all(math.isfinite(x) for x in values):
                raise ValueError('invalid state')
            if first and math.dist((position.x, position.y, position.z), (start.position.x, start.position.y, start.position.z)) > (0.2 if self.local_mode else 0.05):
                raise ValueError('start moved during solve')
            if first and math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2) > (0.15 if self.local_mode else 0.05):
                raise ValueError('stationary commit required')
            segment = TimedTrajectorySegment()
            segment.interface_version = 1
            segment.task_id = path.task_id
            segment.generated_at = copy.deepcopy(path.generated_at)
            # Explicit new stationary execution commitment, after revalidating held state.
            if self.local_mode:
                segment.valid_from = copy.deepcopy(path.valid_from)
            else:
                seconds_to_time(self.now_seconds() + 0.5, segment.valid_from)
            segment.is_final = path.is_final
            vehicle = VehicleTrajectory()
            vehicle.vehicle_id = self.vehicle
            vehicle.valid_fields = 7
            previous = -1
            for original in path.points:
                point = copy.deepcopy(original)
                stamp = point.time_from_start.sec*10**9 + point.time_from_start.nanosec
                if stamp <= previous or (previous < 0 and stamp != 0):
                    raise ValueError('non-increasing sample times')
                previous = stamp
                fields = (point.position, point.velocity, point.acceleration)
                if not all(math.isfinite(getattr(field, axis)) for field in fields for axis in ('x', 'y', 'z')) or not math.isfinite(point.yaw):
                    raise ValueError('nonfinite sample')
                if math.sqrt(point.velocity.x**2+point.velocity.y**2+point.velocity.z**2) > path.max_speed*1.01:
                    raise ValueError('sample speed limit')
                # Shared coordinator expects fleet NED and transforms into local ENU.
                for field in fields:
                    field.x, field.y, field.z = field.y, field.x, -field.z
                point.yaw = math.pi/2-point.yaw
                vehicle.points.append(point)
            segment.vehicle_trajectories = [vehicle]
            # Preserve this plan's bound in the Behavior goal; the 4m/s
            # adapter admission ceiling is not the requested execution speed.
            self.client.config['planner']['parameters']['max_velocity'] = path.max_speed
            self.seen.add(path.task_id)
            self.task = path.task_id
            self.last_path = path
            self.client.submit(segment)
        except (ValueError, TypeError) as exc:
            self.fail('PLAN_REJECTED', str(exc))

    def tick(self):
        if self.client:
            self.client.tick(self.now_seconds())
            if (self.local_mode and self.client.revision and not self.client.pending and
                    not self.client.failed and set(self.client.handles) == set(self.client.actions)):
                self.accepted_revision = self.client.revision
            if self.enabled and self.client.revision and not self.client.complete:
                if self.odom is None or not 0 <= self.now_seconds()-time_to_seconds(self.odom.header.stamp) <= 0.2:
                    self.fail('STATE_LOST', 'state watchdog')
        msg = String()
        msg.data = json.dumps({'task': self.task, 'enabled': self.enabled, 'error': self.error,
                               'complete': bool(self.client and self.client.complete),
                               'accepted_revision': self.accepted_revision,
                               'stop_confirmed': self.client is None or self.client.quiescent})
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    finally:
        if node.client:
            node.client.cancel()
        node.destroy_node()
        rclpy.try_shutdown()
