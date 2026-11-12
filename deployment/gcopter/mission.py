"""PC acceptance client: real registered LiDAR -> GCOPTER -> FollowTimedPath."""
import json
import math
import os
from pathlib import Path
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.serialization import serialize_message
from as2_msgs.action import Takeoff, Land
from as2_msgs.msg import PlatformInfo, TrajectorySetpoints
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from uav_planner_interfaces.msg import TimedPath
from rosgraph_msgs.msg import Clock
sys.path.insert(0, '/workspace/simulator/airsim/sensors/uav_as2_sensor_adapter')
from rpc import Rpc


class Mission(Node):
    def __init__(self):
        super().__init__('gcopter_acceptance', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.out = Path('/evidence')
        self.odom = self.platform = None
        self.state = {}
        self.runtime = {}
        self.plans = []
        self.references = 0
        self.reference = None
        self.last_clock = None
        self.clock_regressed = False
        self.case = os.environ.get('GCOPTER_CASE', 'short')
        self.local = self.case in ('local', 'forest')
        self.local_status = {}
        self.route_samples = []
        self.route_started = None
        self.midpoint_captured = False
        self.result = {'success': False, 'platform_input': False, 'behavior_consumed': False,
                       'mission_role': 'workstation-ROS-client', 'compute_role': 'independent-compute-runtime'}
        self.rpc = Rpc('127.0.0.1', 43482, timeout=5)
        self.collision_stamp = None
        self.samples = (self.out/'flight.jsonl').open('w')
        self.create_subscription(Odometry, '/drone0/sensor_measurements/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PlatformInfo, '/drone0/platform/info', lambda m: setattr(self, 'platform', m), qos_profile_sensor_data)
        self.create_subscription(String, '/drone0/gcopter/state', lambda m: setattr(self, 'state', json.loads(m.data)), 10)
        self.create_subscription(String, '/drone0/gcopter/runtime_state', lambda m: setattr(self, 'runtime', json.loads(m.data)), 10)
        self.create_subscription(TimedPath, '/drone0/gcopter/plan', self.on_plan, 10)
        self.create_subscription(String, '/drone0/gcopter/local_status',
            lambda m: setattr(self, 'local_status', json.loads(m.data)), 10)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoints, '/drone0/motion_reference/trajectory',
            self.on_reference, qos_profile_sensor_data)
        self.create_timer(0.1, self.record)

    def on_odom(self, msg):
        self.odom = msg

    def on_plan(self, path):
        if path.task_id:
            self.plans.append(path)
            (self.out/f'output-plan-{path.revision:04d}.cdr').write_bytes(serialize_message(path))
        else:
            (self.out/'plan-invalidation.cdr').write_bytes(serialize_message(path))

    def on_reference(self, msg):
        self.references += 1
        if msg.setpoints:
            point = msg.setpoints[0]
            self.reference = dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                position=[point.position.x, point.position.y, point.position.z],
                velocity=[point.twist.x, point.twist.y, point.twist.z], yaw=point.yaw_angle)

    def on_clock(self, msg):
        stamp = msg.clock.sec*10**9+msg.clock.nanosec
        if self.last_clock is not None and stamp < self.last_clock:
            self.clock_regressed = True
        self.last_clock = stamp

    def position(self):
        p = self.odom.pose.pose.position
        return [p.x, p.y, p.z]

    def speed(self):
        v = self.odom.twist.twist.linear
        return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

    def yaw(self):
        q = self.odom.pose.pose.orientation
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def record(self):
        if self.odom:
            if self.route_started is not None:
                self.route_samples.append((self.get_clock().now().nanoseconds/1e9,
                                           self.position(), self.speed()))
            self.samples.write(json.dumps({'time': self.get_clock().now().nanoseconds/1e9,
                'position': self.position(), 'speed': self.speed(), 'yaw': self.yaw(), 'adapter': self.state,
                'reference': self.reference, 'local_planner': self.local_status})+'\n')
            self.samples.flush()

    def wait(self, predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('condition timeout')
            rclpy.spin_once(self, timeout_sec=0.02)

    def service(self, kind, name, request, timeout=20):
        client = self.create_client(kind, name)
        self.wait(client.service_is_ready)
        future = client.call_async(request)
        self.wait(future.done, timeout)
        response = future.result()
        if not response.success:
            raise RuntimeError(name + ': ' + str(response))
        return response

    def action(self, kind, name, goal, success_field):
        client = ActionClient(self, kind, name)
        self.wait(client.server_is_ready, 60)
        future = client.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(name + ' rejected')
        result = handle.get_result_async()
        self.wait(result.done, 90)
        if result.result().status != 4 or not getattr(result.result().result, success_field):
            raise RuntimeError(name + ' failed')

    def capture(self, name):
        response = self.rpc.call('simGetImages', [dict(camera_name='swarm_overhead', image_type=0,
            pixels_as_float=False, compress=True)], '', True)[0]
        (self.out/(name+'.png')).write_bytes(bytes(response['image_data_uint8']))

    def run(self):
        try:
            self.wait(lambda: self.odom is not None and self.platform is not None and self.runtime.get('ready') and bool(self.state), 90)
            settled = time.monotonic()
            self.wait(lambda: time.monotonic()-settled >= 3)
            authorities = {topic: len(self.get_publishers_info_by_topic(topic)) for topic in
                ('/clock', '/drone0/sensor_measurements/odom')}
            if self.clock_regressed or any(value != 1 for value in authorities.values()):
                raise RuntimeError('clock/state authority gate failed before arming: '+str(authorities))
            self.result['authority_counts'] = authorities
            self.service(SetBool, '/drone0/set_arming_state', SetBool.Request(data=True))
            self.wait(lambda: self.platform.armed)
            self.service(SetBool, '/drone0/set_offboard_mode', SetBool.Request(data=True))
            self.wait(lambda: self.platform.offboard)
            self.action(Takeoff, '/drone0/TakeoffBehavior', Takeoff.Goal(takeoff_height=2., takeoff_speed=.5), 'takeoff_success')
            settled = time.monotonic()
            self.wait(lambda: time.monotonic()-settled > 6 and self.speed() < .05, 30)
            collision_baseline = self.rpc.call('simGetCollisionInfo', 'PX4')
            self.collision_stamp = collision_baseline['time_stamp']
            (self.out/'collision-baseline.json').write_text(
                json.dumps(collision_baseline, indent=2, default=str)+'\n')
            if self.local:
                self.capture('takeoff-overhead')
            start = self.position()
            target = self.runtime['target']
            # Batch mode publishes one snapshot; local mode records readiness while the
            # Function continues to consume the live registered LiDAR topic directly.
            self.service(Trigger, '/drone0/gcopter/snapshot', Trigger.Request())
            health = self.create_client(Trigger, '/drone0/gcopter/health')
            self.wait(health.service_is_ready)
            ready = False
            deadline = time.monotonic()+10
            while not ready and time.monotonic() < deadline:
                self.service(Trigger, '/drone0/gcopter/snapshot', Trigger.Request())
                future = health.call_async(Trigger.Request())
                self.wait(future.done)
                ready = future.result().success
                if not ready:
                    until = time.monotonic()+.1
                    self.wait(lambda: time.monotonic() >= until)
            if not ready:
                raise RuntimeError('Function input readiness failed')
            self.service(SetBool, '/drone0/gcopter/enable', SetBool.Request(data=True))
            if self.local:
                self.wait(lambda: self.state.get('enabled'))
                # Let the Function receive the execution readiness publication too.
                until = time.monotonic()+.2
                self.wait(lambda: time.monotonic() >= until)
            self.service(Trigger, '/drone0/gcopter/plan_once', Trigger.Request(), timeout=90)
            self.wait(lambda: bool(self.plans) or bool(self.state.get('error')), 60)
            if self.state.get('error'):
                raise RuntimeError(self.state['error'])
            (self.out/'output-plan.cdr').write_bytes(serialize_message(self.plans[-1]))
            self.result.update(platform_input=True, map_source='live-lidar-rog-map' if self.local else 'registered_lidar_snapshot',
                               plan_points=len(self.plans[-1].points), start=start, target=target)
            self.route_started = self.get_clock().now().nanoseconds/1e9
            if self.case in ('cancel', 'map-reset'):
                self.wait(lambda: bool(self.state.get('task')))
                began = self.get_clock().now().nanoseconds/1e9
                self.wait(lambda: self.get_clock().now().nanoseconds/1e9-began >= 1)
                if self.case == 'cancel':
                    self.service(SetBool, '/drone0/gcopter/enable', SetBool.Request(data=False))
                else:
                    self.service(Trigger, '/drone0/gcopter/reset', Trigger.Request())
                self.wait(lambda: self.state.get('stop_confirmed'), 15)
                self.wait(lambda: self.speed() < .3, 10)
                reference_count = self.references
                stopped = self.get_clock().now().nanoseconds/1e9
                self.wait(lambda: self.get_clock().now().nanoseconds/1e9-stopped >= 2, 10)
                if self.references != reference_count:
                    raise RuntimeError('trajectory output continued after stop confirmation')
                self.result.update(fault_validated=True, fault=self.case,
                    behavior_consumed=True, stop_confirmed=True, output_silent=True)
                return
            stable = None
            def arrived():
                nonlocal stable
                if self.state.get('error'):
                    raise RuntimeError(self.state['error'])
                if self.clock_regressed:
                    raise RuntimeError('clock regressed during route')
                if not self.local and self.speed() > 4.04:
                    raise RuntimeError(f'speed limit exceeded: {self.speed():.6f}m/s')
                collision = self.rpc.call('simGetCollisionInfo', 'PX4')
                if collision['time_stamp'] != self.collision_stamp:
                    self.result['collision'] = collision
                    (self.out/'collision.json').write_text(
                        json.dumps(collision, indent=2, default=str)+'\n')
                    raise RuntimeError('collision:'+str(collision.get('object_name', 'unknown')))
                if math.dist(self.position(), target) <= (1.5 if self.local else .25) and self.speed() < .3:
                    stable = stable or time.monotonic()
                else:
                    stable = None
                return self.state.get('complete') and stable is not None and time.monotonic()-stable >= 2
            self.wait(arrived, 300 if self.case == 'forest' else 90)
            self.result.update(plan_revisions=len(self.plans),
                accepted_revision=self.state.get('accepted_revision'), local_status=self.local_status)
            self.result.update(behavior_consumed=True, arrival_error=math.dist(self.position(), target))
            if self.local:
                self.capture('arrival-overhead')
        except BaseException as exc:
            self.result['error'] = str(exc)
        finally:
            if self.platform and self.platform.armed:
                try:
                    if self.state:
                        self.service(SetBool, '/drone0/gcopter/enable', SetBool.Request(data=False))
                        self.wait(lambda: self.state.get('stop_confirmed'), 15)
                        self.result['stop_confirmed'] = True
                    self.action(Land, '/drone0/LandBehavior', Land.Goal(land_speed=.5), 'land_success')
                    self.wait(lambda: not self.platform.armed, 30)
                    self.result['landed_disarmed'] = True
                except BaseException as exc:
                    self.result['recovery_error'] = str(exc)
            self.result['success'] = bool(self.result.get('behavior_consumed') and self.result.get('stop_confirmed') and
                self.result.get('landed_disarmed') and not self.result.get('error') and not self.result.get('recovery_error'))
            (self.out/'result.json').write_text(json.dumps(self.result, indent=2)+'\n')
            print(json.dumps(self.result), flush=True)

    def close(self):
        self.samples.close()


def main():
    rclpy.init()
    node = Mission()
    try:
        node.run()
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    if not node.result['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
