"""ProjectAirSim ROS YOPO task; native collision scoring remains unavailable."""
import argparse
import math
import json
from pathlib import Path
import time
import struct
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import Takeoff, Land
from rosgraph_msgs.msg import Clock
from as2_msgs.msg import PlatformInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs
import signal
from urllib.request import Request, build_opener, ProxyHandler


class Mission(Node):
    def __init__(self, output, goal, rpc_port):
        super().__init__('yopo_mission', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.output, self.goal = output, goal
        self.platform = None
        self.status = {}
        self.pose = self.twist = None
        self.status_at = 0.
        self.samples = (output/'mission.jsonl').open('w')
        self.events = (output/'plans.jsonl').open('w')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.depth = None
        self.create_subscription(Image, '/drone0/sensor_measurements/yopo_front/depth_planar/image',
                                 lambda m: setattr(self, 'depth', m), qos_profile_sensor_data)
        self.http = build_opener(ProxyHandler({}))
        self.hud_started = False
        self.result = dict(success=False, motion_requested=False, arrived_5m=False, stopped=False,
                           land_success=False, disarmed=False, collision_coverage='unverified: native collision topic not bridged')
        self.create_subscription(PlatformInfo, '/drone0/platform/info', lambda m: setattr(self, 'platform', m), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/drone0/self_localization/twist', lambda m: setattr(self, 'twist', m), qos_profile_sensor_data)
        self.create_subscription(String, '/drone0/yopo/status', self.on_status, 10)
        self.create_subscription(String, '/drone0/yopo/events', self.on_event, 100)
        self.goal_pub = self.create_publisher(PoseStamped, '/drone0/yopo/goal', 1)
        self.task_clients = []
        self.last_clock_ns = None
        self.clock_samples = 0
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.last_collision = 0.
        self.initial_collision = None

    def on_pose(self, message):
        try:
            self.pose = self.tf.transform(message, 'drone0/odom')
        except TransformException:
            pass

    def hud(self, event):
        try:
            request = Request('http://127.0.0.1:9011/aerohub/mission/'+event, data=b'', method='POST')
            with self.http.open(request, timeout=3) as response:
                response.read()
        except Exception as error:
            self.result.setdefault('hud_errors', []).append(str(error))

    def on_clock(self, msg):
        stamp=msg.clock.sec*1000000000+msg.clock.nanosec
        if self.last_clock_ns is not None and stamp < self.last_clock_ns:
            raise RuntimeError('simulation clock moved backwards; stop flight validation')
        self.last_clock_ns=stamp
        self.clock_samples+=1

    def graph_ready(self):
        expected=['/clock','/drone0/self_localization/pose','/drone0/self_localization/twist','/drone0/yopo/status']
        return self.clock_samples >= 25 and all(self.count_publishers(topic)==1 for topic in expected)

    def on_event(self, msg):
        self.events.write(msg.data+'\n'); self.events.flush()

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        self.status_at = time.monotonic()
        sample = dict(wall=time.time(), sim=self.get_clock().now().nanoseconds/1e9, **self.status)
        if self.pose is not None:
            sample['pose_frame'] = self.pose.header.frame_id
            sample['pose'] = [self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z]
        if self.twist is not None:
            sample['velocity_frame'] = self.twist.header.frame_id
            sample['velocity'] = [self.twist.twist.linear.x, self.twist.twist.linear.y, self.twist.twist.linear.z]
        self.samples.write(json.dumps(sample)+'\n')
        self.samples.flush()

    def wait(self, predicate, timeout=20, score=False):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('condition timeout: '+str(self.status))
            rclpy.spin_once(self, timeout_sec=.02)
            if score and self.pose is not None and 'origin' in self.result:
                p = self.pose.pose.position
                local = [p.x-self.result['origin'][0], p.y-self.result['origin'][1], p.z-self.result['origin'][2]]
                if not all(lo <= value <= hi for value,lo,hi in zip(local[:2],[-6.,-2.],[6.,123.])):
                    raise RuntimeError('course bounds exceeded: '+str(local))
            if score and time.monotonic()-self.status_at > 1:
                raise RuntimeError('planner status lost')

    def service(self, kind, name, request, timeout=10):
        client = self.create_client(kind, name); self.task_clients.append(client)
        self.wait(client.service_is_ready, timeout)
        future = client.call_async(request); self.wait(future.done, timeout)
        response = future.result()
        if not response.success:
            raise RuntimeError(name+' rejected: '+str(response))

    def action(self, kind, name, goal):
        client = ActionClient(self, kind, name); self.task_clients.append(client)
        self.wait(client.server_is_ready)
        accepted = client.send_goal_async(goal); self.wait(accepted.done)
        handle = accepted.result()
        if not handle.accepted:
            raise RuntimeError(name+' goal rejected')
        future = handle.get_result_async(); self.wait(future.done, 60)
        result = future.result()
        if result.status != 4 or not getattr(result.result, 'takeoff_success' if kind is Takeoff else 'land_success'):
            raise RuntimeError(name+' failed: '+str(result))

    def run(self, ground_only=False, recover_only=False, fault="none", require_obstacle=False, timeout_s=90.):
        self.result.update(goal_enu_m=list(self.goal), navigation_timeout_s=timeout_s)
        try:
            if recover_only:
                self.wait(lambda: self.platform is not None, 10)
                self.result['recovery_requested'] = True
                return
            self.wait(lambda: self.platform is not None and self.status.get('ready') and self.pose is not None and self.graph_ready(), 45)
            if self.platform.armed:
                raise RuntimeError('requires disarmed platform')
            origin = [self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z]
            self.goal = [a+b for a,b in zip(origin, self.goal)]
            self.result['origin'] = origin
            self.result['target'] = self.goal
            self.result['ground_ready'] = True
            self.result['motion_reference_publishers'] = {
                topic: [dict(node=p.node_name, namespace=p.node_namespace, type=p.topic_type)
                        for p in self.get_publishers_info_by_topic(topic)]
                for topic in ['/drone0/motion_reference/trajectory', '/drone0/motion_reference/pose']}

            if ground_only:
                if self.depth is not None:
                    image=self.depth
                    (self.output/'ground-depth.bin').write_bytes(bytes(image.data))
                    (self.output/'ground-depth.json').write_text(json.dumps(dict(width=image.width,height=image.height,encoding=image.encoding,step=image.step)))
                self.result.update(success=True, disarmed=True)
                return
            self.result['motion_requested'] = True
            self.service(SetBool, '/drone0/set_arming_state', SetBool.Request(data=True))
            self.wait(lambda: self.platform.armed)
            self.service(SetBool, '/drone0/set_offboard_mode', SetBool.Request(data=True))
            self.wait(lambda: self.platform.offboard)
            self.action(Takeoff, '/drone0/TakeoffBehavior', Takeoff.Goal(takeoff_height=2., takeoff_speed=.5))
            # Let measured motion settle; the Planner uses a future reference epoch.
            until = time.monotonic()+3
            self.wait(lambda: time.monotonic() >= until)
            goal = PoseStamped(); goal.header.frame_id = 'drone0/odom'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self.goal
            self.wait(lambda: self.depth is not None, 10)
            image = self.depth
            (self.output/'depth-before-goal.bin').write_bytes(bytes(image.data))
            (self.output/'depth-before-goal.json').write_text(json.dumps(dict(
                width=image.width, height=image.height, step=image.step, encoding=image.encoding,
                frame=image.header.frame_id, stamp=dict(sec=image.header.stamp.sec,nanosec=image.header.stamp.nanosec))))
            previous_task = self.status.get('task_id')
            self.hud('start'); self.hud_started = True
            self.goal_pub.publish(goal)
            self.wait(lambda: self.status.get('task_id') != previous_task and self.status.get('state') == 'RUNNING', 5, score=True)
            if fault != 'none':
                self.wait(lambda: self.status.get('revision', 0) >= 3, 15, score=True)
                self.result['fault'] = fault
                if fault == 'cancel':
                    self.service(Trigger, '/drone0/yopo/cancel', Trigger.Request())
                else:
                    (self.output/'fault-request.json').write_text(json.dumps(dict(fault=fault, wall=time.time()))+'\n')
            self.wait(lambda: self.status.get('state') in ('STOPPED', 'FAULT'), timeout_s, score=True)
            self.result['arrived_5m'] = 'ARRIVED_5M' in self.status.get('reason', '')
            self.result['stopped'] = self.status.get('state') == 'STOPPED'
            expected_reason = {'none':'ARRIVED_5M','cancel':'CANCELED','stale':'STALE_OR_INVALID_INPUT',
                               'worker-exit':'INFERENCE_RESULT:WORKER_UNAVAILABLE'}[fault]
            if self.status.get('state') != 'STOPPED' or not self.status.get('reason', '').startswith(expected_reason):
                raise RuntimeError('planner ended: '+str(self.status))
            self.result['expected_stop'] = True
            self.result['stop_reason'] = self.status.get('reason')
        except Exception as error:
            self.result['error'] = str(error)
        finally:
            if self.hud_started:
                self.hud('finish')
            if self.platform is not None and self.platform.armed:
                try:
                    try:
                        if self.status.get('state') != 'STOPPED':
                            self.service(Trigger, '/drone0/yopo/cancel', Trigger.Request())
                    except Exception as error:
                        self.result['cancel_error'] = str(error)
                    try:
                        self.wait(lambda: self.status.get('state') in ('STOPPED', 'FAULT'), 12)
                    except Exception as error:
                        self.result['stop_error'] = str(error)
                    self.action(Land, '/drone0/LandBehavior', Land.Goal(land_speed=.5))
                    self.result['land_success'] = True
                    # Native Land publishes its result before its own disarm cleanup.
                    # Wait for that state update instead of racing it with a second disarm.
                    try:
                        self.wait(lambda: not self.platform.armed, 5)
                    except TimeoutError:
                        try:
                            self.service(SetBool, '/drone0/set_arming_state', SetBool.Request(data=False))
                        except RuntimeError:
                            # Already-disarmed rejection is acceptable only after fresh feedback.
                            self.wait(lambda: not self.platform.armed, 3)
                        self.wait(lambda: not self.platform.armed, 15)
                except Exception as error:
                    self.result['recovery_error'] = str(error)
            self.result['disarmed'] = self.platform is not None and not self.platform.armed
            if self.result['motion_requested'] and self.result['disarmed']:
                try:
                    self.service(SetBool, '/drone0/set_offboard_mode', SetBool.Request(data=False))
                except Exception as error:
                    self.result['offboard_release_error'] = str(error)
            if not ground_only:
                self.result['success'] = bool(self.result.get('expected_stop') and self.result['stopped'] and
                                              self.result['land_success'] and self.result['disarmed'] and not self.result.get('error'))
            self.samples.close(); self.events.close()
            (self.output/'result.json').write_text(json.dumps(self.result, indent=2)+'\n')
            print(json.dumps(self.result), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ground-only', action='store_true')
    parser.add_argument('--require-obstacle', action='store_true')
    parser.add_argument('--recover-only', action='store_true')
    parser.add_argument('--fault', choices=['none','cancel','stale','worker-exit'], default='none')
    parser.add_argument('--goal', type=float, nargs=3, default=[0., 120., 2.])
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--rpc-port', type=int, default=45471)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0 or not all(math.isfinite(v) for v in args.goal):
        parser.error('timeout must be finite positive; goal must be finite')
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Mission(args.output, args.goal, args.rpc_port)
    def interrupted(*_):
        raise RuntimeError('Mission interrupted; recovering')
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        node.run(args.ground_only, args.recover_only, args.fault, args.require_obstacle, args.timeout)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
