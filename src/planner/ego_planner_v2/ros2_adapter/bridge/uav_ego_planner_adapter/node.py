"""Native EGO inputs/outputs, shared fleet client, real extended AS2 FollowPath."""
import copy
import json
import math
import time
import signal
from collections import deque
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from as2_msgs.msg import BehaviorStatus
from quadrotor_msgs.msg import GoalSet
from traj_utils.msg import PolyTraj
from uav_planner_interfaces.msg import TimedTrajectorySegment, PlannerFailure
from uav_planner_framework.follow_path_client import FleetFollowPathClient
from .conversion import Polynomial, segment_for, world_odometry, nanoseconds, equivalent_final_tail
from .peer_cloud import remove_peer_returns
from .scope import AdapterScope

class EgoAdapter(Node):
    def __init__(self):
        super().__init__('ego_as2_adapter')
        path = self.declare_parameter('resolved_config', '').value
        self.scope = AdapterScope(yaml.safe_load(Path(path).read_text()),
                                  self.declare_parameter('vehicle', '').value)
        self.config = self.scope.config
        self.vehicles = self.config['vehicles']
        if not self.vehicles or len(set(self.vehicles)) != len(self.vehicles):
            raise ValueError('unique nonempty vehicles required')
        self.ids = self.config['planner_ids']
        self.peer_mask_radius=float(self.config.get('peer_body_mask_radius_m',0.))
        if not math.isfinite(self.peer_mask_radius) or not 0 <= self.peer_mask_radius <= .75:
            raise ValueError('peer mask radius must be within [0, 0.75] m')
        self.peer_odometry={v:deque(maxlen=60) for v in self.scope.fleet['vehicles']}
        if set(self.ids) != set(self.vehicles) or len(set(self.ids.values())) != len(self.ids):
            raise ValueError('unique planner ID per vehicle required')
        limits = self.config['limits']
        self.max_v = float(limits['velocity'])*(1+float(limits['tolerance']))
        self.max_a = float(limits['acceleration'])*(1+float(limits['tolerance']))
        if not all(math.isfinite(x) and x > 0 for x in [self.max_v,self.max_a]): raise ValueError('invalid limits')
        self.config['planner']['parameters']['max_velocity'] = self.max_v
        params = self.config['planner']['parameters']
        if not .05 <= float(params['valid_from_lead']) <= 1 or not .001 <= float(params['sample_dt']) <= .2:
            raise ValueError('invalid execution lead or sampling interval')
        self.started = False; self.closed = False; self.dirty = False
        self.goals = {}; self.latest = {}; self.received = {}; self.stamps = {}; self.goal_messages = {}
        self.goal_epochs = {}
        self.submitted = {}; self.submitted_epochs = {}; self.submitted_final = False
        self.rejected = {}; self.counts = {'segments':0, 'trajectories':0, 'goals':0}
        self.state = 'WAITING'; self.reason = ''; self.last_clock = self.get_clock().now().nanoseconds
        self.finished_vehicles = []; self.stop_started = None
        self.planning_started = None
        self.ready_since = None; self.behavior_status = {}
        self.status_pub = self.create_publisher(String, self.scope.api('status'), 10)
        self.failure_pub = self.create_publisher(PlannerFailure, self.scope.planner('failure'), 10)
        self.segment_pub = self.create_publisher(TimedTrajectorySegment, self.scope.planner('trajectory'), 10)
        goal_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pubs = {}
        self.create_subscription(GoalSet, self.scope.api('goal'), self.goal, goal_qos)
        self.stop_pubs = {}; self.odom_pubs = {}; self.cloud_pubs = {}
        for vehicle in self.vehicles:
            deployment = self.config['deployment'][vehicle]
            ns = deployment['namespace'].strip('/')
            self.goal_pubs[vehicle] = self.create_publisher(GoalSet, self.scope.native(vehicle,'goal'), goal_qos)
            self.stop_pubs[vehicle] = self.create_publisher(Empty, f'/{ns}/ego_planner/mandatory_stop', 10)
            self.odom_pubs[vehicle] = self.create_publisher(Odometry, self.scope.native(vehicle,'odom'), qos_profile_sensor_data)
            self.cloud_pubs[vehicle] = self.create_publisher(PointCloud2, self.scope.native(vehicle,'points'), qos_profile_sensor_data)
            self.create_subscription(Odometry, f'/{ns}/navigation/odom', lambda m,v=vehicle:self.odometry(v,m), qos_profile_sensor_data)
            self.create_subscription(PointCloud2, f'/{ns}/navigation/points', lambda m,v=vehicle:self.cloud(v,m), qos_profile_sensor_data)
            self.create_subscription(PolyTraj, f'/{ns}/ego_planner/planning/trajectory', lambda m,v=vehicle:self.trajectory(v,m), 10)
            self.create_subscription(Empty, f'/{ns}/ego_planner/planning/heartbeat', lambda m,v=vehicle:self.heartbeat(v), 10)
            self.create_subscription(BehaviorStatus, f'/{ns}/follow_timed_path/_behavior/behavior_status',
                lambda m,v=vehicle:self.behavior_status.update({v:(m.status,time.monotonic())}),10)
        # Peers are read-only context for the static-map body mask. They never
        # become execution members or receive this instance's mandatory_stop.
        if self.peer_mask_radius:
            for peer in self.scope.fleet['vehicles']:
                if peer in self.vehicles: continue
                ns = self.scope.fleet['deployment'][peer]['namespace']
                self.create_subscription(Odometry, f'/{ns}/navigation/odom',
                    lambda m,v=peer:self.peer_input(v,m), qos_profile_sensor_data)
        self.client = FleetFollowPathClient(self, self.config, self.fail)
        self.create_service(Trigger, self.scope.api('cancel'), self.cancel)
        self.create_timer(.05, self.tick)
        self.create_timer(.2, self.report)

    def reject(self, key):
        self.rejected[key] = self.rejected.get(key,0)+1

    def valid_stamp(self, vehicle, kind, msg):
        stamp = nanoseconds(msg.header.stamp)
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if stamp <= self.stamps.get((vehicle,kind),0) or not -.01 <= age <= .5:
            self.reject(kind+'_stamp'); return False
        return True

    def accepted_input(self, vehicle, kind, msg):
        self.stamps[(vehicle,kind)] = nanoseconds(msg.header.stamp)
        self.received[(vehicle,kind)] = time.monotonic()

    def odometry(self, vehicle, msg):
        if not self.valid_stamp(vehicle,'odom',msg): return
        try:
            converted = world_odometry(msg, self.config['deployment'][vehicle]['namespace'], self.config['deployment'][vehicle]['px4_origin_in_fleet_ned'])
        except ValueError:
            self.reject('odom_contract'); return
        self.odom_pubs[vehicle].publish(converted)
        p=converted.pose.pose.position
        self.peer_odometry[vehicle].append((nanoseconds(converted.header.stamp),time.monotonic(),(p.x,p.y,p.z)))
        self.accepted_input(vehicle,'odom',msg)

    def cloud(self, vehicle, msg):
        if not self.valid_stamp(vehicle,'cloud',msg): return
        deployment = self.config['deployment'][vehicle]
        if msg.header.frame_id != deployment['namespace']+'/odom':
            self.reject('cloud_frame'); return
        try:
            origin = deployment['px4_origin_in_fleet_ned']
            points = [(float(p[0])+origin[1],float(p[1])+origin[0],float(p[2])-origin[2])
                      for p in point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True)]
            if not points or not all(math.isfinite(x) for p in points for x in p): raise ValueError('empty/nonfinite cloud')
            if self.peer_mask_radius:
                points,removed=remove_peer_returns(points,[self.peer_odometry[v] for v in self.scope.fleet['vehicles'] if v!=vehicle],
                    nanoseconds(msg.header.stamp),time.monotonic(),self.peer_mask_radius)
                key='peer_points_removed_'+vehicle
                self.counts[key]=self.counts.get(key,0)+removed
                key='peer_mask_frames_'+vehicle
                self.counts[key]=self.counts.get(key,0)+1
                if not points:raise ValueError('empty static cloud after peer mask')
            header = copy.deepcopy(msg.header); header.frame_id = 'world'
            converted = point_cloud2.create_cloud_xyz32(header,points)
        except (ValueError,AssertionError,KeyError):
            self.reject('cloud_contract'); return
        self.cloud_pubs[vehicle].publish(converted)
        self.accepted_input(vehicle,'cloud',msg)

    def peer_input(self, vehicle, msg):
        if not self.valid_stamp(vehicle,'peer_odom',msg): return
        deployment = self.scope.fleet['deployment'][vehicle]
        try:
            converted = world_odometry(msg, deployment['namespace'], deployment['px4_origin_in_fleet_ned'])
        except ValueError:
            self.reject('peer_odom_contract'); return
        p = converted.pose.pose.position
        self.peer_odometry[vehicle].append((nanoseconds(converted.header.stamp),time.monotonic(),(p.x,p.y,p.z)))
        self.accepted_input(vehicle,'peer_odom',msg)

    def heartbeat(self, vehicle):
        self.received[(vehicle,'heartbeat')] = time.monotonic()

    def goal(self, msg):
        matching = [v for v in self.vehicles if self.ids[v] == msg.drone_id]
        if self.closed or not matching or not all(math.isfinite(x) for x in msg.goal) or msg.goal[2] < -.1:
            self.reject('goal'); return
        vehicle = matching[0]
        self.goals[vehicle] = list(msg.goal); self.goal_messages[vehicle] = msg
        self.goal_epochs[vehicle] = self.get_clock().now().nanoseconds
        self.latest.pop(vehicle,None)
        self.counts['goals'] += 1
        if self.started:
            self.planning_started = time.monotonic()
            self.goal_pubs[vehicle].publish(msg)

    def trajectory(self, vehicle, msg):
        if not self.started or self.closed: return
        if nanoseconds(msg.start_time) < self.goal_epochs[vehicle]:
            self.reject('previous_goal_trajectory'); return
        try:
            source = Polynomial(msg,self.ids[vehicle],self.max_v,self.max_a)
            now = self.get_clock().now().nanoseconds
            if not -10_000_000 <= now-source.start_ns <= 2_000_000_000:
                raise ValueError('invalid source time')
            if vehicle in self.latest and source.start_ns <= self.latest[vehicle].start_ns:
                self.reject('duplicate_trajectory'); return
            # A stationary emergency output is not an executable navigation plan.
            p,v,a = source.sample(source.start_ns)
            end = source.sample(source.end_ns)[0]
            if math.dist(p,end) < 1e-6 and math.dist(end,self.goals[vehicle]) > .03:
                raise ValueError('planner emergency hold')
        except (ValueError,ArithmeticError) as exc:
            self.fail('INVALID_TRAJECTORY',str(exc)); return
        self.latest[vehicle] = source; self.dirty = True
        self.counts['trajectories'] += 1

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if self.started and not self.closed and now < self.last_clock:
            self.fail('CLOCK_RESET','ROS time moved backward')
        self.last_clock = now
        if self.closed: return
        healthy = all(time.monotonic()-self.received.get((v,k),0) < .75
                      for v in self.vehicles for k in ['odom','cloud','heartbeat'])
        if not self.started:
            idle = all(v in self.behavior_status and self.behavior_status[v][0] == BehaviorStatus.IDLE
                       and time.monotonic()-self.behavior_status[v][1] < .75 for v in self.vehicles)
            ready = healthy and idle and self.client.ready and len(self.goals) == len(self.vehicles) and all(p.get_subscription_count() >= 1 for p in self.goal_pubs.values())
            if not ready: self.ready_since = None; return
            if self.ready_since is None: self.ready_since = time.monotonic()
            # Discovery alone does not prove the newly created Action request
            # endpoints have settled. Keep the actual execution ACK deadline.
            if time.monotonic()-self.ready_since >= .5:
                self.started = True; self.state = 'RUNNING'
                self.planning_started = time.monotonic()
                for vehicle,msg in self.goal_messages.items(): self.goal_pubs[vehicle].publish(msg)
            return
        if not healthy:
            self.fail('STALE_INPUT','odometry, cloud or planner heartbeat expired'); return
        if len(self.latest) != len(self.vehicles) and time.monotonic()-self.planning_started > 5:
            self.fail('PLANNING_TIMEOUT','no complete valid fleet plan within five seconds'); return
        self.client.tick(now/1e9)
        if self.client.complete:
            self.finished_vehicles = sorted(self.client.results)
            self.closed = True; self.state = 'STOPPING'; self.stop_started = time.monotonic()
            for pub in self.stop_pubs.values(): pub.publish(Empty())
            self.client.cancel()
            return
        if not self.dirty or len(self.latest) != len(self.vehicles) or self.client.pending: return
        if self.client.results or self.client.settling_vehicles:
            if self.redundant_tail(now): self.dirty = False; return
            self.fail('FINAL_EXECUTION_MISMATCH','new plan differs after a vehicle began final settling'); return
        try:
            segment = segment_for(self.config,self.latest,self.goals,now)
        except ValueError as exc:
            # Native EGO can replan the last ~0.1 s even though AS2 already has
            # that final tail. Keep the accepted plan only after a full-interval
            # equivalence check, never on endpoint equality alone.
            if self.redundant_tail(now): self.dirty = False; return
            remaining = {v:round((p.end_ns-now)/1e9,6) for v,p in self.latest.items()}
            self.fail('INVALID_COVERAGE',str(exc)+'; remaining seconds='+str(remaining)); return
        self.segment_pub.publish(segment)
        self.client.submit(segment)
        source_end = max(p.end_ns for p in self.latest.values())
        segment_end = nanoseconds(segment.valid_from) + nanoseconds(segment.vehicle_trajectories[0].points[-1].time_from_start)
        if segment_end > source_end:
            self.counts['terminal_hold_segments'] = self.counts.get('terminal_hold_segments',0)+1
        self.submitted = dict(self.latest); self.submitted_epochs = dict(self.goal_epochs)
        self.submitted_final = segment.is_final
        self.counts['segments'] += 1; self.dirty = False

    def redundant_tail(self, now):
        if not self.submitted_final or self.submitted_epochs != self.goal_epochs: return False
        try:
            equivalent = all(equivalent_final_tail(self.submitted[v],self.latest[v],self.goals[v],now,
                float(self.config['limits']['acceleration'])*float(self.config['limits']['tolerance']))
                for v in self.vehicles)
        except (ValueError,ArithmeticError): return False
        if equivalent:self.counts['redundant_final_tails'] = self.counts.get('redundant_final_tails',0)+1
        return equivalent

    def fail(self, reason, detail):
        if self.closed: return
        self.closed = True; self.state = 'FAILED'; self.reason = reason+': '+detail
        self.stop_started = time.monotonic()
        for pub in self.stop_pubs.values(): pub.publish(Empty())
        self.client.cancel()
        self.failure_pub.publish(PlannerFailure(task_id=self.config['task']['task_id'],reason=reason,detail=detail))
        self.get_logger().error(self.reason)

    def cancel(self, request, response):
        self.fail('CANCELED','explicit task cancel; restart with a new task for replanning')
        response.success = True; response.message = 'stop requested; '+self.scope.api('status')+' reports confirmation'
        return response

    def report(self):
        if self.state == 'STOPPING' and self.client.quiescent: self.state = 'SUCCEEDED'
        if self.stop_started and not self.client.quiescent and time.monotonic()-self.stop_started > 5:
            self.state = 'FAILED'; self.reason = 'STOP_UNCONFIRMED: '+self.reason
            self.stop_started = None
        state = {'state':self.state,'reason':self.reason,'counts':self.counts,'rejected':self.rejected,
                 'vehicles':self.vehicles,'interface_scope':self.scope.namespace or 'legacy_fleet',
                 'revision':self.client.revision,'completed':self.finished_vehicles or sorted(self.client.results),
                 'stop_confirmed':self.client.quiescent if self.client.cancel_started else False}
        self.status_pub.publish(String(data=json.dumps(state)))

def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO); node = EgoAdapter()
    stopping = [False]
    def stop(signum,frame): stopping[0] = True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while rclpy.ok() and not stopping[0]:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        if rclpy.ok():
            node.fail('SHUTDOWN','adapter stopping')
            node.client.close()
            deadline = time.monotonic()+2
            while time.monotonic() < deadline and not node.client.quiescent:rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
