"""SUPER committed polynomial -> existing FollowTimedPath execution interface."""
import copy
import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from mars_quadrotor_msgs.msg import PolynomialTrajectory, PositionCommand
from std_msgs.msg import String
from std_srvs.srv import SetBool
from uav_planner_interfaces.msg import TimedTrajectorySegment, VehicleTrajectory, TimedTrajectoryPoint
from uav_planner_framework.follow_path_client import FleetFollowPathClient
from uav_planner_framework.ros_conversions import seconds_to_time, time_to_seconds
from .polynomial import Polynomial, InsufficientCoverage


class SuperAdapter(Node):
    def __init__(self):
        super().__init__('super_timed_path_adapter')
        for key,value in [('vehicle_id','drone0'),('frame_id','drone0/odom'),
                          ('goal',[0.,0.,2.]),('goal_tolerance',1.5),('max_speed',8.),('lead_time',.1)]:
            self.declare_parameter(key,value)
        self.vehicle = self.get_parameter('vehicle_id').value
        self.frame = self.get_parameter('frame_id').value
        self.goal = self.get_parameter('goal').value
        if len(self.goal) != 3 or not all(math.isfinite(x) for x in self.goal):
            raise ValueError('invalid local ENU goal')
        self.config = {'vehicles':[self.vehicle], 'deployment':{self.vehicle:{
            'namespace':self.vehicle,'px4_origin_in_fleet_ned':[0.,0.,0.]}},
            'planner':{'parameters':{'max_velocity':self.get_parameter('max_speed').value}}}
        self.client = None
        self.enabled = False
        self.error = ''
        self.queued = None
        self.last_plan = 0.
        self.last_command = 0.
        self.last_wall = 0.
        self.pose_stamp = 0.
        self.started = 0.
        self.flags = {}
        self.task = ''
        self.late_updates = 0
        self.last_curve = {}
        self.create_subscription(Odometry,'sensor_measurements/odom',self.on_pose,qos_profile_sensor_data)
        self.create_subscription(PolynomialTrajectory,'/planning_cmd/poly_traj',self.on_plan,qos_profile_sensor_data)
        self.create_subscription(PositionCommand,'/planning/pos_cmd',self.on_command,qos_profile_sensor_data)
        self.create_service(SetBool,'super_adapter/enable',self.enable)
        self.publisher = self.create_publisher(String,'super_adapter/state',10)
        self.create_timer(.02,self.tick)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_pose(self,msg):
        p = msg.pose.pose.position
        if msg.header.frame_id == self.frame and all(math.isfinite(x) for x in (p.x,p.y,p.z)):
            self.pose_stamp = time_to_seconds(msg.header.stamp)

    def on_command(self,msg):
        if not self.enabled:
            return
        stamp = time_to_seconds(msg.header.stamp)
        if msg.header.frame_id != self.frame or not -.05 <= self.now_seconds()-stamp <= .3 or stamp <= self.last_command:
            return
        if not all(math.isfinite(v) for vector in (msg.position,msg.velocity,msg.acceleration)
                   for v in (vector.x,vector.y,vector.z)):
            self.fail('SUPER command PVA is nonfinite')
            return
        if msg.trajectory_flag not in (1,2):
            self.fail('SUPER has no executable primary/backup command')
            return
        self.last_command = stamp
        self.last_wall = time.monotonic()
        key = str(msg.trajectory_flag)
        self.flags[key] = self.flags.get(key,0)+1

    def on_plan(self,msg):
        if not self.enabled:
            return
        if msg.type & PolynomialTrajectory.EMER_STOP:
            self.fail('SUPER emergency stop')
            return
        if not msg.type & PolynomialTrajectory.POSITION_TRAJ:
            return  # Heartbeats never extend or regenerate a cached trajectory.
        stamp = time_to_seconds(msg.header.stamp)
        if msg.header.frame_id != self.frame or not -.05 <= self.now_seconds()-stamp <= .3 or stamp <= self.last_plan:
            return
        self.last_plan = stamp
        self.queued = copy.deepcopy(msg)

    def enable(self,request,response):
        if not request.data:
            self.enabled = False
            self.queued = None
            if self.client:
                self.client.close()
            response.success = True
        elif self.enabled:
            response.success = True
        elif self.client and not self.client.quiescent:
            response.success = False
            response.message = 'Previous execution has not stopped'
        elif not 0 < self.pose_stamp or not -.05 <= self.now_seconds()-self.pose_stamp <= .5:
            response.success = False
            response.message = 'Fresh local odometry required'
        else:
            self.task = str(uuid.uuid4())
            self.client = FleetFollowPathClient(self,self.config,self.fail)
            self.enabled = True
            self.error = ''
            self.queued = None
            self.last_plan = self.last_command = self.last_wall = 0.
            self.started = time.monotonic()
            response.success = True
        return response

    def fail(self,reason,detail=''):
        self.error = reason+(':'+detail if detail else '')
        self.get_logger().error(self.error)
        self.enabled = False
        self.queued = None
        if self.client:
            self.client.close()

    def convert(self,msg):
        curve = Polynomial(msg.start_wt_pos,msg.time_pos,
                           [msg.coef_pos_x,msg.coef_pos_y,msg.coef_pos_z],msg.order_pos)
        if msg.piece_num_pos != len(curve.durations):
            raise ValueError('piece count mismatch')
        start = max(self.now_seconds()+self.get_parameter('lead_time').value,curve.start)
        segment = TimedTrajectorySegment(interface_version=1,task_id=self.task)
        segment.generated_at = copy.deepcopy(msg.header.stamp)
        seconds_to_time(start,segment.valid_from)
        endpoint,velocity,_ = curve.sample(curve.end)
        segment.is_final = math.dist(endpoint,self.goal) <= self.get_parameter('goal_tolerance').value and math.sqrt(sum(v*v for v in velocity)) <= .3
        self.last_curve = {'source_start':curve.start,'source_end':curve.end,
                           'submitted_from':start,'endpoint':endpoint,'end_velocity':velocity,
                           'is_final':segment.is_final,'pieces':len(curve.durations)}
        trajectory = VehicleTrajectory(vehicle_id=self.vehicle,valid_fields=3)
        for absolute in curve.times(start):
            point = TimedTrajectoryPoint()
            seconds_to_time(absolute-start,point.time_from_start)
            for name,vector in zip(('position','velocity','acceleration'),curve.sample(absolute)):
                target = getattr(point,name)
                target.x,target.y,target.z = vector[1],vector[0],-vector[2]
            trajectory.points.append(point)
        segment.vehicle_trajectories = [trajectory]
        return segment

    def tick(self):
        if self.client:
            self.client.tick(self.now_seconds())
        if self.enabled and self.client.complete:
            self.enabled = False
            self.queued = None
        if self.enabled:
            if self.last_wall and time.monotonic()-self.last_wall > .5:
                self.fail('SUPER command stream expired')
            elif self.client.revision == 0 and time.monotonic()-self.started > 10:
                self.fail('SUPER initial planning timeout')
            elif self.queued is not None and self.client.ready and not self.client.pending and time_to_seconds(self.queued.header.stamp) <= self.now_seconds():
                msg,self.queued = self.queued,None
                try:
                    self.client.submit(self.convert(msg))
                except InsufficientCoverage:
                    # Keep the currently accepted plan. Its original expiry and
                    # final/result semantics remain authoritative.
                    self.late_updates += 1
                except (ValueError,OverflowError) as exc:
                    self.fail('Invalid committed polynomial',str(exc))
        self.publisher.publish(String(data=json.dumps({
            'enabled':self.enabled,'error':self.error,'task_id':self.task,
            'revision':self.client.revision if self.client else 0,
            'complete':self.client.complete if self.client else False,
            'settling':self.vehicle in self.client.settling_vehicles if self.client else False,
            'stop_confirmed':self.client.quiescent if self.client and self.client.cancel_started else False,
            'source_flags':self.flags,'late_updates':self.late_updates,'last_curve':self.last_curve})))


def main(args=None):
    rclpy.init(args=args)
    node = SuperAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node.client and rclpy.ok():
            node.client.close()
        node.destroy_node()
        rclpy.try_shutdown()
