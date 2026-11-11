"""Task/score role for isolated FollowReference ACRO AirSim acceptance."""
import collections
import json
import math
import numpy as np
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from nav_msgs.msg import Path as NavigationPath
from sensor_msgs.msg import PointCloud2
from as2_msgs.action import Takeoff, Land, FollowReference
from as2_msgs.msg import PlatformInfo, PlatformStatus, ControllerInfo, ControlMode, Thrust, YawMode
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import VehicleRatesSetpoint, VehicleControlMode, VehicleCommand
from uav_follow_reference_acro.msg import ControlFrame

sys.path.insert(0, '/repo/simulator/airsim/sensors')
from uav_as2_sensor_adapter.rpc import Rpc


class Mission(Node):
    def __init__(self):
        super().__init__('acro_control_validation_task', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.out = Path('/evidence')
        self.task=json.loads((self.out/'forest-task.json').read_text())
        self.pose = self.twist = self.platform = self.controller = None
        self.px4_mode = None
        self.source_ready = False
        self.nav_counts={'odom':0,'cloud':0}; self.path_length=0.; self.cloud_xyz=None;self.detour_seen=False;self.route_points=[];self.max_planned_deviation=0.
        self.create_subscription(PoseStamped,'/drone0/self_localization/pose',lambda m:None,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/drone0/navigation/points',self.on_cloud,qos_profile_sensor_data)
        def path(m):
            self.route_points=[(p.pose.position.x,p.pose.position.y,p.pose.position.z)for p in m.poses]
            self.path_length=sum(math.dist((a.pose.position.x,a.pose.position.y,a.pose.position.z),(b.pose.position.x,b.pose.position.y,b.pose.position.z)) for a,b in zip(m.poses,m.poses[1:]))
        self.create_subscription(NavigationPath,'/drone0/ipc/path',path,1)
        self.action_handle = None; self.phase = 'ground'; self.flying = False
        self.counts = collections.Counter(); self.frames = {}; self.rates = []; self.targets = []
        self.actuator_rates = []; self.actuator_thrust = None
        self.result = {'success': False, 'scope': 'Real IPC forest navigation on PC + AirSim + flight_board'}
        self.log = (self.out/'samples.jsonl').open('w')
        self.rpc = Rpc('127.0.0.1', int(os.environ.get('IPC_TEST_RPC_PORT','24474')), timeout=2)
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
            lambda m: setattr(self, 'pose', m), qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/drone0/self_localization/twist',
            lambda m: setattr(self, 'twist', m), qos_profile_sensor_data)
        self.create_subscription(PlatformInfo, '/drone0/platform/info',
            lambda m: setattr(self, 'platform', m), qos_profile_sensor_data)
        self.create_subscription(ControllerInfo, '/drone0/controller/info',
            lambda m: setattr(self, 'controller', m), qos_profile_sensor_data)
        self.create_subscription(VehicleControlMode, '/drone0/fmu/out/vehicle_control_mode',
            lambda m: setattr(self, 'px4_mode', m), qos_profile_sensor_data)
        self.create_subscription(Bool, '/drone0/ipc/ready',
            lambda m: setattr(self, 'source_ready', m.data), 1)
        for topic, kind in [('external_acro/control', ControlFrame),
                            ('motion_reference/thrust', Thrust),
                            ('actuator_command/thrust', Thrust),
                            ('actuator_command/twist', TwistStamped),
                            ('fmu/in/vehicle_rates_setpoint', VehicleRatesSetpoint)]:
            self.create_subscription(kind, '/drone0/'+topic,
                lambda m, t=topic: self.sample(t, m), QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_cloud(self,m):
        self.nav_counts['cloud']+=1
        if m.header.frame_id!='earth' or m.is_bigendian or m.row_step!=m.width*m.point_step:return
        offsets={f.name:f.offset for f in m.fields}
        if not all(k in offsets for k in ('x','y','z')):return
        data=np.ndarray((m.width*m.height,),dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k]for k in ('x','y','z')],'itemsize':m.point_step}),buffer=m.data)
        points=np.column_stack([data[k]for k in ('x','y','z')]);self.cloud_xyz=points[np.isfinite(points).all(axis=1)]

    def sample(self, topic, msg):
        if self.phase not in ('probe', 'acro', 'loss', 'stopping'):
            return
        self.counts[topic] += 1
        rec = {'topic': topic, 'wall': time.monotonic(), 'sim': self.get_clock().now().nanoseconds/1e9}
        if hasattr(msg, 'header'):
            rec['stamp_ns'] = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if isinstance(msg, VehicleRatesSetpoint):
            rec.update(rate=[msg.roll, msg.pitch, msg.yaw], thrust=list(map(float,msg.thrust_body)))
            self.rates.append(rec)
        elif isinstance(msg, ControlFrame):
            rec.update(rate=[msg.body_rates.x, msg.body_rates.y, msg.body_rates.z],
                       thrust=msg.thrust_newtons, sequence=msg.sequence, solver_ok=msg.solver_ok, last_successful_solve_ns=msg.last_successful_solve.sec*1000000000+msg.last_successful_solve.nanosec)
        elif isinstance(msg, TwistStamped):
            rec['rate'] = [msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z]
            self.actuator_rates.append(rec)
        else:
            rec['thrust'] = msg.thrust
            if topic == 'actuator_command/thrust': self.actuator_thrust = msg.thrust
        self.log.write(json.dumps(rec)+'\n'); self.log.flush()

    def wait(self, predicate, timeout=20):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline: raise TimeoutError(self.phase)
            if self.phase not in ('stopping','landing') and (self.out/'compute-error.json').exists():
                raise RuntimeError('compute runtime child exited')
            rclpy.spin_once(self, timeout_sec=.01)

    def delay(self, duration):
        until = time.monotonic()+duration
        self.wait(lambda: time.monotonic()>=until, duration+5)

    def service(self, name, value):
        cli = self.create_client(SetBool, '/drone0/'+name)
        self.wait(cli.service_is_ready)
        future = cli.call_async(SetBool.Request(data=value)); self.wait(future.done)
        if not future.result().success: raise RuntimeError(name+' rejected')

    def action(self, kind, name, goal, field):
        cli = ActionClient(self, kind, '/drone0/'+name); self.wait(cli.server_is_ready)
        sent = cli.send_goal_async(goal); self.wait(sent.done)
        handle = sent.result()
        if not handle.accepted: raise RuntimeError(name+' goal rejected')
        future = handle.get_result_async(); self.wait(future.done, 100)
        if not getattr(future.result().result, field): raise RuntimeError(name+' failed')

    def position(self):
        p = self.pose.pose.position
        return [p.x, p.y, p.z]

    def preflight_probe(self):
        self.phase = 'probe'
        cli = self.create_client(SetControlMode, '/drone0/controller/set_control_mode')
        self.wait(cli.service_is_ready)
        request = SetControlMode.Request()
        request.control_mode.control_mode = ControlMode.BODY_RATES
        request.control_mode.yaw_mode = ControlMode.YAW_SPEED
        future = cli.call_async(request); self.wait(future.done)
        if not future.result().success: raise RuntimeError('ground BODY_RATES negotiation failed')
        rates_pub = self.create_publisher(TwistStamped, '/drone0/motion_reference/twist', qos_profile_sensor_data)
        thrust_pub = self.create_publisher(Thrust, '/drone0/motion_reference/thrust', qos_profile_sensor_data)
        self.wait(lambda: rates_pub.get_subscription_count()>0 and thrust_pub.get_subscription_count()>0)
        def publish():
            stamp = self.get_clock().now().to_msg()
            twist = TwistStamped(); twist.header.stamp = stamp; twist.header.frame_id = 'drone0/base_link'
            twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = .1, -.2, .3
            thrust = Thrust(); thrust.header = twist.header; thrust.thrust = 3.0
            rates_pub.publish(twist); thrust_pub.publish(thrust)
        timer = self.create_timer(.02, publish)
        try:
            self.delay(.2)
            if self.actuator_rates or self.actuator_thrust is not None:
                raise RuntimeError('unarmed controller unexpectedly emitted actuators')
            self.result['unarmed_output_suppressed'] = True
            if self.platform.armed: raise RuntimeError('probe unexpectedly armed')
        finally:
            self.destroy_timer(timer)
            self.destroy_publisher(rates_pub); self.destroy_publisher(thrust_pub)
        # Leave the ground probe in BODY_RATES: Offboard initialization uses
        # the same mode. Native Takeoff will negotiate POSITION after PX4 confirms Offboard.
        self.counts.clear(); self.rates.clear()

    def run(self):
        try:
            self.wait(lambda: self.pose and self.platform, 60)
            self.wait(lambda: (self.out/'preflight.json').exists())
            self.result['preflight'] = json.loads((self.out/'preflight.json').read_text())
            if not self.result['preflight']['checks_pass']: raise RuntimeError('PX4 preflight not passed')
            if 'target_from_ground_enu_m' in self.task:
                settled_at=self.get_clock().now().nanoseconds/1e9
                self.wait(lambda:self.get_clock().now().nanoseconds/1e9-settled_at>=10.,40)
            self.result['initial_position'] = self.position()
            ground_origin=self.position()
            ground_truth=self.rpc.call('simGetGroundTruthKinematics','PX4')['position']
            self.result['task_ground_origin_enu']=ground_origin
            self.result['task_ground_origin_truth_ned']=ground_truth
            self.result['collision_start'] = self.rpc.call('simGetCollisionInfo', 'PX4')
            self.wait(lambda:self.nav_counts['cloud']>=3 and self.cloud_xyz is not None and len(self.cloud_xyz)>0,30)
            self.preflight_probe()
            self.phase = 'arming'; self.service('set_arming_state', True); self.flying = True
            self.wait(lambda: self.platform.armed and self.px4_mode and self.px4_mode.flag_armed)
            self.service('set_offboard_mode', True)
            self.wait(lambda: self.platform.offboard and self.px4_mode.flag_control_offboard_enabled)
            self.delay(.2)
            self.phase = 'takeoff'
            self.action(Takeoff, 'TakeoffBehavior', Takeoff.Goal(takeoff_height=float(self.task['takeoff_height_m']), takeoff_speed=.5), 'takeoff_success')
            self.wait(lambda: self.platform.status.state == PlatformStatus.FLYING)
            self.delay(3)
            self.out.joinpath('need-hover').touch()
            self.wait(lambda: (self.out/'hover.json').exists(), 30)
            self.result['hover'] = json.loads((self.out/'hover.json').read_text())
            self.wait(lambda: self.source_ready and
                self.count_subscribers('/drone0/external_acro/session') >= 1 and
                self.count_publishers('/drone0/external_acro/control') == 1, 15)
            goal = FollowReference.Goal()
            goal.target_pose.header.frame_id = self.pose.header.frame_id
            origin=self.position()
            offset=self.task.get('target_from_ground_enu_m',self.task.get('target_offset_enu_m'))
            target_origin=ground_origin if 'target_from_ground_enu_m' in self.task else origin
            p=[a+b for a,b in zip(target_origin,offset)]
            goal.target_pose.point.x, goal.target_pose.point.y, goal.target_pose.point.z = p
            q = self.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            goal.yaw.mode = YawMode.FIXED_YAW; goal.yaw.angle = yaw+.15
            goal.max_speed_x = goal.max_speed_y = goal.max_speed_z = float(self.task['speed_m_s'])
            self.result['target'] = p
            self.result['navigation_origin'] = origin
            truth_start = self.rpc.call('simGetGroundTruthKinematics', 'PX4')
            self.result['truth_start'] = truth_start
            tp=ground_truth if 'target_from_ground_enu_m' in self.task else truth_start['position']
            truth_goal=[tp['x_val']+offset[1],tp['y_val']+offset[0],tp['z_val']-offset[2]]
            self.result['truth_goal_ned']=truth_goal
            cli = ActionClient(self, FollowReference, '/drone0/FollowReferenceBehavior')
            self.wait(cli.server_is_ready)
            self.phase = 'acro'
            future = cli.send_goal_async(goal); self.wait(future.done)
            self.action_handle = future.result()
            if not self.action_handle.accepted: raise RuntimeError('ACRO goal rejected')
            self.out.joinpath('acro-active').touch()
            run_result = self.action_handle.get_result_async()
            self.wait(lambda: self.controller.input_control_mode.control_mode == ControlMode.BODY_RATES and
                self.px4_mode.flag_control_rates_enabled and not self.px4_mode.flag_control_attitude_enabled and
                not self.px4_mode.flag_control_velocity_enabled and self.counts['fmu/in/vehicle_rates_setpoint'] >= 5, 5)
            self.out.joinpath('acro-established').touch()
            start_sim=self.get_clock().now().nanoseconds/1e9; start_wall=time.monotonic(); settled=None
            minimum_clearance=float('inf');straight_blocked=False
            previous_position=origin;previous_sim=start_sim;travelled=0.;speed_samples=[];cruise_seconds=0.;height_min=height_max=origin[2]
            max_tilt=0.; closest_goal=float('inf'); path_seen=False;max_speed=0.;max_cross_track=0.;truth_error=float('inf');score_at=0.
            while True:
                self.delay(.03)
                if run_result.done():raise RuntimeError('IPC execution ended before arrival')
                sim=self.get_clock().now().nanoseconds/1e9
                if sim-start_sim>self.task['timeout_sim_s'] or time.monotonic()-start_wall>self.task.get('timeout_wall_s',300):raise TimeoutError('forest goal')
                error=math.dist(self.position(),p);closest_goal=min(closest_goal,error)
                v=self.twist.twist.linear;speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
                q=self.pose.pose.orientation;tilt=math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y))))
                max_tilt=max(max_tilt,tilt);max_speed=max(max_speed,speed)
                position=self.position()
                dt=max(0.,sim-previous_sim);travelled+=math.dist(position,previous_position)
                speed_samples.append(speed)
                if speed>=self.task.get('cruise_min_m_s',float('inf')):cruise_seconds+=dt
                height_min=min(height_min,position[2]);height_max=max(height_max,position[2])
                previous_position=position;previous_sim=sim
                self.result.update(navigation_sim_s=sim-start_sim,travelled_m=travelled,trajectory_average_m_s=travelled/max(1e-6,sim-start_sim),speed_percentiles_m_s=np.percentile(speed_samples,[10,50,90,95,99]).tolist(),cruise_above_threshold_sim_s=cruise_seconds,height_range_enu_m=[height_min,height_max],planar_path=self.task['planar_path'])
                if 'bounds_enu_min_m' in self.task:
                    local=[v-a for v,a in zip(position,ground_origin)]
                    if any(v<lo or v>hi for v,lo,hi in zip(local,self.task['bounds_enu_min_m'],self.task['bounds_enu_max_m'])):raise RuntimeError('task spatial bounds')
                delta=[b-a for a,b in zip(origin,p)];alpha=max(0.,min(1.,sum((v-a)*d for v,a,d in zip(position,origin,delta))/sum(d*d for d in delta)))
                cross_track=math.dist(position,[a+alpha*d for a,d in zip(origin,delta)]);max_cross_track=max(max_cross_track,cross_track)
                self.result.update(closest_goal_m=closest_goal,max_tilt_rad=max_tilt,max_speed_norm_mps=max_speed,max_cross_track_m=max_cross_track)
                self.log.write(json.dumps({'topic':'navigation_state','sim':sim,'p':position,'goal_error_m':error,'speed_mps':speed,'tilt_rad':tilt})+'\n')
                if cross_track>self.task['max_cross_track_m']:raise RuntimeError('forest cross-track bound')
                if tilt>self.task['max_tilt_rad']:raise RuntimeError('forest tilt bound')
                path_seen=path_seen or self.path_length>0
                if self.cloud_xyz is not None and len(self.cloud_xyz):
                    current=np.asarray(self.position());minimum_clearance=min(minimum_clearance,float(np.linalg.norm(self.cloud_xyz-current,axis=1).min()))
                    a=np.asarray(origin);d=np.asarray(p)-a;t=np.clip(((self.cloud_xyz[:,0]-a[0])*d[0]+(self.cloud_xyz[:,1]-a[1])*d[1]+(self.cloud_xyz[:,2]-a[2])*d[2])/(d@d),0,1)
                    straight_blocked=straight_blocked or bool(np.any(np.linalg.norm(self.cloud_xyz-(a+t[:,None]*d),axis=1)<.35))
                    for point in self.route_points:
                        alpha=max(0.,min(1.,sum((v-a)*d for v,a,d in zip(point,origin,delta))/sum(d*d for d in delta)))
                        deviation=math.dist(point,[a+alpha*d for a,d in zip(origin,delta)])
                        self.max_planned_deviation=max(self.max_planned_deviation,deviation)
                    self.detour_seen=self.max_planned_deviation>=self.task.get('minimum_planned_deviation_m',.35)
                    self.result.update(minimum_clearance_m=minimum_clearance,straight_blocked=straight_blocked,detour_seen=self.detour_seen,max_planned_deviation_m=self.max_planned_deviation)
                    if minimum_clearance<self.task['minimum_clearance_m']:raise RuntimeError('forest clearance bound')
                if time.monotonic()>=score_at:
                    truth=self.rpc.call('simGetGroundTruthKinematics','PX4');tp=truth['position'];truth_p=[tp['x_val'],tp['y_val'],tp['z_val']]
                    truth_error=math.dist(truth_p,truth_goal);self.result['truth_goal_error_m']=truth_error
                    self.log.write(json.dumps({'topic':'airsim_truth','sim':sim,'p_ned':truth_p})+'\n')
                    collision=self.rpc.call('simGetCollisionInfo','PX4')
                    if collision['time_stamp'] and collision['time_stamp']!=self.result['collision_start']['time_stamp']:raise RuntimeError('forest collision')
                    score_at=time.monotonic()+.1
                if error<=self.task['goal_tolerance_m'] and truth_error<=self.task['goal_tolerance_m'] and speed<=self.task['goal_speed_m_s']:
                    if settled is None:settled=sim
                    if sim-settled>=self.task['dwell_sim_s']:break
                else:settled=None
            if not path_seen:raise RuntimeError('no IPC path evidence')
            if cruise_seconds<self.task.get('cruise_required_sim_s',0):raise RuntimeError('nominal cruise speed was not physically reached for required duration')
            if self.task.get('require_detour',True) and (not straight_blocked or not self.detour_seen):raise RuntimeError('no blocked straight route/detour evidence')
            self.result.update(minimum_clearance_m=minimum_clearance,straight_blocked=straight_blocked,detour_seen=self.detour_seen,max_planned_deviation_m=self.max_planned_deviation)
            self.result.update(goal_error_m=error,closest_goal_m=closest_goal,max_tilt_rad=max_tilt,navigation_passed=True)
            self.phase = 'stopping'
            result = self.action_handle.get_result_async()
            canceled = self.action_handle.cancel_goal_async(); self.wait(canceled.done,5)
            if not canceled.result().goals_canceling: raise RuntimeError('cancel not accepted')
            self.wait(result.done,5); self.action_handle=None
            self.wait(lambda: self.controller.input_control_mode.control_mode == ControlMode.POSITION, 10)
            stop_origin=self.position();stop_deadline=time.monotonic()+self.task['stop_timeout_wall_s'];stable_since=None;stop_drift=0.
            while True:
                self.delay(.03)
                v=self.twist.twist.linear;speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
                stop_drift=max(stop_drift,math.dist(self.position(),stop_origin))
                q=self.pose.pose.orientation;tilt=math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y))))
                truth=self.rpc.call('simGetGroundTruthKinematics','PX4')['position']
                truth_stop_error=math.dist([truth['x_val'],truth['y_val'],truth['z_val']],truth_goal)
                estimated_stop_error=math.dist(self.position(),p)
                self.result.update(stop_max_drift_m=stop_drift,stop_speed_mps=speed,truth_stop_goal_error_m=truth_stop_error)
                self.log.write(json.dumps({'topic':'stopping_state','wall':time.monotonic(),'sim':self.get_clock().now().nanoseconds/1e9,
                    'p':self.position(),'goal_error_m':estimated_stop_error,'truth_goal_error_m':truth_stop_error,
                    'speed_mps':speed,'tilt_rad':tilt,'drift_m':stop_drift})+'\n')
                if stop_drift>self.task['max_stop_drift_m'] or tilt>self.task['max_tilt_rad']:raise RuntimeError('hold handoff bound')
                if (speed<=self.task['stop_speed_m_s'] and estimated_stop_error<=self.task['goal_tolerance_m'] and
                    truth_stop_error<=self.task['goal_tolerance_m']):
                    if stable_since is None:stable_since=time.monotonic()
                    if time.monotonic()-stable_since>=self.task['stop_dwell_wall_s']:break
                else:stable_since=None
                if time.monotonic()>stop_deadline:raise RuntimeError('stop not confirmed')
            self.result['stop_speed_mps'] = speed
            self.result['collision_end'] = self.rpc.call('simGetCollisionInfo', 'PX4')
            if (self.result['collision_end']['time_stamp'] and
                self.result['collision_end']['time_stamp'] != self.result['collision_start']['time_stamp']):
                raise RuntimeError('new AirSim collision')
            self.result['control_passed'] = True
        except BaseException as exc:
            self.result['error'] = str(exc)
        finally:
            self.phase = 'stopping'
            if self.action_handle and self.action_handle.accepted:
                try:
                    canceled = self.action_handle.cancel_goal_async(); self.wait(canceled.done, 5)
                except BaseException as exc: self.result['cancel_error'] = str(exc)
            if self.flying:
                self.phase = 'landing'
                try:
                    self.action(Land,'LandBehavior',Land.Goal(land_speed=.5),'land_success')
                    self.wait(lambda: not self.platform.armed, 40)
                    self.result['landed_and_disarmed'] = True
                except BaseException as exc:
                    self.result['recovery_error'] = str(exc)
                    try:
                        pub = self.create_publisher(VehicleCommand,'/drone0/fmu/in/vehicle_command',qos_profile_sensor_data)
                        self.wait(lambda: pub.get_subscription_count()>0)
                        pub.publish(VehicleCommand(timestamp=time.time_ns()//1000, command=21,
                            target_system=73,target_component=1,source_system=255,source_component=1,
                            from_external=True,param4=float('nan'),param5=float('nan'),param6=float('nan'),param7=float('nan')))
                        self.wait(lambda: not self.platform.armed, 100)
                        self.result['landed_and_disarmed'] = True
                    except BaseException as failure: self.result['emergency_error'] = str(failure)
            else:
                self.result['landed_and_disarmed'] = bool(self.platform and not self.platform.armed)
            self.result['success'] = bool(self.result.get('control_passed') and
                self.result.get('landed_and_disarmed') and not self.result.get('error') and
                not self.result.get('recovery_error'))
            (self.out/'result.json').write_text(json.dumps(self.result, indent=2)+'\n')
            self.log.close()
        return 0 if self.result['success'] else 1


def main():
    rclpy.init(); node = Mission()
    try: code = node.run()
    finally: node.destroy_node(); rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__': main()
