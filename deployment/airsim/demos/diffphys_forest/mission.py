"""Task/score role for isolated FollowReference ACRO AirSim acceptance."""
import collections
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from as2_msgs.action import Takeoff, Land, FollowReference, GoToWaypoint
from as2_msgs.msg import PlatformInfo, PlatformStatus, ControllerInfo, ControlMode, Thrust, YawMode
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import VehicleRatesSetpoint, VehicleAttitudeSetpoint, VehicleControlMode, VehicleCommand
from uav_follow_reference_acro.msg import ControlFrame

sys.path.insert(0, '/repo/simulator/airsim/sensors')
from uav_as2_sensor_adapter.rpc import Rpc


class Mission(Node):
    def __init__(self):
        super().__init__('acro_control_validation_task', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.out = Path('/evidence')
        self.pose = self.twist = self.platform = self.controller = None
        self.px4_mode = None
        self.source_ready = False
        self.action_handle = None; self.terminal_handle = None; self.phase = 'ground'; self.flying = False
        self.counts = collections.Counter(); self.frames = {}; self.rates = []; self.targets = []
        self.actuator_rates = []; self.actuator_thrust = None
        self.result = {'success': False, 'scope': 'DiffPhys '+os.environ.get('DIFFPHYS_PROFILE','swarm')+' -> ROS Inference -> ATTITUDE Behavior -> AS2/PX4, forest'}
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
        self.create_subscription(Bool, '/drone0/diffphys/ready',
            lambda m: setattr(self, 'source_ready', m.data), 1)
        for topic, kind in [('external_acro/control', ControlFrame),
                            ('motion_reference/thrust', Thrust),
                            ('actuator_command/thrust', Thrust),
                            ('actuator_command/pose', PoseStamped),
                            ('fmu/in/vehicle_attitude_setpoint_v1', VehicleAttitudeSetpoint)]:
            self.create_subscription(kind, '/drone0/'+topic,
                lambda m, t=topic: self.sample(t, m), qos_profile_sensor_data)

    def sample(self, topic, msg):
        if self.phase not in ('probe', 'acro', 'loss', 'handover', 'terminal', 'stopping'):
            return
        self.counts[topic] += 1
        rec = {'topic': topic, 'wall': time.monotonic(), 'sim': self.get_clock().now().nanoseconds/1e9}
        if hasattr(msg, 'header'):
            rec['stamp_ns'] = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if isinstance(msg, VehicleAttitudeSetpoint):
            rec.update(q=list(map(float,msg.q_d)), thrust=list(map(float,msg.thrust_body)))
        elif isinstance(msg, VehicleRatesSetpoint):
            rec.update(rate=[msg.roll, msg.pitch, msg.yaw], thrust=list(map(float,msg.thrust_body)))
            self.rates.append(rec)
        elif isinstance(msg, ControlFrame):
            rec.update(rate=[msg.body_rates.x, msg.body_rates.y, msg.body_rates.z],
                       q=[msg.attitude.w,msg.attitude.x,msg.attitude.y,msg.attitude.z], attitude_frame=msg.attitude_frame_id, control_mode=msg.control_mode,
                       thrust=msg.thrust_newtons, sequence=msg.sequence, solver_ok=msg.solver_ok, last_successful_solve_ns=msg.last_successful_solve.sec*1000000000+msg.last_successful_solve.nanosec)
        elif isinstance(msg, PoseStamped):
            rec['q']=[msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z]
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

    def confirm_execution_stop(self):
        """Acknowledge execution shutdown after arrival, separately from braking."""
        self.phase='stopping'
        started=time.monotonic();deadline=started+10.
        def await_done(future):
            self.wait(future.done,max(0.,deadline-time.monotonic()))
            return future.result()
        for handle in (self.terminal_handle,self.action_handle):
            if not handle or not handle.accepted:continue
            finished=handle.get_result_async()
            if not finished.done():
                canceled=await_done(handle.cancel_goal_async())
                # A completed goal rejects cancel, while its result callback may
                # still be queued. Await the terminal result within the same budget.
                if not canceled.goals_canceling:
                    self.result['completed_or_rejected_cancel_observed']=True
            await_done(finished)
        self.wait(lambda:self.controller.input_control_mode.control_mode==ControlMode.POSITION,
            max(0.,deadline-time.monotonic()))
        count=self.counts['external_acro/control']
        self.delay(.25)
        if self.counts['external_acro/control']!=count:
            raise RuntimeError('policy output continued after execution stop')
        if time.monotonic()>deadline:raise RuntimeError('execution stop acknowledgement timed out')
        self.result.update(execution_stop_confirmed=True,
            execution_stop_wall_s=time.monotonic()-started)

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
            self.result['initial_position'] = self.position()
            self.result['collision_start'] = self.rpc.call('simGetCollisionInfo', 'PX4')
            self.preflight_probe()
            self.phase = 'arming'; self.service('set_arming_state', True); self.flying = True
            self.wait(lambda: self.platform.armed and self.px4_mode and self.px4_mode.flag_armed)
            self.service('set_offboard_mode', True)
            self.wait(lambda: self.platform.offboard and self.px4_mode.flag_control_offboard_enabled)
            self.delay(.2)
            self.phase = 'takeoff'
            takeoff_height=float(os.environ.get('DIFFPHYS_TAKEOFF_HEIGHT','2.'))
            if not math.isfinite(takeoff_height) or takeoff_height<=0:
                raise ValueError('invalid DiffPhys takeoff height')
            self.result['takeoff_height_m']=takeoff_height
            self.action(Takeoff, 'TakeoffBehavior', Takeoff.Goal(takeoff_height=takeoff_height, takeoff_speed=.5), 'takeoff_success')
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
            p = self.position(); goal.target_pose.point.x, goal.target_pose.point.y, goal.target_pose.point.z = p
            self.result['policy_start_position_m']=p
            long_case=os.environ.get('DIFFPHYS_LONG')=='1'
            task=json.loads(Path('/repo/deployment/airsim/demos/super_forest/task.json').read_text())
            if long_case:
                goal.target_pose.point.x,goal.target_pose.point.y,goal.target_pose.point.z=task['waypoints_enu_m'][0]
            else:goal.target_pose.point.y += 15.
            self.result['case']='super-forest-route' if long_case else 'short'
            self.result['policy_profile']=os.environ.get('DIFFPHYS_PROFILE','swarm')
            self.result['nominal_policy_speed_m_s']=float(os.environ.get('DIFFPHYS_SPEED','2.5'))
            self.result['super_nominal_speed_m_s']=task['speed_m_s']
            # Restore the original 0.6rad gate for new acceptance runs. Historical
            # 0.9rad diagnostic runs retain their original recorded thresholds.
            tilt_limit=float(os.environ.get('DIFFPHYS_TILT_LIMIT','.6'))
            self.result['tilt_limit_rad']=tilt_limit
            navigation_budget=task['leg_timeout_s'] if long_case else 45.
            observed_speeds=[];observed_path=0.;previous_position=p;boundary_violations=0
            sim_begin=self.get_clock().now().nanoseconds/1e9
            q = self.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            goal.yaw.mode = YawMode.YAW_FROM_TOPIC; goal.yaw.angle = yaw
            session_speed=max(3.,float(os.environ.get('DIFFPHYS_SPEED','2.5')))
            goal.max_speed_x = goal.max_speed_y = goal.max_speed_z = session_speed
            target=[goal.target_pose.point.x,goal.target_pose.point.y,goal.target_pose.point.z]
            self.result['target'] = target
            truth_start = self.rpc.call('simGetGroundTruthKinematics', 'PX4')
            self.result['truth_start'] = truth_start
            cli = ActionClient(self, FollowReference, '/drone0/FollowReferenceBehavior')
            self.wait(cli.server_is_ready)
            self.phase = 'acro'
            future = cli.send_goal_async(goal); self.wait(future.done)
            self.action_handle = future.result()
            if not self.action_handle.accepted: raise RuntimeError('ACRO goal rejected')
            self.out.joinpath('acro-active').touch()
            run_result = self.action_handle.get_result_async()
            self.wait(lambda: self.controller.input_control_mode.control_mode == ControlMode.ATTITUDE and
                self.px4_mode.flag_control_rates_enabled and self.px4_mode.flag_control_attitude_enabled and
                not self.px4_mode.flag_control_velocity_enabled and self.counts['fmu/in/vehicle_attitude_setpoint_v1'] >= 5, 5)
            self.out.joinpath('acro-established').touch()
            self.result['goal_lookahead_m']=float(os.environ.get('DIFFPHYS_GOAL_LOOKAHEAD','0'))
            self.result['command_tilt_limit_rad']=float(os.environ.get('DIFFPHYS_COMMAND_TILT','0'))
            start = time.monotonic(); max_error = 0.; max_tilt = 0.
            truth_error = 0.; truth_at = 0.
            fault=os.environ.get('DIFFPHYS_FAULT','none');self.result['fault_scenario']=fault
            goal_hold_started=None;goal_hold_sim_s=0.
            navigation_sim_budget=task['leg_timeout_s'] if long_case else 45.
            wall_deadline=start+(task['mission_timeout_s'] if long_case else navigation_budget*2)
            self.result['navigation_sim_budget_s']=navigation_sim_budget
            # Like SUPER's Mission.wait, the arrival budget includes settling and
            # uses monotonic wall time. The separate simulator watchdog is retained.
            arrival_deadline=min(start+navigation_budget,wall_deadline)
            self.result.update(stop_contract_version=2,arrival_wall_budget_s=navigation_budget,
                execution_stop_budget_wall_s=10.,required_low_speed_dwell_sim_s=2.,
                required_stop_speed_m_s=.3)
            while self.get_clock().now().nanoseconds/1e9-sim_begin<navigation_sim_budget and time.monotonic()<wall_deadline:
                self.delay(.03)
                if fault=='camera' and time.monotonic()-start>=2.:
                    self.out.joinpath('inject-camera-loss').touch()
                    self.wait(run_result.done,5)
                    if run_result.result().result.follow_reference_success:raise RuntimeError('fault did not fail execution')
                    self.result['expected_fault']=True
                    break
                if run_result.done(): raise RuntimeError('ACRO ended before hold interval')
                current_position=self.position()
                observed_path+=math.dist(current_position,previous_position);previous_position=current_position
                v=self.twist.twist.linear;speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z);observed_speeds.append(speed)
                if long_case and any(x<lo or x>hi for x,lo,hi in zip(current_position,task['bounds_enu_min_m'],task['bounds_enu_max_m'])):
                    self.result.update(observed_path_m=observed_path,boundary_violations=boundary_violations+1,
                        observed_speed_median_m_s=statistics.median(observed_speeds) if observed_speeds else None,
                        observed_speed_peak_m_s=max(observed_speeds,default=0),failure_position=current_position)
                    boundary_violations+=1
                    raise RuntimeError('SUPER forest boundary violated')
                collision=self.rpc.call('simGetCollisionInfo','PX4')
                if collision['has_collided'] and collision['time_stamp']!=self.result['collision_start']['time_stamp']:
                    self.result.update(collision_end=collision,failure_position=current_position,
                        observed_path_m=observed_path,boundary_violations=boundary_violations,
                        observed_speed_median_m_s=statistics.median(observed_speeds) if observed_speeds else None,
                        observed_speed_peak_m_s=max(observed_speeds,default=0))
                    raise RuntimeError('new collision during navigation')
                max_error = max(max_error, math.dist(self.position(), p))
                quat = self.pose.pose.orientation
                max_tilt = max(max_tilt, math.acos(max(-1., min(1., 1-2*(quat.x**2+quat.y**2)))))
                self.log.write(json.dumps({'topic':'pose','p':self.position(),'q':[quat.x,quat.y,quat.z,quat.w]})+'\n')
                if time.monotonic()-truth_at > .2:
                    truth = self.rpc.call('simGetGroundTruthKinematics', 'PX4'); truth_at = time.monotonic()
                    self.log.write(json.dumps({'topic':'airsim_truth','state':truth})+'\n')
                    truth_error = max(truth_error, math.dist(
                        [truth['position'][k] for k in ('x_val','y_val','z_val')],
                        [truth_start['position'][k] for k in ('x_val','y_val','z_val')]))
                if max_tilt > tilt_limit or max_error>(300 if long_case else 30):
                    self.result.update(tilt_max_rad=max_tilt,hold_error_max_m=max_error,
                        observed_path_m=observed_path,boundary_violations=boundary_violations,
                        observed_speed_median_m_s=statistics.median(observed_speeds) if observed_speeds else None,
                        observed_speed_peak_m_s=max(observed_speeds,default=0),
                        failure_position=current_position)
                    raise RuntimeError('policy flight bounds exceeded')
                if truth_error > (300 if long_case else 30): raise RuntimeError('AirSim navigation bounds exceeded')
                if fault=='none' and math.dist(current_position,target)<=1.5:
                    self.result['goal_entry_sim_s']=self.get_clock().now().nanoseconds/1e9-sim_begin
                    break
            self.result.update(observed_path_m=observed_path,boundary_violations=boundary_violations,
                observed_speed_median_m_s=statistics.median(observed_speeds) if observed_speeds else None,
                observed_speed_peak_m_s=max(observed_speeds,default=0),
                navigation_sim_s=self.get_clock().now().nanoseconds/1e9-sim_begin,
                navigation_wall_s=time.monotonic()-start)
            self.result['goal_distance_m']=math.dist(self.position(),target)
            if fault=='none' and self.result['goal_distance_m']>1.5: raise RuntimeError('forest goal not reached')
            if min(self.counts.values(), default=0) < 10 or len(self.counts) < 4:
                raise RuntimeError('missing control-chain messages')
            self.result.update(hold_error_max_m=max_error, tilt_max_rad=max_tilt,
                truth_hold_drift_max_m=truth_error,
                counts=dict(self.counts), nonzero_yaw=any(r['rate'][2]<-.02 for r in self.rates))
            def truth_yaw(q):
                return math.atan2(2*(q['w_val']*q['z_val']+q['x_val']*q['y_val']),
                                  1-2*(q['y_val']**2+q['z_val']**2))
            yaw_delta = math.remainder(truth_yaw(truth['orientation'])-
                truth_yaw(truth_start['orientation']), 2*math.pi)
            self.result['truth_yaw_change_rad'] = yaw_delta
            self.result['attitude_mode'] = True
            self.phase = 'handover'
            self.result['stop_scenario'] = 'cancel'
            if self.result.get('expected_fault'):pass
            elif self.result['stop_scenario'] == 'cancel':
                canceled = self.action_handle.cancel_goal_async(); self.wait(canceled.done, 5)
                if not canceled.result().goals_canceling: raise RuntimeError('cancel rejected')
            else:
                self.out.joinpath('stop-source').touch()
            result = self.action_handle.get_result_async(); self.wait(result.done, 5)
            self.wait(lambda: self.controller.input_control_mode.control_mode == ControlMode.POSITION, 10)
            stop_started=time.monotonic();slow_since=None
            terminal_sim_started=self.get_clock().now().nanoseconds/1e9
            self.phase='terminal'
            if not self.result.get('expected_fault'):
                # The policy supplies navigation, but has no terminal hold controller.
                # Hand over only after its measured entry into the goal sphere.
                self.result['terminal_handover_position_m']=self.position()
                cli=ActionClient(self,GoToWaypoint,'/drone0/GoToBehavior')
                self.wait(cli.server_is_ready,2)
                goal=GoToWaypoint.Goal();goal.target_pose.header=self.pose.header
                goal.target_pose.point.x,goal.target_pose.point.y,goal.target_pose.point.z=target
                goal.max_speed=2.5;goal.yaw.mode=YawMode.KEEP_YAW
                sent=cli.send_goal_async(goal);self.wait(sent.done,2)
                self.terminal_handle=sent.result()
                if not self.terminal_handle.accepted:raise RuntimeError('terminal GoTo rejected')
                self.result['terminal_behavior']='GoToBehavior'
                self.result['terminal_max_speed_m_s']=goal.max_speed
                terminal_result=self.terminal_handle.get_result_async()
            while True:
                self.delay(.03)
                v=self.twist.twist.linear
                speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
                stamp=self.twist.header.stamp.sec+self.twist.header.stamp.nanosec/1e9
                now_sim=self.get_clock().now().nanoseconds/1e9
                age=now_sim-stamp
                self.result.update(stop_speed_mps=speed,terminal_position_m=self.position(),
                    terminal_distance_m=math.dist(self.position(),target),stop_wall_s=time.monotonic()-stop_started)
                self.log.write(json.dumps(dict(topic='terminal_state',sim=now_sim,wall=time.monotonic(),
                    p=self.position(),speed=speed,state_age=age))+'\n');self.log.flush()
                if not self.result.get('expected_fault') and terminal_result.done():
                    if not terminal_result.result().result.go_to_success:raise RuntimeError('terminal GoTo failed')
                position_ok=self.result.get('fault_scenario')!='none' or math.dist(self.position(),target)<=1.5
                if 0<=age<=.2 and speed<.3 and position_ok:
                    if slow_since is None:slow_since=now_sim
                    goal_hold_sim_s=now_sim-slow_since
                    if goal_hold_sim_s>=2. and (self.result.get('expected_fault') or terminal_result.done()):break
                else:slow_since=None
                if self.result.get('expected_fault'):
                    if time.monotonic()-stop_started>10:raise RuntimeError('fault recovery stop gate failed')
                elif time.monotonic()>arrival_deadline:
                    raise RuntimeError('arrival budget expired during terminal settling')
                collision=self.rpc.call('simGetCollisionInfo','PX4')
                self.result['collision_end']=collision
                if collision['has_collided'] and collision['time_stamp']!=self.result['collision_start']['time_stamp']:
                    raise RuntimeError('new collision during terminal settling')
            self.result['stop_wall_s']=time.monotonic()-stop_started
            self.result.update(terminal_settle_wall_s=time.monotonic()-stop_started,
                terminal_settle_sim_s=self.get_clock().now().nanoseconds/1e9-terminal_sim_started,
                arrival_wall_s=time.monotonic()-start,
                arrival_sim_s=self.get_clock().now().nanoseconds/1e9-sim_begin,
                terminal_settled=True)
            self.result['low_speed_dwell_sim_s']=goal_hold_sim_s
            self.result['goal_hold_sim_s']=goal_hold_sim_s
            self.result['stop_speed_mps'] = speed
            self.result['goal_distance_m']=math.dist(self.position(),target)
            self.result['collision_end'] = self.rpc.call('simGetCollisionInfo', 'PX4')
            if (self.result['collision_end']['has_collided'] and
                self.result['collision_end']['time_stamp'] != self.result['collision_start']['time_stamp']):
                raise RuntimeError('new AirSim collision')
            self.confirm_execution_stop()
            self.result['control_passed'] = True
        except BaseException as exc:
            self.result['error'] = str(exc)
            # Preserve the trigger before a potentially long landing recovery.
            (self.out/'failure.json').write_text(json.dumps(self.result,indent=2)+'\n')
        finally:
            self.phase = 'stopping'
            if self.terminal_handle and self.terminal_handle.accepted and not self.result.get('execution_stop_confirmed'):
                try:
                    canceled=self.terminal_handle.cancel_goal_async();self.wait(canceled.done,5)
                    finished=self.terminal_handle.get_result_async();self.wait(finished.done,5)
                except BaseException as exc:self.result['terminal_cancel_error']=str(exc)
            if self.action_handle and self.action_handle.accepted and not self.result.get('execution_stop_confirmed'):
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
