"""Task/score role for isolated FollowReference ACRO AirSim acceptance."""
import collections
import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
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
        self.pose = self.twist = self.platform = self.controller = None
        self.px4_mode = None
        self.source_ready = False
        self.action_handle = None; self.phase = 'ground'; self.flying = False
        self.counts = collections.Counter(); self.frames = {}; self.rates = []; self.targets = []
        self.actuator_rates = []; self.actuator_thrust = None
        self.result = {'success': False, 'scope': 'FollowReference ACRO control only; source is not IPC'}
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
        self.create_subscription(Bool, '/drone0/acro_validation_source/ready',
            lambda m: setattr(self, 'source_ready', m.data), 1)
        for topic, kind in [('external_acro/control', ControlFrame),
                            ('motion_reference/thrust', Thrust),
                            ('actuator_command/thrust', Thrust),
                            ('actuator_command/twist', TwistStamped),
                            ('fmu/in/vehicle_rates_setpoint', VehicleRatesSetpoint)]:
            self.create_subscription(kind, '/drone0/'+topic,
                lambda m, t=topic: self.sample(t, m), qos_profile_sensor_data)

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
            self.result['initial_position'] = self.position()
            self.result['collision_start'] = self.rpc.call('simGetCollisionInfo', 'PX4')
            self.preflight_probe()
            self.phase = 'arming'; self.service('set_arming_state', True); self.flying = True
            self.wait(lambda: self.platform.armed and self.px4_mode and self.px4_mode.flag_armed)
            self.service('set_offboard_mode', True)
            self.wait(lambda: self.platform.offboard and self.px4_mode.flag_control_offboard_enabled)
            self.delay(.2)
            self.phase = 'takeoff'
            self.action(Takeoff, 'TakeoffBehavior', Takeoff.Goal(takeoff_height=2., takeoff_speed=.5), 'takeoff_success')
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
            q = self.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            goal.yaw.mode = YawMode.FIXED_YAW; goal.yaw.angle = yaw+.15
            goal.max_speed_x = goal.max_speed_y = goal.max_speed_z = .5
            self.result['target'] = p
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
            self.wait(lambda: self.controller.input_control_mode.control_mode == ControlMode.BODY_RATES and
                self.px4_mode.flag_control_rates_enabled and not self.px4_mode.flag_control_attitude_enabled and
                not self.px4_mode.flag_control_velocity_enabled and self.counts['fmu/in/vehicle_rates_setpoint'] >= 5, 5)
            self.out.joinpath('acro-established').touch()
            start = time.monotonic(); max_error = 0.; max_tilt = 0.
            truth_error = 0.; truth_at = 0.
            while time.monotonic()-start < 5:
                self.delay(.03)
                if run_result.done(): raise RuntimeError('ACRO ended before hold interval')
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
                if max_error > .75 or max_tilt > .35: raise RuntimeError('ACRO bounds exceeded')
                if truth_error > .75: raise RuntimeError('AirSim actual position bounds exceeded')
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
            if not -.4 < yaw_delta < -.03: raise RuntimeError('actual yaw response direction/magnitude failed')
            if not self.result['nonzero_yaw']: raise RuntimeError('no nonzero PX4 rate command')
            self.phase = 'loss'; self.out.joinpath('stop-source').touch()
            result = self.action_handle.get_result_async(); self.wait(result.done, 5)
            self.wait(lambda: self.controller.input_control_mode.control_mode == ControlMode.POSITION, 10)
            self.delay(1)
            v = self.twist.twist.linear
            speed = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            if speed > .5: raise RuntimeError('stop not confirmed')
            self.result['stop_speed_mps'] = speed
            self.result['collision_end'] = self.rpc.call('simGetCollisionInfo', 'PX4')
            if (self.result['collision_end']['has_collided'] and
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
