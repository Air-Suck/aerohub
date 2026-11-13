#!/usr/bin/env python3
"""Exercise FAST-LIVO2 as AS2's authoritative state and consume its registered cloud."""
import json
import math
import os
from pathlib import Path
import struct
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import GoToWaypoint, Land, Takeoff
from as2_msgs.msg import PlatformInfo, PlatformStatus, ControlMode, ControllerInfo
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleControlMode, VehicleStatus
from sensor_msgs.msg import Image, PointCloud2
from std_srvs.srv import SetBool

from airsim_rpc import Rpc


class Mission:
    def __init__(self):
        self.output = Path(os.environ.get('FAST_LIVO_RESULTS', '/results'))
        self.output.mkdir(parents=True, exist_ok=True)
        self.scenario = os.environ.get('FAST_LIVO_SCENARIO', 'authority')
        if self.scenario not in ('shadow', 'authority', 'sensor_fault', 'algorithm_fault'):
            raise ValueError('invalid FAST_LIVO_SCENARIO')
        self.warmup_seconds = float(os.environ.get('FAST_LIVO_WARMUP_SECONDS', '0'))
        if not 0 <= self.warmup_seconds <= 30:
            raise ValueError('FAST_LIVO_WARMUP_SECONDS must be between 0 and 30')
        self.rpc = Rpc(os.environ.get('AIRSIM_HOST', '127.0.0.1'),
                       int(os.environ.get('AIRSIM_RPC_PORT', '44471')), timeout=5)
        self.node = rclpy.create_node('fast_livo2_airsim_acceptance')
        self.latest, self.counts, self.received_at, self.acks = {}, {}, {}, {}
        self.cloud_metrics = {'messages': 0, 'nonempty': 0, 'max_width': 0, 'max_span_m': 0.0}
        self.truth_samples = []
        self.fast_truth_samples = []
        self.synchronized_samples = []
        self.pose_origin = None
        self.truth_origin = None
        self.fast_pose_origin = None
        self.fast_truth_origin = None
        self.guard_enabled = False
        self.preflight_checks_pass_before_arming = False
        topics = {
            '/drone0/self_localization/pose': PoseStamped,
            '/drone0/self_localization/twist': TwistStamped,
            '/drone0/fast_livo2/odometry': Odometry,
            '/drone0/platform/info': PlatformInfo,
            '/drone0/controller/info': ControllerInfo,
            '/drone0/sensor_measurements/FrontCamera/rgb/image': Image,
            '/drone0/sensor_measurements/Lidar/points': PointCloud2,
            '/drone0/fmu/out/vehicle_control_mode': VehicleControlMode,
            '/drone0/fmu/out/vehicle_status_v4': VehicleStatus,
        }
        self.subscriptions = [self.node.create_subscription(
            kind, topic, lambda msg, topic=topic:self.receive(topic, msg), qos_profile_sensor_data)
            for topic, kind in topics.items()]
        self.subscriptions.append(self.node.create_subscription(
            VehicleCommandAck, '/drone0/fmu/out/vehicle_command_ack_v' + str(VehicleCommandAck.MESSAGE_VERSION),
            self.ack, qos_profile_sensor_data))
        self.subscriptions.append(self.node.create_subscription(
            PointCloud2, '/drone0/perception/fast_livo2/registered_points', self.cloud,
            qos_profile_sensor_data))
        self.vehicle_command = self.node.create_publisher(
            VehicleCommand, '/drone0/fmu/in/vehicle_command', qos_profile_sensor_data)
        self.clients = []

    def receive(self, topic, message):
        if hasattr(message, 'header') and message.header.stamp.sec <= 0:
            return
        self.latest[topic] = message
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.received_at[topic] = time.monotonic()

    def ack(self, message):
        self.acks[message.command] = message.result

    def cloud(self, message):
        topic = '/drone0/perception/fast_livo2/registered_points'
        self.received_at[topic] = time.monotonic()
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.cloud_metrics['messages'] += 1
        if message.header.frame_id != 'drone0/livo_world' or message.width == 0:
            return
        fields = {field.name: field for field in message.fields}
        if not {'x', 'y', 'z'} <= fields.keys():
            return
        values = []
        step = max(1, message.width // 500)
        endian = '>' if message.is_bigendian else '<'
        for index in range(0, message.width, step):
            base = index * message.point_step
            point = tuple(struct.unpack_from(endian + 'f', message.data, base + fields[name].offset)[0]
                          for name in ('x', 'y', 'z'))
            if all(math.isfinite(value) for value in point):
                values.append(point)
        if not values:
            return
        span = max(max(point[axis] for point in values) - min(point[axis] for point in values)
                   for axis in range(3))
        self.cloud_metrics['nonempty'] += 1
        self.cloud_metrics['max_width'] = max(self.cloud_metrics['max_width'], message.width)
        self.cloud_metrics['max_span_m'] = max(self.cloud_metrics['max_span_m'], span)

    def airsim_position(self):
        return self.rpc.call('simGetVehiclePose', 'PX4')['position']

    def truth(self):
        point = self.airsim_position()
        return (float(point['y_val']), float(point['x_val']), -float(point['z_val']))

    def livo_truth(self):
        point = self.airsim_position()
        return (float(point['x_val']), -float(point['y_val']), -float(point['z_val']))

    def sample(self):
        topic = '/drone0/self_localization/pose'
        fast_topic = '/drone0/fast_livo2/odometry'
        if topic not in self.latest or fast_topic not in self.latest:
            return
        p = self.latest[topic].pose.position
        estimate = (p.x, p.y, p.z)
        actual = self.truth()
        livo_actual = self.livo_truth()
        if self.pose_origin is None:
            self.pose_origin, self.truth_origin = estimate, actual
        estimate_relative = tuple(estimate[i] - self.pose_origin[i] for i in range(3))
        truth_relative = tuple(actual[i] - self.truth_origin[i] for i in range(3))
        self.truth_samples.append({'estimate': estimate_relative, 'truth': truth_relative,
                                   'error_m': math.dist(estimate_relative, truth_relative),
                                   'wall_time': time.time()})
        fp = self.latest[fast_topic].pose.pose.position
        fast = (fp.x, fp.y, fp.z)
        if self.fast_pose_origin is None:
            self.fast_pose_origin, self.fast_truth_origin = fast, livo_actual
        fast_relative = tuple(fast[i] - self.fast_pose_origin[i] for i in range(3))
        fast_truth_relative = tuple(livo_actual[i] - self.fast_truth_origin[i] for i in range(3))
        wall_time = time.time()
        self.fast_truth_samples.append({
            'estimate': fast_relative, 'truth': fast_truth_relative,
            'error_m': math.dist(fast_relative, fast_truth_relative),
            'wall_time': wall_time,
        })
        self.synchronized_samples.append({
            'canonical_estimate': estimate_relative,
            'canonical_truth': truth_relative,
            'canonical_error_m': math.dist(estimate_relative, truth_relative),
            'raw_estimate': fast_relative,
            'raw_truth': fast_truth_relative,
            'raw_error_m': math.dist(fast_relative, fast_truth_relative),
            'wall_time': wall_time,
        })

    def write_samples(self):
        (self.output/'trajectory-truth.json').write_text(
            json.dumps(self.truth_samples, indent=2)+'\n')
        (self.output/'fast-livo2-shadow-truth.json').write_text(
            json.dumps(self.fast_truth_samples, indent=2)+'\n')


        (self.output/'synchronized-state-truth.json').write_text(
            json.dumps(self.synchronized_samples, indent=2)+'\n')
    def spin_until(self, predicate, timeout=45):
        deadline = time.monotonic() + timeout
        next_sample = 0.0
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('FAST-LIVO2 AS2 mission timed out')
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if time.monotonic() >= next_sample:
                self.sample(); next_sample = time.monotonic() + 0.2
                if (self.guard_enabled and self.fast_truth_samples and
                        self.fast_truth_samples[-1]['error_m'] > 1.0):
                    raise RuntimeError('FAST-LIVO2 flight guard exceeded 1.0 m')

    def stream_age(self, topic):
        return time.monotonic() - self.received_at.get(topic, 0.0)

    def service(self, name, command, feedback):
        client = self.node.create_client(SetBool, '/drone0/' + name); self.clients.append(client)
        self.spin_until(client.service_is_ready, 20)
        self.acks.pop(command, None)
        future = client.call_async(SetBool.Request(data=True)); self.spin_until(future.done, 10)
        if not future.result().success:
            raise RuntimeError('AS2 rejected ' + name)
        self.spin_until(lambda: command in self.acks, 10)
        if self.acks[command] != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError('PX4 rejected ' + name)
        self.spin_until(feedback, 15)

    def action(self, kind, name, goal, field, timeout=90):
        client = ActionClient(self.node, kind, '/drone0/' + name); self.clients.append(client)
        self.spin_until(client.server_is_ready, 20)
        future = client.send_goal_async(goal); self.spin_until(future.done, 10)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('AS2 rejected ' + name)
        result = handle.get_result_async()
        try:
            self.spin_until(result.done, timeout)
        except Exception:
            handle.cancel_goal_async()
            raise
        if not getattr(result.result().result, field):
            raise RuntimeError('AS2 failed ' + name)

    def screenshot(self, name):
        request = {'camera_name':'survey','image_type':0,'pixels_as_float':False,'compress':True}
        response = self.rpc.call('simGetImages', [request], '', True)[0]
        (self.output / (name + '.png')).write_bytes(bytes(response['image_data_uint8']))

    def hover(self):
        client = self.node.create_client(SetControlMode, '/drone0/controller/set_control_mode')
        self.clients.append(client)
        self.spin_until(client.service_is_ready, 5)
        request = SetControlMode.Request()
        request.control_mode.control_mode = ControlMode.HOVER
        request.control_mode.yaw_mode = ControlMode.YAW_ANGLE
        future = client.call_async(request); self.spin_until(future.done, 5)

    def px4_land(self):
        self.acks.pop(VehicleCommand.VEHICLE_CMD_NAV_LAND, None)
        message = VehicleCommand()
        message.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        message.target_system = 71
        message.target_component = 1
        message.source_system = 255
        message.source_component = 1
        message.from_external = True
        for _ in range(10):
            message.timestamp = self.node.get_clock().now().nanoseconds // 1000
            self.vehicle_command.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.acks.get(message.command) == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                break
        self.spin_until(
            lambda:not self.latest.get(
                '/drone0/fmu/out/vehicle_control_mode', VehicleControlMode()).flag_armed,
            120)

    def initialize(self):
        required = ['/drone0/self_localization/pose', '/drone0/self_localization/twist',
                    '/drone0/fast_livo2/odometry', '/drone0/platform/info',
                    '/drone0/fmu/out/vehicle_control_mode', '/drone0/fmu/out/vehicle_status_v4']
        self.spin_until(lambda: all(self.counts.get(topic, 0) >= 20 for topic in required[:3]) and
                        all(topic in self.latest for topic in required[3:]) and
                        self.cloud_metrics['nonempty'] >= 3, 120)
        ready_since = None

        def stable_preflight():
            nonlocal ready_since
            ready = bool(
                self.latest['/drone0/fmu/out/vehicle_status_v4'].pre_flight_checks_pass)
            if not ready:
                ready_since = None
                return False
            ready_since = ready_since or time.monotonic()
            return time.monotonic() - ready_since >= 2.0

        self.spin_until(stable_preflight, 120)
        self.preflight_checks_pass_before_arming = True
        self.screenshot('initialized')

    def retry_service(self, name, command, feedback):
        for attempt in range(12):
            try:
                self.service(name, command, feedback)
                return
            except (RuntimeError, TimeoutError):
                if attempt == 11:
                    raise
                retry_at = time.monotonic() + 5.0
                self.spin_until(lambda: time.monotonic() >= retry_at, 6)

    def arm_and_offboard(self):
        self.retry_service('set_arming_state', 400,
                           lambda:self.latest['/drone0/fmu/out/vehicle_control_mode'].flag_armed)
        self.retry_service('set_offboard_mode', 176,
                           lambda:self.latest['/drone0/fmu/out/vehicle_control_mode'].flag_control_offboard_enabled)

    def run_shadow(self):
        self.initialize()
        warmup_until = time.monotonic() + self.warmup_seconds
        if self.warmup_seconds:
            self.spin_until(lambda:time.monotonic() >= warmup_until, self.warmup_seconds + 4)
        warmup_drift = self.fast_truth_samples[-1]['error_m']
        if warmup_drift > 0.2:
            raise RuntimeError(f'FAST-LIVO2 stationary warmup drift {warmup_drift:.3f} m')
        self.pose_origin = self.truth_origin = None
        self.fast_pose_origin = self.fast_truth_origin = None
        self.truth_samples.clear()
        self.fast_truth_samples.clear()
        self.synchronized_samples.clear()
        self.sample()
        self.arm_and_offboard()
        initial_z = self.latest['/drone0/self_localization/pose'].pose.position.z
        self.action(Takeoff, 'TakeoffBehavior', Takeoff.Goal(takeoff_height=0.75, takeoff_speed=0.25),
                    'takeoff_success', 90)
        self.spin_until(lambda:self.latest['/drone0/self_localization/pose'].pose.position.z > initial_z + 0.5, 20)
        self.spin_until(lambda:self.latest['/drone0/platform/info'].status.state == PlatformStatus.FLYING, 20)
        self.screenshot('shadow-airborne')
        until = time.monotonic() + 2
        self.spin_until(lambda:time.monotonic() >= until, 5)
        airborne = self.fast_truth_samples[-1]
        current = self.latest['/drone0/self_localization/pose'].pose.position
        target = GoToWaypoint.Goal()
        target.target_pose.header.frame_id = 'earth'
        target.target_pose.point.x = current.x + 1.0
        target.target_pose.point.y = current.y
        target.target_pose.point.z = current.z
        target.max_speed = 0.25
        self.action(GoToWaypoint, 'GoToBehavior', target, 'go_to_success', 90)
        until = time.monotonic() + 2
        self.spin_until(lambda:time.monotonic() >= until, 5)
        horizontal = self.fast_truth_samples[-1]
        truth_movement = math.dist((0.0, 0.0, 0.0), horizontal['truth'])
        estimate_movement = math.dist((0.0, 0.0, 0.0), horizontal['estimate'])
        self.action(Land, 'LandBehavior', Land.Goal(land_speed=0.3), 'land_success', 90)
        self.spin_until(lambda:not self.latest['/drone0/fmu/out/vehicle_control_mode'].flag_armed, 30)
        self.sample(); self.screenshot('shadow-landed'); self.write_samples()
        result = {
            'success': True, 'scenario': 'shadow',
            'authoritative_state_provider': 'PX4 raw odometry AS2 plugin',
            'fast_livo2_role': 'shadow state estimator',
            'stationary_warmup_seconds': self.warmup_seconds,
            'stationary_warmup_drift_m': warmup_drift,
            'px4_preflight_checks_pass_before_arming': self.preflight_checks_pass_before_arming,
            'fast_livo2_airborne_error_m': airborne['error_m'],
            'fast_livo2_horizontal_error_m': horizontal['error_m'],
            'fast_livo2_airborne_movement_m': estimate_movement,
            'truth_airborne_movement_m': truth_movement,
            'cloud_consumer': self.cloud_metrics,
            'landed_and_disarmed': True,
        }
        if (airborne['error_m'] > 0.5 or horizontal['error_m'] > 0.5 or
                truth_movement < 0.5 or estimate_movement < 0.3):
            raise RuntimeError('FAST-LIVO2 shadow threshold failed: ' + json.dumps(result))
        (self.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)

    def run(self):
        if self.scenario == 'shadow':
            self.run_shadow()
            return
        if self.scenario in ('sensor_fault', 'algorithm_fault'):
            self.run_fault()
            return
        self.initialize()
        self.arm_and_offboard()
        self.guard_enabled = True
        initial_z = self.latest['/drone0/self_localization/pose'].pose.position.z
        self.action(Takeoff, 'TakeoffBehavior', Takeoff.Goal(takeoff_height=0.35, takeoff_speed=0.15),
                    'takeoff_success', 120)
        self.spin_until(
            lambda: self.latest['/drone0/platform/info'].status.state == PlatformStatus.FLYING, 20)
        settle_until = time.monotonic() + 2.0
        self.spin_until(lambda: time.monotonic() >= settle_until, 4)
        for target_z in (initial_z + 0.45, initial_z + 0.55,
                         initial_z + 0.65, initial_z + 0.75):
            current = self.latest['/drone0/self_localization/pose'].pose.position
            climb = GoToWaypoint.Goal()
            climb.target_pose.header.frame_id = 'earth'
            climb.target_pose.point.x = current.x
            climb.target_pose.point.y = current.y
            climb.target_pose.point.z = target_z
            climb.max_speed = 0.1
            self.action(GoToWaypoint, 'GoToBehavior', climb, 'go_to_success', 120)
            settle_until = time.monotonic() + 2.0
            self.spin_until(lambda: time.monotonic() >= settle_until, 4)
        self.spin_until(lambda:self.latest['/drone0/self_localization/pose'].pose.position.z > initial_z + 0.5, 30)
        self.spin_until(lambda:self.latest['/drone0/platform/info'].status.state == PlatformStatus.FLYING, 20)
        self.screenshot('airborne')
        current = self.latest['/drone0/self_localization/pose'].pose.position
        horizontal_origin = (current.x, current.y, current.z)
        target = None
        for step in range(1, 6):
            target = GoToWaypoint.Goal()
            target.target_pose.header.frame_id = 'earth'
            target.target_pose.point.x = horizontal_origin[0] + 0.2 * step
            target.target_pose.point.y = horizontal_origin[1]
            target.target_pose.point.z = horizontal_origin[2]
            target.max_speed = 0.15
            self.action(GoToWaypoint, 'GoToBehavior', target, 'go_to_success', 120)
            settle_until = time.monotonic() + 0.5
            self.spin_until(lambda: time.monotonic() >= settle_until, 2)
        reached = self.latest['/drone0/self_localization/pose'].pose.position
        waypoint_error = math.dist((reached.x,reached.y,reached.z),
                                   (target.target_pose.point.x,target.target_pose.point.y,target.target_pose.point.z))
        hold = (reached.x,reached.y,reached.z); until = time.monotonic() + 2
        self.spin_until(lambda:time.monotonic() >= until, 5)
        reached = self.latest['/drone0/self_localization/pose'].pose.position
        hold_drift = math.dist(hold,(reached.x,reached.y,reached.z))
        self.screenshot('waypoint')
        self.action(Land, 'LandBehavior', Land.Goal(land_speed=0.3), 'land_success', 120)
        self.spin_until(lambda:not self.latest['/drone0/fmu/out/vehicle_control_mode'].flag_armed, 30)
        self.sample(); self.screenshot('landed')
        errors = [sample['error_m'] for sample in self.truth_samples]
        result = {
            'success': True, 'authoritative_state_provider': 'FAST-LIVO2 AS2 plugin',
            'actual_downstream_consumer': 'AS2 motion controller and Takeoff/GoTo/Land behaviors',
            'px4_preflight_checks_pass_before_arming': self.preflight_checks_pass_before_arming,
            'pose_messages': self.counts.get('/drone0/self_localization/pose', 0),
            'raw_fast_livo2_messages': self.counts.get('/drone0/fast_livo2/odometry', 0),
            'cloud_consumer': self.cloud_metrics,
            'waypoint_error_m': waypoint_error, 'hold_drift_m': hold_drift,
            'truth_error_max_m': max(errors), 'truth_error_final_m': errors[-1],
            'truth_samples': len(errors), 'landed_and_disarmed': True,
        }
        if waypoint_error > 0.5 or hold_drift > 0.5 or result['truth_error_final_m'] > 1.0:
            raise RuntimeError('FAST-LIVO2 acceptance threshold failed: ' + json.dumps(result))
        self.write_samples()
        (self.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)

    def run_fault(self):
        self.initialize()
        self.arm_and_offboard()
        self.guard_enabled = True
        self.action(
            Takeoff, 'TakeoffBehavior',
            Takeoff.Goal(takeoff_height=0.5, takeoff_speed=0.15),
            'takeoff_success', 120)
        self.spin_until(
            lambda: self.latest['/drone0/platform/info'].status.state == PlatformStatus.FLYING,
            20)
        settle_until = time.monotonic() + 2.0
        self.spin_until(lambda: time.monotonic() >= settle_until, 4)
        self.screenshot(self.scenario + '-airborne')

        if self.scenario == 'sensor_fault':
            watched = [
                '/drone0/sensor_measurements/FrontCamera/rgb/image',
                '/drone0/sensor_measurements/Lidar/points',
            ]
            stopped_component = 'AirSim sensor adapter container'
        else:
            watched = [
                '/drone0/fast_livo2/odometry',
                '/drone0/perception/fast_livo2/registered_points',
            ]
            stopped_component = 'FAST-LIVO2 algorithm container'
        baseline_counts = {topic: self.counts.get(topic, 0) for topic in watched}
        ready = {
            'fault': self.scenario,
            'watched_topics': watched,
            'baseline_counts': baseline_counts,
            'ready_wall_time': time.time(),
        }
        (self.output / 'fault-ready.json').write_text(json.dumps(ready, indent=2) + '\n')
        detection_started = time.monotonic()
        self.spin_until(lambda: all(self.stream_age(topic) >= 2.0 for topic in watched), 30)
        detected_wall_time = time.time()
        final_counts = {topic: self.counts.get(topic, 0) for topic in watched}
        recovery = self.controlled_recovery()
        self.sample()
        self.screenshot(self.scenario + '-landed')
        self.write_samples()
        result = {
            'success': recovery['landed_and_disarmed'],
            'scenario': self.scenario,
            'fault': self.scenario,
            'stopped_component': stopped_component,
            'watched_topics': watched,
            'baseline_counts': baseline_counts,
            'final_counts': final_counts,
            'detection_seconds': time.monotonic() - detection_started,
            'detected_wall_time': detected_wall_time,
            'hover_requested': recovery['hover'],
            'landed_and_disarmed': recovery['landed_and_disarmed'],
            'cloud_consumer': self.cloud_metrics,
        }
        if recovery.get('hover_error'):
            result['hover_error'] = recovery['hover_error']
        if recovery.get('land_error'):
            result['land_error'] = recovery['land_error']
        if not result['success']:
            raise RuntimeError('FAST-LIVO2 fault recovery failed: ' + json.dumps(result))
        (self.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)

    def controlled_recovery(self):
        report = {'hover': False, 'landed_and_disarmed': False}
        self.guard_enabled = False
        mode = self.latest.get('/drone0/fmu/out/vehicle_control_mode')
        if not mode or not mode.flag_armed:
            report['landed_and_disarmed'] = True
            return report
        try:
            self.hover()
            report['hover'] = True
        except Exception as error:
            report['hover_error'] = str(error)
        try:
            self.px4_land()
        except Exception as error:
            report['land_error'] = str(error)
        report['landed_and_disarmed'] = not self.latest.get(
            '/drone0/fmu/out/vehicle_control_mode', VehicleControlMode()).flag_armed
        return report

    def emergency(self, error):
        report = {'error': str(error), **self.controlled_recovery()}
        self.write_samples()
        (self.output/'failure.json').write_text(json.dumps(report, indent=2)+'\n')


def main():
    rclpy.init(); mission = Mission()
    try:
        mission.run()
    except Exception as error:
        mission.emergency(error)
        raise
    finally:
        mission.node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
