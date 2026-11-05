"""Observe the actual AS2 outputs, not just raw PX4 DDS discovery."""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.msg import PlatformInfo, PlatformStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from std_srvs.srv import SetBool
from as2_msgs.action import Takeoff, Land, GoToWaypoint
from px4_msgs.msg import (VehicleCommandAck, VehicleControlMode, VehicleOdometry,
                          OffboardControlMode, TimesyncStatus)
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from as2_msgs.srv import GetOrigin


def spin_until(node, predicate, timeout=30):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError('AS2/PX4 feedback timed out')
        rclpy.spin_once(node, timeout_sec=0.1)


def verify_flight(node, latest, acknowledgements):
    """Opt-in test for the dedicated SITL scene created by verify.sh."""
    def call(service, command, feedback):
        print('Checking AS2 service:', service, file=sys.stderr, flush=True)
        client = node.create_client(SetBool, '/drone0/' + service)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('service unavailable: ' + service)
        # Require a new PX4 ACK as well as actual firmware state feedback.
        acknowledgements.pop(command, None)
        future = client.call_async(SetBool.Request(data=True))
        spin_until(node, future.done)
        if not future.result().success:
            raise RuntimeError('AS2 rejected ' + service)
        spin_until(node, lambda: command in acknowledgements)
        diagnostic = dict(command=command, result=acknowledgements[command],
                          clock_us=latest['/clock'].clock.sec * 1000000 + latest['/clock'].clock.nanosec // 1000,
                          odometry_us=latest['/drone0/fmu/out/vehicle_odometry'].timestamp,
                          wall_us=time.time_ns() // 1000)
        mode = latest.get('/drone0/fmu/in/offboard_control_mode')
        if mode is not None:
            diagnostic['offboard_us'] = mode.timestamp
        print(json.dumps(diagnostic), file=sys.stderr, flush=True)
        if acknowledgements[command] != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError(f'PX4 rejected {service}: {acknowledgements[command]}')
        spin_until(node, feedback)
        print('PX4 ACK and state confirmed:', service, file=sys.stderr, flush=True)

    def action(kind, name, goal, result_field):
        print('Checking AS2 action:', name, file=sys.stderr, flush=True)
        client = ActionClient(node, kind, '/drone0/' + name)
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('action unavailable: ' + name)
        future = client.send_goal_async(goal)
        spin_until(node, future.done)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('AS2 rejected action ' + name)
        result = handle.get_result_async()
        try:
            spin_until(node, result.done, timeout=90)
        except TimeoutError:
            handle.cancel_goal_async()
            raise
        if not getattr(result.result().result, result_field):
            raise RuntimeError('AS2 action failed: ' + name)
        print('AS2 action succeeded:', name, file=sys.stderr, flush=True)

    # Allow the simulated EKF/GPS to settle; no forced arm or disabled checks.
    settle = time.monotonic() + 15
    spin_until(node, lambda: time.monotonic() >= settle)
    call('set_arming_state', 400,
         lambda: latest['/drone0/fmu/out/vehicle_control_mode'].flag_armed)
    call('set_offboard_mode', 176,
         lambda: latest['/drone0/fmu/out/vehicle_control_mode'].flag_control_offboard_enabled)
    start = latest['/drone0/self_localization/pose']
    initial_z = start.pose.position.z
    action(Takeoff, 'TakeoffBehavior', Takeoff.Goal(takeoff_height=1.5, takeoff_speed=0.5),
           'takeoff_success')
    spin_until(node, lambda: latest['/drone0/self_localization/pose'].pose.position.z > initial_z + 1.0)
    # Action completion precedes the asynchronous platform/info broadcast.
    # Let the other behavior nodes observe the FLYING state before the next goal.
    spin_until(node, lambda: latest['/drone0/platform/info'].status.state == PlatformStatus.FLYING)
    transition_stamp = latest['/drone0/platform/info'].header.stamp
    transition_ns = transition_stamp.sec * 1000000000 + transition_stamp.nanosec
    spin_until(node, lambda: (
        latest['/drone0/platform/info'].status.state == PlatformStatus.FLYING and
        latest['/drone0/platform/info'].header.stamp.sec * 1000000000 +
        latest['/drone0/platform/info'].header.stamp.nanosec > transition_ns + 300000000))
    target = GoToWaypoint.Goal()
    target.target_pose.header.frame_id = start.header.frame_id
    current = latest['/drone0/self_localization/pose'].pose.position
    target.target_pose.point.x = current.x + 1.0
    target.target_pose.point.y = current.y
    target.target_pose.point.z = current.z
    target.max_speed = 0.5
    action(GoToWaypoint, 'GoToBehavior', target, 'go_to_success')
    current = latest['/drone0/self_localization/pose'].pose.position
    waypoint_error = math.dist((current.x, current.y, current.z), (
        target.target_pose.point.x, target.target_pose.point.y, target.target_pose.point.z))
    if waypoint_error > 0.5:
        raise RuntimeError(f'waypoint tracking error: {waypoint_error}')
    hold_position = (current.x, current.y, current.z)
    hold_until = time.monotonic() + 2
    spin_until(node, lambda: time.monotonic() >= hold_until)
    current = latest['/drone0/self_localization/pose'].pose.position
    hold_drift = math.dist(hold_position, (current.x, current.y, current.z))
    if hold_drift > 0.5:
        raise RuntimeError(f'hold drift: {hold_drift}')
    action(Land, 'LandBehavior', Land.Goal(land_speed=0.4), 'land_success')
    spin_until(node, lambda: not latest['/drone0/fmu/out/vehicle_control_mode'].flag_armed)
    return dict(takeoff=True, waypoint=True, waypoint_error_m=waypoint_error,
                hold_drift_m=hold_drift, land=True, disarmed=True)


def main():
    rclpy.init()
    node = rclpy.create_node('verify_as2_interaction')
    received = {}
    latest = {}
    acknowledgements = {}
    subscriptions = []
    expected = {
        '/clock': Clock,
        '/drone0/sensor_measurements/odom': Odometry,
        '/drone0/sensor_measurements/imu': Imu,
        '/drone0/sensor_measurements/gps': NavSatFix,
        '/drone0/sensor_measurements/battery': BatteryState,
        '/drone0/self_localization/pose': PoseStamped,
        '/drone0/self_localization/twist': TwistStamped,
        '/drone0/platform/info': PlatformInfo,
        '/drone0/fmu/out/vehicle_control_mode': VehicleControlMode,
        '/drone0/fmu/out/vehicle_odometry': VehicleOdometry,
    }

    def callback(topic, msg):
        if isinstance(msg, Clock):
            if msg.clock.sec <= 0:
                return
        elif hasattr(msg, 'header'):
            if msg.header.stamp.sec <= 0:
                return
        if isinstance(msg, Odometry):
            p = msg.pose.pose.position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                return
        if isinstance(msg, NavSatFix):
            if not (-90 <= msg.latitude <= 90 and -180 <= msg.longitude <= 180):
                return
        if isinstance(msg, BatteryState):
            if not (0 <= msg.percentage <= 1):
                return
        latest[topic] = msg
        received[topic] = received.get(topic, 0) + 1

    for topic, kind in expected.items():
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, topic=topic: callback(topic, msg),
            qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        VehicleCommandAck, '/drone0/fmu/out/vehicle_command_ack_v1',
        lambda msg: acknowledgements.update({msg.command: msg.result})
        if msg.target_system == 1 and msg.target_component == 1 else None,
        qos_profile_sensor_data))
    for topic, kind in [('/drone0/fmu/in/offboard_control_mode', OffboardControlMode),
                        ('/drone0/fmu/out/timesync_status', TimesyncStatus)]:
        subscriptions.append(node.create_subscription(kind, topic,
            lambda msg, topic=topic: latest.update({topic: msg}), qos_profile_sensor_data))
    deadline = time.monotonic() + 45
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if all(received.get(topic, 0) >= 3 for topic in expected):
                break
        missing = [topic for topic in expected if received.get(topic, 0) < 3]
        report = dict(received=received, missing=missing)
        if missing:
            print(json.dumps(report, indent=2), flush=True)
            raise SystemExit(1)
        # Stationary startup: independently check the platform's NED → ENU path.
        raw = latest['/drone0/fmu/out/vehicle_odometry']
        converted = latest['/drone0/sensor_measurements/odom'].pose.pose.position
        error = math.dist((converted.x, converted.y, converted.z), (
            raw.position[1], raw.position[0], -raw.position[2]))
        report['ned_to_enu_error_m'] = error
        if not math.isfinite(error) or error > 0.25:
            print(json.dumps(report, indent=2), flush=True)
            raise RuntimeError('PX4 → AS2 NED/ENU conversion failed')
        parameters = node.create_client(GetParameters, '/drone0/state_estimator/get_parameters')
        if not parameters.wait_for_service(timeout_sec=10):
            raise RuntimeError('state estimator parameters unavailable')
        future = parameters.call_async(GetParameters.Request(names=[
            'raw_odometry.use_gps', 'raw_odometry.gps_origin.set_origin']))
        spin_until(node, future.done)
        values = future.result().values
        if not values[0].bool_value or values[1].string_value != 'first_gps':
            raise RuntimeError('AS2 launch discarded the configured GPS/origin parameters')
        origin = node.create_client(GetOrigin, '/drone0/get_origin')
        if not origin.wait_for_service(timeout_sec=10):
            raise RuntimeError('GPS origin service unavailable')
        future = origin.call_async(GetOrigin.Request())
        spin_until(node, future.done)
        if not future.result().success:
            raise RuntimeError('GPS origin not initialized')
        report['gps_origin'] = dict(latitude=future.result().origin.latitude,
                                    longitude=future.result().origin.longitude,
                                    altitude=future.result().origin.altitude)
        if os.environ.get('VERIFY_AS2_FLIGHT') == '1':
            try:
                report['flight'] = verify_flight(node, latest, acknowledgements)
            except Exception as exc:
                report['error'] = str(exc)
                print(json.dumps(report, indent=2), flush=True)
                raise
        print(json.dumps(report, indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
