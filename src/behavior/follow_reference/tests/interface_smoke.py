"""Real FollowReference host/plugin with simulated algorithm and controller services.

No PX4, no flight dynamics: this tests the ROS contract and lifecycle only.
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from as2_msgs.action import FollowReference
from as2_msgs.msg import ControllerInfo, ControlMode, PlatformInfo, PlatformStatus, Thrust, YawMode
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import TransformStamped, TwistStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger
from uav_follow_reference_acro.msg import ControlFrame, ControlSession


def main():
    rclpy.init()
    node = rclpy.create_node('external_acro_test')
    mode = ControlMode()
    attitude_mode = "--attitude" in sys.argv
    sessions, rates, thrusts = [], [], []
    poses = []
    state = {'session': None, 'sequence': 0, 'publish': False, 'revision_offset': 0,
             'rate_x': 0.2, 'solver_ok': True, 'wrong_mode': False}

    def set_mode(request, response):
        nonlocal mode
        mode = request.control_mode
        response.success = True
        return response

    service = node.create_service(SetControlMode, '/drone0/controller/set_control_mode', set_mode)
    platform_pub = node.create_publisher(PlatformInfo, '/drone0/platform/info', 10)
    controller_pub = node.create_publisher(ControllerInfo, '/drone0/controller/info', 10)
    twist_pub = node.create_publisher(TwistStamped, '/drone0/self_localization/twist',
                                      qos_profile_sensor_data)
    frame_pub = node.create_publisher(ControlFrame, '/drone0/external_acro/control',
                                      qos_profile_sensor_data)
    broadcaster = TransformBroadcaster(node)

    def session_callback(msg):
        sessions.append(msg)
        state['session'] = msg

    sub_session = node.create_subscription(ControlSession, '/drone0/external_acro/session',
        session_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    sub_rate = node.create_subscription(TwistStamped, '/drone0/motion_reference/twist',
        rates.append, qos_profile_sensor_data)
    sub_thrust = node.create_subscription(Thrust, '/drone0/motion_reference/thrust',
        thrusts.append, qos_profile_sensor_data)

    node.create_subscription(PoseStamped, "/drone0/motion_reference/pose", poses.append, qos_profile_sensor_data)

    def tick():
        stamp = node.get_clock().now().to_msg()
        info = PlatformInfo()
        info.header.stamp = stamp
        info.connected = info.armed = info.offboard = True
        info.status.state = PlatformStatus.FLYING
        platform_pub.publish(info)
        ctrl = ControllerInfo()
        ctrl.header.stamp = stamp
        ctrl.input_control_mode = mode
        controller_pub.publish(ctrl)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'earth'
        transform.child_frame_id = 'drone0/base_link'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = 'earth'
        twist_pub.publish(twist)
        session = state['session']
        if state['publish'] and session and session.active:
            msg = ControlFrame()
            msg.header.stamp = stamp
            msg.header.frame_id = 'drone0/base_link'
            msg.session_id = session.session_id
            msg.revision = session.revision + state['revision_offset']
            state['sequence'] += 1
            msg.sequence = state['sequence']
            msg.body_rates.x, msg.body_rates.y, msg.body_rates.z = state['rate_x'], -0.3, 0.4
            if attitude_mode:
                msg.control_mode = ControlFrame.ATTITUDE
                msg.attitude_frame_id = 'earth'
                msg.attitude.w = 1.0 if state['rate_x'] < 3 else 2.0
                msg.body_rates.x = float('nan')  # Inactive mode field is intentionally ignored.
            else:
                msg.attitude.w = float('nan')  # Likewise for legacy angular-rate clients.
            if state['wrong_mode']:
                msg.control_mode = 255
            msg.thrust_newtons = 6.0
            msg.last_successful_solve = stamp
            msg.solver_ok = state['solver_ok']
            frame_pub.publish(msg)

    timer = node.create_timer(0.01, tick)

    def until(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        assert predicate(), 'timed out waiting for interface assertion'

    def spin_for(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)

    proc = None
    try:
        proc = subprocess.Popen([
            'ros2', 'run', 'uav_follow_reference_acro', 'follow_reference_acro_node',
            '--ros-args', '-r', '__ns:=/drone0',
            '--params-file', '/repo/src/behavior/follow_reference/config/external_acro.yaml',
            '-p', 'external_acro.control_mode:=' + ('attitude' if attitude_mode else 'body_rates'),
            '-p', 'external_acro.output_enabled:=' + ('false' if '--disabled' in sys.argv else 'true'),
        ], start_new_session=True)
        client = ActionClient(node, FollowReference, '/drone0/FollowReferenceBehavior')
        until(lambda: client.server_is_ready())
        spin_for(1.0)

        def goal(yaw_mode=YawMode.FIXED_YAW):
            out = FollowReference.Goal()
            out.target_pose.header.frame_id = 'earth'
            out.target_pose.point.x = 2.0
            out.yaw.mode = yaw_mode
            out.yaw.angle = 0.5
            out.max_speed_x, out.max_speed_y, out.max_speed_z = 1.0, 0.7, 0.5
            return out

        # Unsupported semantics are rejected rather than silently ignored.
        rejected = client.send_goal_async(goal(YawMode.YAW_FROM_TOPIC))
        until(rejected.done)
        assert not rejected.result().accepted
        if '--disabled' in sys.argv:
            rejected = client.send_goal_async(goal())
            until(rejected.done)
            assert not rejected.result().accepted
            assert not rates and not thrusts and not sessions
            print('PASS: output-disabled host rejects goals and publishes no commands')
            return
        sent = client.send_goal_async(goal())
        until(sent.done)
        handle = sent.result()
        assert handle.accepted
        until(lambda: bool(sessions))
        assert sessions[-1].target.point.x == 2.0
        assert sessions[-1].yaw.angle == 0.5
        assert abs(sessions[-1].max_speed.y - 0.7) < 1e-6

        # Acknowledgement from a different revision must not switch mode or publish.
        state['revision_offset'] = 1
        state['publish'] = True
        spin_for(0.25)
        assert not rates and not thrusts
        assert mode.control_mode != ControlMode.BODY_RATES
        state['revision_offset'] = 0
        state['rate_x'] = 10.0
        spin_for(0.15)
        assert not rates and not thrusts
        state['rate_x'] = 0.2
        state['solver_ok'] = False
        spin_for(0.15)
        assert not rates and not thrusts
        state['solver_ok'] = True
        state['wrong_mode'] = True
        spin_for(.1)
        assert not rates and not thrusts and not poses
        state['wrong_mode'] = False
        if attitude_mode:
            until(lambda: len(poses) >= 5 and len(thrusts) >= 5)
            assert mode.control_mode == ControlMode.ATTITUDE
            assert mode.yaw_mode == ControlMode.YAW_ANGLE
            assert not rates
            assert poses[-1].header.frame_id == 'earth'
            assert poses[-1].pose.orientation.w == 1.
            state['publish'] = False
            ended = handle.get_result_async()
            until(ended.done)
            until(lambda: mode.control_mode == ControlMode.HOVER)
            spin_for(.1)
            count = len(poses); spin_for(.15); assert len(poses) == count
            print('PASS: attitude host mode, quaternion/frame output, invalid quaternion/revision/solver rejection and loss hold')
            return
        until(lambda: len(rates) >= 5 and len(thrusts) >= 5)
        assert mode.control_mode == ControlMode.BODY_RATES
        assert mode.yaw_mode == ControlMode.YAW_SPEED
        assert abs(rates[-1].twist.angular.y + 0.3) < 1e-6
        assert thrusts[-1].thrust == 6.0
        assert rates[-1].header.frame_id == 'drone0/base_link'

        # Mode feedback and control remain live through sustained execution.
        active_result = handle.get_result_async()
        spin_for(1.0)
        assert not active_result.done(), 'healthy feedback/control aborted'

        # Stale stream aborts and requests hold rather than replaying forever.
        state['publish'] = False
        result = handle.get_result_async()
        until(result.done)
        assert not result.result().result.follow_reference_success
        until(lambda: mode.control_mode == ControlMode.HOVER)
        until(lambda: sessions and not sessions[-1].active)
        spin_for(0.1)
        previous_count = len(rates)
        spin_for(0.15)
        assert len(rates) == previous_count

        # A new action gets a new session. Cancel revokes further stream output.
        previous_id = sessions[-1].session_id
        sent = client.send_goal_async(goal())
        until(sent.done)
        handle = sent.result()
        assert handle.accepted
        state['publish'] = True
        until(lambda: sessions[-1].session_id != previous_id)
        until(lambda: len(rates) > previous_count + 3)

        # Modify maps a new target and rejects old-revision output during handoff.
        modify = node.create_client(FollowReference.Impl.SendGoalService,
            '/drone0/FollowReferenceBehavior/_behavior/modify')
        until(modify.service_is_ready)
        request = FollowReference.Impl.SendGoalService.Request()
        request.goal = goal()
        request.goal.target_pose.point.x = 3.0
        old_revision = sessions[-1].revision
        state['revision_offset'] = -1
        changed = modify.call_async(request)
        until(changed.done)
        assert changed.result().accepted
        until(lambda: sessions[-1].revision > old_revision)
        assert sessions[-1].target.point.x == 3.0
        spin_for(0.05)
        previous_count = len(rates)
        spin_for(0.15)
        assert len(rates) == previous_count
        state['revision_offset'] = 0
        until(lambda: len(rates) > previous_count + 3)

        pause = node.create_client(Trigger, '/drone0/FollowReferenceBehavior/_behavior/pause')
        resume = node.create_client(Trigger, '/drone0/FollowReferenceBehavior/_behavior/resume')
        until(pause.service_is_ready)
        paused = pause.call_async(Trigger.Request())
        until(paused.done)
        assert paused.result().success
        until(lambda: not sessions[-1].active)
        until(resume.service_is_ready)
        resumed = resume.call_async(Trigger.Request())
        until(resumed.done)
        assert not resumed.result().success
        canceled = handle.cancel_goal_async()
        until(canceled.done)
        assert canceled.result().goals_canceling
        until(lambda: not sessions[-1].active)
        until(lambda: mode.control_mode == ControlMode.HOVER)
        spin_for(0.1)
        previous_count = len(rates)
        spin_for(0.15)
        assert len(rates) == previous_count
        print('PASS: real host/plugin; goal mapping, unsupported yaw, revision rejection, '
              'bounds, solver failure, paired output, stale abort, new session, modify, '
              'pause/resume rejection and cancellation')
    finally:
        if proc is not None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
