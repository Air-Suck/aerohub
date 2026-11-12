"""Real DDS/action transport with deterministic model/Behavior doubles, no flight."""
import copy
import json
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from as2_msgs.msg import ControllerInfo, ControlMode
from uav_inference_interfaces.srv import InferYopo
from uav_inference_interfaces.msg import YopoCandidate
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal
from uav_yopo_planner.node import YopoPlanner, set_time, seconds


class Fixture(Node):
    def __init__(self):
        super().__init__('fixture')
        self.dynamic = TransformBroadcaster(self)
        self.broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped(); tf.header.frame_id = 'drone0/base_link'
        tf.child_frame_id = 'drone0/yopo_front_optical'
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = -.5, .5, -.5, .5
        self.broadcaster.sendTransform(tf)
        self.group = ReentrantCallbackGroup()
        self.ticks = 0
        self.calls = self.concurrent = self.max_concurrent = 0
        self.delay = 0.
        self.reject = False
        self.stopped = threading.Event()
        self.path = None
        self.accepted = []
        self.modifications = 0
        self.mode = ControlMode.HOVER
        self.create_service(InferYopo, 'inference/yopo/infer', self.infer, callback_group=self.group)
        self.create_service(FollowTimedPath_SendGoal, 'follow_timed_path/_behavior/modify', self.modify, callback_group=self.group)
        self.create_service(Trigger, 'follow_timed_path/_behavior/stop', self.stop, callback_group=self.group)
        self.server = ActionServer(self, FollowTimedPath, 'follow_timed_path', self.execute,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        self.pose_pub = self.create_publisher(PoseStamped, 'self_localization/pose', 1)
        self.twist_pub = self.create_publisher(TwistStamped, 'self_localization/twist', 1)
        self.image_pub = self.create_publisher(Image, 'yopo/depth', 1)
        self.info_pub = self.create_publisher(CameraInfo, 'yopo/camera_info', 1)
        self.mode_pub = self.create_publisher(ControllerInfo, 'controller/info', 1)
        self.goal_pub = self.create_publisher(PoseStamped, 'yopo/goal', 1)
        self.create_timer(.02, self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'drone0/odom'
        set_time(pose.header.stamp, seconds(stamp)-.08)
        pose.pose.orientation.w = 1.
        self.pose_pub.publish(pose)
        tf = TransformStamped(); tf.header.frame_id = 'drone0/odom'; tf.child_frame_id = 'drone0/base_link'
        set_time(tf.header.stamp, self.get_clock().now().nanoseconds/1e9-.12)
        tf.transform.rotation.w = 1.
        self.dynamic.sendTransform(tf)
        twist = TwistStamped(); twist.header = copy.deepcopy(pose.header)
        twist.header.frame_id = 'drone0/base_link'
        self.twist_pub.publish(twist)
        mode = ControllerInfo(); mode.input_control_mode.control_mode = self.mode
        self.mode_pub.publish(mode)
        self.ticks += 1
        if self.ticks % 10:
            return
        image = Image(); image.header.stamp = self.get_clock().now().to_msg(); image.header.frame_id = 'drone0/yopo_front_optical'
        image.width, image.height = 320, 185
        image.encoding, image.step = '32FC1', 1280
        image.data = np.full((185, 320), 20., dtype='<f4').tobytes()
        self.image_pub.publish(image)
        info = CameraInfo(); info.header = image.header
        info.width, info.height = image.width, image.height
        info.k = [160., 0., 159.5, 0., 160., 92., 0., 0., 1.]
        self.info_pub.publish(info)

    def infer(self, request, response):
        self.calls += 1; self.concurrent += 1
        self.max_concurrent = max(self.concurrent, self.max_concurrent)
        time.sleep(self.delay)
        self.concurrent -= 1
        response.request_id, response.header = request.request_id, request.state_header
        response.observation_header = request.observation_header
        response.success, response.model_version, response.device = True, 'fixture', 'cpu'
        response.duration_s = 1.7
        candidate = YopoCandidate(); candidate.position.x = .3
        response.candidates = [candidate]
        return response

    def execute(self, handle):
        self.path = handle.request.path
        self.accepted.append(copy.deepcopy(self.path))
        self.stopped.clear()
        while not self.stopped.is_set() and not handle.is_cancel_requested:
            now = self.get_clock().now().nanoseconds/1e9
            feedback = FollowTimedPath.Feedback()
            feedback.task_id, feedback.revision = self.path.task_id, self.path.revision
            feedback.elapsed = max(0., now-seconds(self.path.valid_from))
            feedback.waiting_for_start = now < seconds(self.path.valid_from)
            if not feedback.waiting_for_start:
                self.mode = ControlMode.TRAJECTORY
            handle.publish_feedback(feedback)
            time.sleep(.02)
        self.mode = ControlMode.HOVER
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return FollowTimedPath.Result(success=False, reason='STOPPED', revision=self.path.revision)

    def modify(self, request, response):
        self.modifications += 1
        response.accepted = not self.reject
        if response.accepted:
            self.path = request.goal.path
            self.accepted.append(copy.deepcopy(self.path))
        return response

    def stop(self, request, response):
        self.stopped.set(); self.mode = ControlMode.HOVER
        response.success = True
        return response


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/drone0'])
    fixture = Fixture()
    # Constructor parameters via ROS process arguments keep the production node unchanged.
    planner = YopoPlanner()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(fixture); executor.add_node(planner)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    def wait(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if predicate(): return
            time.sleep(.01)
        raise AssertionError((planner.state, planner.reason, planner.timeline.revision, fixture.calls))
    try:
        # Parameters are immutable planning inputs loaded here before any goal.
        planner.depth_model = 'axial'; planner.optical_frame = 'drone0/yopo_front_optical'
        wait(lambda: planner.observations_ready(planner.now_s()) and planner.ready())
        latest = planner.pose
        older = copy.deepcopy(latest); older.header.stamp.sec -= 1
        planner.on_pose(older)
        assert seconds(planner.pose.header.stamp) >= seconds(latest.header.stamp)
        goal = PoseStamped(); goal.header.frame_id = 'drone0/odom'; goal.pose.position.x = 20.
        fixture.goal_pub.publish(goal)
        wait(lambda: planner.timeline.revision >= 3)
        # Pending/rejected plans cannot become the accepted reference.
        fixture.reject = True
        accepted_count = len(fixture.accepted)
        revision = planner.timeline.revision
        wait(lambda: fixture.modifications >= accepted_count+1)
        assert planner.timeline.revision == revision
        # Stop on consecutive failure; real mode and speed confirmation gate must complete.
        wait(lambda: planner.state == 'STOPPED')
        assert fixture.max_concurrent == 1
        # New generation: delayed in-flight result after cancellation must not start execution.
        fixture.reject = False; fixture.delay = .15
        fixture.goal_pub.publish(goal)
        wait(lambda: planner.inference is not None)
        client = fixture.create_client(Trigger, 'yopo/cancel')
        future = client.call_async(Trigger.Request())
        wait(future.done)
        count = len(fixture.accepted)
        wait(lambda: planner.state == 'STOPPED')
        assert len(fixture.accepted) == count
        assert fixture.max_concurrent == 1
        # Arrival retains the upstream 5m condition and closes execution before landing.
        fixture.delay = 0.
        near = copy.deepcopy(goal); near.pose.position.x = 3.
        fixture.goal_pub.publish(near)
        wait(lambda: planner.state == 'STOPPING')
        wait(lambda: planner.state == 'STOPPED')
        assert planner.reason == 'ARRIVED_5M'
        repeated = client.call_async(Trigger.Request()); wait(repeated.done)
        assert repeated.result().success and planner.state == 'STOPPED'
        assert planner.reason == 'ARRIVED_5M'
        # A real Service future remains occupied until its delayed response completes.
        fixture.delay = .15
        fixture.goal_pub.publish(goal)
        wait(lambda: planner.state == 'RUNNING')
        wait(lambda: planner.state == 'STOPPED')
        assert planner.reason == 'INFERENCE_TIMEOUT'
        assert fixture.max_concurrent == 1
        fixture.delay = 0.
        fixture.goal_pub.publish(goal)
        wait(lambda: planner.state == 'RUNNING' and planner.timeline.revision >= 1)
        previous_task = planner.task
        replacement = copy.deepcopy(goal); replacement.pose.position.y = 1.
        fixture.goal_pub.publish(replacement)
        wait(lambda: planner.state == 'RUNNING' and planner.task != previous_task)
        wait(lambda: planner.timeline.revision >= 1)
        assert fixture.max_concurrent == 1
        # Inject a backward read into the business clock (not a real /clock transport test).
        actual_now = planner.now_s
        planner.now_s = lambda: actual_now()-1.
        wait(lambda: planner.state == 'STOPPING')
        planner.now_s = actual_now
        wait(lambda: planner.state == 'STOPPED')
        assert planner.reason == 'CLOCK_RESET'

        print(json.dumps(dict(result='passed', accepted_revisions=accepted_count,
                              rejection_preserves_timeline=True, canceled_response_discarded=True,
                              max_inference_inflight=fixture.max_concurrent, stop_gate=True)))
    finally:
        fixture.stopped.set()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        planner.destroy_node(); fixture.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
