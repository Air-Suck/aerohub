"""ROS-only orchestration. Motion references are exclusively owned by Behavior."""
import copy
from collections import deque
import json
import math
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from as2_msgs.msg import ControllerInfo, ControlMode
from uav_inference_interfaces.srv import InferYopo
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal
from uav_planner_interfaces.msg import TimedTrajectoryPoint
from .core import Curve, Timeline


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def set_time(stamp, value):
    ns = round(value * 1e9)
    stamp.sec, stamp.nanosec = divmod(ns, 1000000000)


def vector(msg):
    return np.array([msg.x, msg.y, msg.z])


def assign(msg, values):
    msg.x, msg.y, msg.z = map(float, values)


class YopoPlanner(Node):
    def __init__(self):
        super().__init__('yopo_planner')
        def param(name, value):
            return self.declare_parameter(name, value).value
        self.vehicle = param('vehicle_id', 'drone0')
        self.frame = param('fixed_frame', self.vehicle + '/odom')
        self.optical_frame = param('optical_frame', self.vehicle + '/camera/front_center_optical')
        self.aligned_frame = self.vehicle + '/yopo_aligned'
        self.lead = param('handoff_lead_s', .2)
        self.timeout = param('inference_timeout_s', .1)
        self.reserve = param('submission_reserve_s', .05)
        self.age = param('max_observation_age_s', .3)
        self.skew = param('max_observation_skew_s', .05)
        self.max_speed = param('max_speed', 6.)
        self.camera_pitch = param('camera_pitch_deg', 0.)
        # Fail closed unless deployment explicitly identifies the measurement model.
        self.depth_model = param('depth_model', 'unverified')
        if not all(math.isfinite(x) and x > 0 for x in
                   (self.lead, self.timeout, self.reserve, self.age, self.skew, self.max_speed)):
            raise ValueError('timing and speed parameters must be finite and positive')
        if self.lead < self.timeout + self.reserve or self.lead > .3:
            raise ValueError('handoff lead must cover timeout+reserve and fit adapter future bound')
        if not math.isfinite(self.camera_pitch):
            raise ValueError('camera pitch must be finite')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rotation_bc = Rotation.from_euler('Y', self.camera_pitch, degrees=True).as_matrix()
        self.pose = self.twist = self.depth = self.info = None
        self.pose_history, self.twist_history = deque(maxlen=200), deque(maxlen=200)
        self.pending_twists = deque(maxlen=200)
        self.goal = self.queued_goal = None
        self.timeline = Timeline()
        self.generation = 0
        self.inference = self.execution = None
        self.goal_handle = None
        self.failures = 0
        self.mode = None
        self.mode_at = 0.
        self.state = 'IDLE'
        self.reason = ''
        self.last_clock = None
        self.last_image = None
        self.sync_missing_since = None
        self.stop_future = None
        self.stop_ack = False
        self.settled_since = None
        self.status = self.create_publisher(String, 'yopo/status', 10)
        self.events = self.create_publisher(String, 'yopo/events', 100)
        self.create_subscription(PoseStamped, 'self_localization/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'self_localization/twist', self.on_twist, qos_profile_sensor_data)
        self.create_subscription(Image, 'yopo/depth', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'yopo/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'yopo/goal', self.on_goal, 1)
        self.create_subscription(ControllerInfo, 'controller/info', self.on_mode, qos_profile_sensor_data)
        self.create_service(Trigger, 'yopo/cancel', self.cancel)
        self.infer_client = self.create_client(InferYopo, 'inference/yopo/infer')
        self.action = ActionClient(self, FollowTimedPath, 'follow_timed_path')
        self.modify = self.create_client(FollowTimedPath_SendGoal, 'follow_timed_path/_behavior/modify')
        self.stopper = self.create_client(Trigger, 'follow_timed_path/_behavior/stop')
        self.create_timer(.02, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def fixed_transform(self, header):
        if header.frame_id == self.frame:
            return np.eye(3), np.zeros(3)
        if header.frame_id not in ('earth', self.vehicle+'/base_link'):
            raise ValueError('unsupported localization frame')
        transform = self.tf_buffer.lookup_transform(
            self.frame, header.frame_id, Time.from_msg(header.stamp)).transform
        q = transform.rotation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), vector(transform.translation)

    def on_pose(self, msg):
        if self.pose is not None and seconds(msg.header.stamp) <= seconds(self.pose.header.stamp):
            return
        try:
            rotation, translation = self.fixed_transform(msg.header)
            converted = copy.deepcopy(msg)
            assign(converted.pose.position, rotation @ vector(msg.pose.position)+translation)
            q = msg.pose.orientation
            orientation = Rotation.from_matrix(rotation) * Rotation.from_quat([q.x, q.y, q.z, q.w])
            converted.pose.orientation.x, converted.pose.orientation.y, converted.pose.orientation.z, converted.pose.orientation.w = map(float, orientation.as_quat())
            converted.header.frame_id = self.frame
            self.pose = converted
            self.pose_history.append(converted)
        except (TransformException, ValueError):
            pass

    def on_twist(self, msg):
        if not self.convert_twist(msg):
            self.pending_twists.append(msg)

    def convert_twist(self, msg):
        if self.twist is not None and seconds(msg.header.stamp) <= seconds(self.twist.header.stamp):
            return True
        try:
            if msg.header.frame_id == self.vehicle+'/base_link':
                # Native AS2 publishTwist emits the authority pose with the EXACT
                # same measurement stamp. Use that body attitude, not a delayed
                # duplicate dynamic TF stream or a latest-time approximation.
                pose = next((p for p in reversed(self.pose_history)
                             if p.header.stamp == msg.header.stamp), None)
                if pose is None:
                    return False
                q = pose.pose.orientation
                rotation = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            else:
                rotation, _ = self.fixed_transform(msg.header)
            converted = copy.deepcopy(msg)
            assign(converted.twist.linear, rotation @ vector(msg.twist.linear))
            assign(converted.twist.angular, rotation @ vector(msg.twist.angular))
            converted.header.frame_id = self.frame
            self.twist = converted
            self.twist_history.append(converted)
            return True
        except (TransformException, ValueError):
            return False

    def on_depth(self, msg):
        if msg.header.frame_id == self.optical_frame:
            self.depth = msg  # latest-only mailbox

    def on_info(self, msg):
        if msg.header.frame_id == self.optical_frame:
            self.info = msg

    def on_mode(self, msg):
        self.mode = msg.input_control_mode.control_mode
        self.mode_at = self.now_s()

    def geometry_ready(self):
        if self.depth_model != 'axial' or self.info is None or self.depth is None:
            return False
        try:
            transform = self.tf_buffer.lookup_transform(
                self.vehicle+'/base_link', self.optical_frame, Time.from_msg(self.depth.header.stamp)).transform
            q = transform.rotation
            actual = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            aligned_optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
            if (np.linalg.norm(vector(transform.translation)) > 1e-6 or
                    not np.allclose(actual, self.rotation_bc @ aligned_optical, atol=1e-6)):
                return False
        except (TransformException, ValueError):
            return False
        c = self.info
        if c.width != self.depth.width or c.height != self.depth.height or c.k[0] <= 0 or c.k[4] <= 0:
            return False
        hfov = math.degrees(2*math.atan(c.width/(2*c.k[0])))
        vfov = math.degrees(2*math.atan(c.height/(2*c.k[4])))
        return (abs(hfov-90) < 1 and abs(vfov-60) < 1 and
                abs(c.k[2]-c.width/2) < 1 and abs(c.k[5]-c.height/2) < 1 and
                all(abs(d) < 1e-9 for d in c.d))

    def observation_pair(self):
        if self.depth is None or not self.pose_history or not self.twist_history:
            return None
        epoch = seconds(self.depth.header.stamp)
        pose = min(self.pose_history, key=lambda m: abs(seconds(m.header.stamp)-epoch))
        twist = min(self.twist_history, key=lambda m: abs(seconds(m.header.stamp)-epoch))
        stamps = [epoch, seconds(pose.header.stamp), seconds(twist.header.stamp)]
        return (pose, twist) if max(stamps)-min(stamps) <= self.skew else None

    def inputs_fresh(self, now):
        if self.pose is None or self.twist is None or not self.geometry_ready():
            return False
        stamps = [seconds(m.header.stamp) for m in (self.pose, self.twist, self.depth)]
        values = np.r_[vector(self.pose.pose.position), vector(self.twist.twist.linear)]
        # Synchronize against the bounded history, not the newest pose on every timer tick.
        return np.isfinite(values).all() and all(0 <= now-t <= self.age for t in stamps)

    def observations_ready(self, now):
        return self.inputs_fresh(now) and self.observation_pair() is not None

    def on_goal(self, msg):
        if msg.header.frame_id != self.frame or not np.isfinite(vector(msg.pose.position)).all():
            self.get_logger().error('Goal requires finite fixed ENU coordinates')
            return
        if self.state == 'FAULT':
            self.get_logger().error('Stop recovery required before restarting planner')
            return
        if self.state in ('RUNNING', 'STOPPING'):
            self.queued_goal = copy.deepcopy(msg)
            self.stop('GOAL_CHANGED')
            return
        if self.inputs_fresh(self.now_s()) and self.ready() and self.observation_pair() is None:
            self.queued_goal = copy.deepcopy(msg)
            return
        if not self.observations_ready(self.now_s()) or not self.ready():
            self.publish_status('NOT_READY', 'sensor geometry, observations or services unavailable')
            return
        self.begin(msg)

    def ready(self):
        return (self.infer_client.service_is_ready() and self.action.server_is_ready()
                and self.modify.service_is_ready() and self.stopper.service_is_ready())

    def begin(self, msg):
        self.generation += 1
        self.queued_goal = None
        self.goal = vector(msg.pose.position)
        self.task = 'yopo-' + uuid.uuid4().hex
        self.timeline = Timeline()
        self.goal_handle = None
        self.last_image = None
        self.failures = 0
        self.state = 'RUNNING'
        self.sync_missing_since = None
        self.reason = ''

    def cancel(self, request, response):
        self.queued_goal = None
        self.stop('CANCELED')
        response.success = self.state != 'FAULT'
        response.message = 'stop requested; await yopo/status STOPPED and speed gate'
        return response

    def publish_status(self, state=None, reason=None):
        self.status.publish(String(data=json.dumps(dict(
            state=state or self.state, reason=self.reason if reason is None else reason,
            ready=self.observations_ready(self.now_s()) and self.ready(),
            task_id=getattr(self, 'task', ''), control_mode=self.mode,
            stop_ack=self.stop_ack, input_ages={name:self.now_s()-seconds(msg.header.stamp)
                for name,msg in [('pose',self.pose),('twist',self.twist),('depth',self.depth)] if msg is not None},
            synchronized=self.observation_pair() is not None, pending_twist_transforms=len(self.pending_twists),
            revision=self.timeline.revision, effective_revision=self.timeline.effective))))

    def stop(self, reason):
        if self.state in ('IDLE', 'STOPPED', 'STOPPING', 'FAULT'):
            return
        self.generation += 1  # invalidates inference and execution responses
        self.state, self.reason = 'STOPPING', reason
        self.stop_started = time.monotonic()
        self.stop_ack = False
        self.settled_since = None
        self.stop_future = None
        self.request_stop()

    def request_stop(self):
        if self.stopper.service_is_ready():
            self.stop_future = self.stopper.call_async(Trigger.Request())

    def stop_tick(self, now):
        # Reissue after any old submission completes: stop must follow late accepts.
        if self.stop_future is not None and self.stop_future.done():
            try:
                self.stop_ack = self.stop_future.result().success
            except Exception:
                self.stop_ack = False
            self.stop_future = None
        unresolved = self.execution is not None and not self.execution['future'].done()
        if self.stop_ack and not unresolved:
            if self.execution is not None:
                self.execution = None
                self.stop_ack = False
                self.request_stop()
                return
        fresh = (self.twist is not None and self.pose is not None and
                 all(0 <= now-seconds(m.header.stamp) <= self.age for m in (self.pose, self.twist)))
        mode_ok = self.mode in (ControlMode.HOVER, ControlMode.POSITION) and 0 <= now-self.mode_at <= .5
        # The stop service confirms hover submission; controller mode and motion confirm effect.
        if self.stop_ack and not unresolved and fresh and mode_ok and np.linalg.norm(vector(self.twist.twist.linear)) < .2:
            if self.settled_since is None:
                self.settled_since = now
            if now-self.settled_since >= 1 and self.inference is None:
                self.state = 'STOPPED'
                self.timeline = Timeline()
                if self.queued_goal is not None and self.observations_ready(now) and self.ready():
                    goal, self.queued_goal = self.queued_goal, None
                    self.begin(goal)
        else:
            self.settled_since = None
        if self.state == 'STOPPING' and time.monotonic()-self.stop_started > 10:
            self.state, self.reason = 'FAULT', 'STOP_TIMEOUT:' + self.reason

    def tick(self):
        now = self.now_s()
        pending, self.pending_twists = self.pending_twists, deque(maxlen=200)
        for measurement in pending:
            if abs(now-seconds(measurement.header.stamp)) <= self.age and not self.convert_twist(measurement):
                self.pending_twists.append(measurement)
        if self.last_clock is not None and now < self.last_clock and self.state == 'RUNNING':
            self.stop('CLOCK_RESET')
            self.pose_history.clear(); self.twist_history.clear(); self.pending_twists.clear()
            self.pose = self.twist = None
            self.queued_goal = None
        self.last_clock = now
        # Drain the real outstanding inference even after timeout/cancel; never overlap.
        if self.inference is not None:
            context = self.inference
            if context['future'].done():
                self.inference = None
                if context['generation'] == self.generation and not context['expired']:
                    self.inferred(context, now)
            elif not context['expired'] and time.monotonic()-context['sent'] > self.timeout:
                context['expired'] = True
                self.failure('INFERENCE_TIMEOUT')
        if self.state == 'STOPPING':
            self.stop_tick(now)
        if self.state in ('IDLE', 'STOPPED') and self.queued_goal is not None and self.observations_ready(now) and self.ready():
            goal, self.queued_goal = self.queued_goal, None
            self.begin(goal)
        if self.state == 'RUNNING':
            if not self.inputs_fresh(now):
                self.stop('STALE_OR_INVALID_INPUT')
            elif np.linalg.norm(vector(self.pose.pose.position)-self.goal) < 5:
                self.stop('ARRIVED_5M')
            elif self.timeline.latest and self.timeline.latest.end-now < self.lead+self.reserve:
                self.stop('COVERAGE_EXHAUSTED')
            elif self.execution is not None and now >= self.execution['curve'].start-self.reserve/2:
                self.stop('EXECUTION_ACK_TIMEOUT')
            elif self.observation_pair() is None:
                if self.sync_missing_since is None:
                    self.sync_missing_since = time.monotonic()
                elif time.monotonic()-self.sync_missing_since > self.age:
                    self.stop('UNSYNCHRONIZED_INPUT')
            elif self.can_infer():
                self.sync_missing_since = None
                self.request_inference(now)
            else:
                self.sync_missing_since = None
        self.publish_status()

    def can_infer(self):
        if self.inference is not None or self.execution is not None or not self.ready() or self.observation_pair() is None:
            return False
        if self.timeline.revision and (self.timeline.effective != self.timeline.revision or
                self.mode != ControlMode.TRAJECTORY or self.now_s()-self.mode_at > .5):
            return False
        return seconds(self.depth.header.stamp) != self.last_image

    def failure(self, reason):
        if self.state != 'RUNNING':
            return
        self.failures += 1
        self.get_logger().warning(reason)
        if self.failures >= 3:
            self.stop(reason)

    def request_inference(self, now):
        epoch = now + self.lead
        measured_pose, _ = self.observation_pair()
        q = measured_pose.pose.orientation
        try:
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix() @ self.rotation_bc
            if self.timeline.latest:
                initial, yaw = self.timeline.reference(epoch)
            else:
                initial = np.array([vector(self.pose.pose.position), vector(self.twist.twist.linear), np.zeros(3)])
                yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX')[0]
        except ValueError as error:
            self.stop('INVALID_REFERENCE:' + str(error))
            return
        request = InferYopo.Request()
        request.request_id = self.task + ':' + uuid.uuid4().hex
        request.depth = copy.deepcopy(self.depth)
        request.observation_header = copy.deepcopy(measured_pose.header)
        request.state_header.frame_id = self.aligned_frame
        set_time(request.state_header.stamp, epoch)
        assign(request.velocity, rotation.T @ initial[1])
        assign(request.acceleration, rotation.T @ initial[2])
        assign(request.relative_goal, rotation.T @ (self.goal-initial[0]))
        self.last_image = seconds(self.depth.header.stamp)
        self.inference = dict(future=self.infer_client.call_async(request), generation=self.generation,
                              sent=time.monotonic(), expired=False, request=request,
                              epoch=epoch, initial=initial, yaw=yaw, rotation=rotation)

    def inferred(self, context, now):
        try:
            result = context['future'].result()
            request = context['request']
            if not result.success:
                raise ValueError(result.error_code+':'+result.message)
            if (result.request_id != request.request_id or
                    result.header != request.state_header or result.observation_header != request.observation_header):
                raise ValueError('inference rejected or response identity mismatch')
            if (not self.inputs_fresh(now) or now-seconds(request.depth.header.stamp) > self.age or
                    now-seconds(request.observation_header.stamp) > self.age or
                    time.monotonic()-context['sent'] > self.timeout or context['epoch']-now < self.reserve):
                raise ValueError('inference result missed submission budget')
            if not result.model_version or not result.candidates or not math.isfinite(result.duration_s) or result.duration_s <= 0:
                raise ValueError('invalid model identity or duration')
            scores = [c.score for c in result.candidates]
            if not np.isfinite(scores).all() or result.best_index != int(np.argmin(scores)):
                raise ValueError('invalid candidate scores')
            candidate = result.candidates[result.best_index]
            terminal = np.array([vector(candidate.position), vector(candidate.velocity), vector(candidate.acceleration)]) @ context['rotation'].T
            terminal[0] += context['initial'][0]
            curve = Curve.create(context['epoch'], context['initial'], terminal, result.duration_s, context['yaw'], self.goal)
            self.submit(curve, now)
        except Exception as error:
            self.failure('INFERENCE_RESULT:' + str(error))

    def submit(self, curve, now):
        revision = self.timeline.propose(curve)
        goal = FollowTimedPath.Goal()
        path = goal.path
        path.interface_version, path.task_id, path.vehicle_id = 1, self.task, self.vehicle
        path.revision, path.frame_id, path.valid_fields = revision, self.frame, 7
        path.max_speed, path.is_final = float(self.max_speed), False
        set_time(path.generated_at, now)
        set_time(path.valid_from, curve.start)
        for t in curve.times:
            point = TimedTrajectoryPoint()
            pva, yaw = curve.sample(curve.start+t)
            set_time(point.time_from_start, t)
            assign(point.position, pva[0]); assign(point.velocity, pva[1]); assign(point.acceleration, pva[2])
            point.yaw = float(yaw)
            path.points.append(point)
        if curve.start-self.now_s() < self.reserve:
            self.timeline.acknowledge(False)
            raise ValueError('serialization missed submission budget')
        generation = self.generation
        if revision == 1:
            future = self.action.send_goal_async(goal, feedback_callback=lambda m: self.feedback(m, generation))
        else:
            request = FollowTimedPath_SendGoal.Request()
            request.goal = goal
            future = self.modify.call_async(request)
        self.events.publish(String(data=json.dumps(dict(
            event='submit', task=self.task, revision=revision, generated=now,
            start=curve.start, duration=curve.duration,
            coefficients=curve.coefficients.tolist(), times=curve.times.tolist(), yaws=curve.yaws.tolist()))))
        self.execution = dict(future=future, curve=curve)
        future.add_done_callback(lambda f: self.accepted(f, revision, generation))

    def accepted(self, future, revision, generation):
        try:
            response = future.result()
            if generation != self.generation:
                if revision == 1 and response.accepted:
                    response.cancel_goal_async()
                return
            self.execution = None
            accepted = response.accepted
            self.timeline.acknowledge(accepted)
            self.events.publish(String(data=json.dumps(dict(event='ack', task=self.task, revision=revision, accepted=accepted, at=self.now_s()))))
            if not accepted:
                self.failure('EXECUTION_REJECTED')
                return
            self.failures = 0
            if revision == 1:
                self.goal_handle = response
                response.get_result_async().add_done_callback(lambda f: self.result(f, generation))
        except Exception as error:
            if generation == self.generation:
                self.stop('EXECUTION_UNCERTAIN:' + str(error))

    def feedback(self, message, generation):
        if generation != self.generation or self.state != 'RUNNING':
            return
        feedback = message.feedback
        if feedback.task_id == self.task:
            self.timeline.feedback(feedback.revision, feedback.elapsed, self.now_s(), feedback.waiting_for_start)

    def result(self, future, generation):
        if generation == self.generation and self.state == 'RUNNING':
            try:
                reason = future.result().result.reason
            except Exception as error:
                reason = str(error)
            self.stop('EXECUTION_ENDED:' + reason)


def main(args=None):
    rclpy.init(args=args)
    node = YopoPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.queued_goal = None
        node.stop('SHUTDOWN')
        deadline = time.monotonic()+11
        while rclpy.ok() and node.state == 'STOPPING' and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
