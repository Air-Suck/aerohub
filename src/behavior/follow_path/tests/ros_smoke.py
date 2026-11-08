#!/usr/bin/env python3
"""Real ROS Actions against the compiled extension; fake controller, NOT SITL."""
import copy
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import FollowPath
from as2_msgs.msg import PoseWithID, YawMode, PlatformInfo, PlatformStatus, TrajectorySetpoints
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal
from uav_planner_interfaces.msg import TimedTrajectoryPoint


def stamp(value, msg):
    msg.sec = int(value)
    msg.nanosec = int((value-int(value))*1e9)


class Test(Node):
    def __init__(self):
        super().__init__("followpath_test")
        self.x = 0.0
        self.references = []
        self.trajectory_references = []
        self.pub = self.create_publisher(PoseStamped, "/drone0/self_localization/pose", 10)
        self.info = self.create_publisher(PlatformInfo, "/drone0/platform/info", 10)
        self.twist = self.create_publisher(TwistStamped, "/drone0/self_localization/twist", qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.sub = self.create_subscription(PoseStamped, "/drone0/motion_reference/pose", self.reference, qos_profile_sensor_data)
        self.trajectory_sub = self.create_subscription(TrajectorySetpoints,
            "/drone0/motion_reference/trajectory", self.trajectory_reference, qos_profile_sensor_data)
        self.mode = self.create_service(SetControlMode, "/drone0/controller/set_control_mode", self.set_mode)
        self.timer = self.create_timer(0.02, self.pose)
        self.action = ActionClient(self, FollowTimedPath, "/drone0/follow_timed_path")
        self.legacy = ActionClient(self, FollowPath, "/drone0/FollowPathBehavior")
        self.modify = self.create_client(FollowTimedPath_SendGoal, "/drone0/follow_timed_path/_behavior/modify")
        self.pause = self.create_client(Trigger, "/drone0/follow_timed_path/_behavior/pause")
        self.resume = self.create_client(Trigger, "/drone0/follow_timed_path/_behavior/resume")

    def set_mode(self, request, response):
        response.success = True
        return response

    def pose(self):
        msg = PoseStamped()
        msg.header.frame_id = "drone0/odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.x
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        info=PlatformInfo(); info.status.state=PlatformStatus.FLYING; self.info.publish(info)
        transform=TransformStamped(); transform.header.stamp=msg.header.stamp
        transform.header.frame_id="earth"; transform.child_frame_id="drone0/base_link"
        transform.transform.translation.x=self.x; transform.transform.rotation.w=1.0
        self.tf.sendTransform(transform)
        twist=TwistStamped(); twist.header.stamp=msg.header.stamp
        twist.header.frame_id="earth"; self.twist.publish(twist)

    def reference(self, msg):
        self.references.append((time.monotonic(), msg.pose.position.x))
        self.x = msg.pose.position.x

    def trajectory_reference(self, msg):
        self.trajectory_references.append(msg)
        self.x = msg.setpoints[0].position.x

    def spin(self, duration):
        until = time.monotonic()+duration
        while time.monotonic()<until:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, future, timeout=6):
        until = time.monotonic()+timeout
        while not future.done() and time.monotonic()<until:
            rclpy.spin_once(self, timeout_sec=0.02)
        assert future.done(), "future timeout"
        return future.result()

    def goal(self, final=True, duration=0.8, lead=0.5):
        goal = FollowTimedPath.Goal()
        p = goal.path
        p.interface_version=1; p.task_id="test"; p.vehicle_id="drone0"
        p.frame_id="drone0/odom"; p.revision=1; p.is_final=final; p.max_speed=1.0
        now=self.get_clock().now().nanoseconds/1e9
        stamp(now,p.generated_at); stamp(now+lead,p.valid_from)
        for t,x in ((0,self.x),(duration,self.x+0.4)):
            point=TimedTrajectoryPoint(); stamp(t,point.time_from_start)
            point.position.x=x; p.points.append(point)
        return goal

    def run(self):
        assert self.action.wait_for_server(timeout_sec=5)
        assert self.legacy.wait_for_server(timeout_sec=5)
        self.spin(0.5)
        goal=self.goal()
        handle=self.wait(self.action.send_goal_async(goal))
        assert handle.accepted
        # Original interface must be mutually exclusive with the timed goal.
        legacy=FollowPath.Goal(); legacy.header.frame_id="earth"; legacy.max_speed=1.0
        wp=PoseWithID(); wp.id="0"; wp.pose.orientation.w=1.0; legacy.path=[wp]
        legacy.yaw.mode=YawMode.KEEP_YAW
        assert not self.wait(self.legacy.send_goal_async(legacy)).accepted
        self.spin(0.15)
        assert not self.references, "reference emitted before common start"
        result=self.wait(handle.get_result_async())
        assert result.result.success, result.result.reason
        assert abs(self.x-0.4)<0.26
        self.spin(0.1)
        late=self.goal(lead=-0.1)
        assert not self.wait(self.action.send_goal_async(late)).accepted
        # Rolling replacement at a future point of the first linear curve.
        goal=self.goal(final=False,duration=2.0)
        handle=self.wait(self.action.send_goal_async(goal)); assert handle.accepted
        update=copy.deepcopy(goal)
        update.path.revision=2; update.path.is_final=True
        start=goal.path.valid_from.sec+goal.path.valid_from.nanosec/1e9
        stamp(start+1.0,update.path.valid_from)
        update.path.points[0].position.x+=0.2
        update.path.points[1].position.x+=0.2
        request=FollowTimedPath_SendGoal.Request(); request.goal=update
        assert self.wait(self.modify.call_async(request)).accepted
        assert not self.wait(self.modify.call_async(request)).accepted, "duplicate revision accepted"
        result=self.wait(handle.get_result_async())
        assert result.result.success and result.result.revision==2
        self.spin(0.1)
        handle=self.wait(self.action.send_goal_async(self.goal(duration=3)))
        assert handle.accepted
        assert self.wait(self.pause.call_async(Trigger.Request())).success
        assert not self.wait(self.resume.call_async(Trigger.Request())).success
        result_future=handle.get_result_async()
        self.wait(handle.cancel_goal_async())
        assert self.wait(result_future).status==5  # STATUS_CANCELED
        self.spin(0.1)
        handle=self.wait(self.action.send_goal_async(self.goal(final=False)))
        assert handle.accepted
        assert self.wait(handle.get_result_async()).result.reason=="TRAJECTORY_EXPIRED"
        self.spin(0.2)
        wp.pose.position.x=self.x+0.3
        handle=self.wait(self.legacy.send_goal_async(legacy))
        assert handle.accepted, "original FollowPath request rejected"
        assert self.wait(handle.get_result_async()).result.follow_path_success
        self.spin(0.2)
        goal=self.goal(); goal.path.valid_fields=7
        handle=self.wait(self.action.send_goal_async(goal)); assert handle.accepted
        assert self.wait(handle.get_result_async()).result.success
        assert self.trajectory_references, "derivatives did not reach trajectory motion handler"
        assert any(abs(msg.setpoints[0].twist.x)>0.01 for msg in self.trajectory_references)
        start=goal.path.valid_from.sec+goal.path.valid_from.nanosec/1e9
        duration=goal.path.points[-1].time_from_start.sec+goal.path.points[-1].time_from_start.nanosec/1e9
        x0=goal.path.points[0].position.x
        dx=goal.path.points[-1].position.x-x0
        for msg in self.trajectory_references:
            t=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            u=min(1.,max(0.,(t-start)/duration))
            p=x0+dx*(10*u**3-15*u**4+6*u**5)
            v=dx*(30*u**2-60*u**3+30*u**4)/duration
            a=dx*(60*u-180*u**2+120*u**3)/(duration**2)
            actual=msg.setpoints[0]
            assert abs(actual.position.x-p)<1e-5, "reference position does not match its sampling timestamp"
            assert abs(actual.twist.x-v)<1e-5, "reference velocity does not match its sampling timestamp"
            assert abs(actual.acceleration.x-a)<1e-4, "reference acceleration does not match its sampling timestamp"
        print("PASS: timed execution, future start, legacy exclusion/execution, stale rejection, rolling update, pause/replan, cancel, expiration, derivative feedforward")


if __name__=="__main__":
    rclpy.init()
    node=Test()
    command=["ros2","run","uav_follow_path_execution","follow_path_extended_node","--ros-args",
             "--params-file",str(Path(get_package_share_directory("as2_behaviors_motion"))/"follow_path_behavior/config/config_default.yaml"),
             "-r","__ns:=/drone0","-p","plugin_name:=follow_path_plugin_position",
             "-p","follow_path_speed:=1.0","-p","follow_path_threshold:=0.2",
             "-p","tf_timeout_threshold:=0.1"]
    process=subprocess.Popen(command,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    try:
        node.run()
    finally:
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
