"""Actual C++ Behavior: intentional final POSITION vs external mode takeover."""
import subprocess
import os
import signal
import time
import rclpy
from as2_msgs.msg import ControllerInfo,ControlMode
from rclpy.qos import qos_profile_sensor_data
from ros_smoke import Test
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


class Settling(Test):
    def __init__(self):
        self.current_mode=ControlMode.POSITION
        self.mode_requests=[]
        super().__init__()
        self.controller_pub=self.create_publisher(ControllerInfo,'/drone0/controller/info',10)

    def set_mode(self,request,response):
        self.current_mode=request.control_mode.control_mode
        self.mode_requests.append(self.current_mode)
        response.success=True
        return response

    def pose(self):
        super().pose()
        msg=ControllerInfo();msg.header.stamp=self.get_clock().now().to_msg()
        msg.input_control_mode.control_mode=self.current_mode
        msg.input_control_mode.yaw_mode=ControlMode.YAW_ANGLE
        self.controller_pub.publish(msg)

    def trajectory_reference(self,msg):
        self.trajectory_references.append(msg)
        self.x=msg.setpoints[0].position.x-.35  # Force final-position settling.

    def run(self):
        assert self.action.wait_for_server(timeout_sec=8)
        self.spin(.5)
        goal=self.goal(duration=.8);goal.path.valid_fields=3
        handle=self.wait(self.action.send_goal_async(goal));assert handle.accepted
        result=self.wait(handle.get_result_async())
        assert result.result.success,result.result.reason
        assert ControlMode.TRAJECTORY in self.mode_requests
        assert ControlMode.POSITION in self.mode_requests[self.mode_requests.index(ControlMode.TRAJECTORY)+1:]
        self.spin(.2)
        goal=self.goal(duration=2.);goal.path.valid_fields=3
        handle=self.wait(self.action.send_goal_async(goal));assert handle.accepted
        deadline=time.monotonic()+3
        while self.current_mode!=ControlMode.TRAJECTORY and time.monotonic()<deadline:
            self.spin(.02)
        assert self.current_mode==ControlMode.TRAJECTORY
        self.spin(.1)
        requests_before_takeover=len(self.mode_requests)
        self.current_mode=ControlMode.POSITION  # Unexpected takeover mid-trajectory.
        result=self.wait(handle.get_result_async())
        assert not result.result.success and result.result.reason=='CONTROL_MODE_LOST',result.result.reason
        assert ControlMode.TRAJECTORY not in self.mode_requests[requests_before_takeover:], 'reclaimed trajectory mode'
        print('PASS: final settling succeeds; unexpected mode takeover still aborts')


rclpy.init();node=Settling()
process=subprocess.Popen(['ros2','run','uav_follow_path_execution','follow_path_extended_node','--ros-args',
    '--params-file',str(Path(get_package_share_directory('as2_behaviors_motion'))/'follow_path_behavior/config/config_default.yaml'),
    '-r','__ns:=/drone0','-p','plugin_name:=follow_path_plugin_position','-p','follow_path_speed:=1.0',
    '-p','follow_path_threshold:=0.2','-p','tf_timeout_threshold:=0.1'],
    stdout=open('/tmp/behavior-terminal.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
try:node.run()
except BaseException:
    print(open('/tmp/behavior-terminal.log').read())
    raise
finally:
    try:os.killpg(process.pid,signal.SIGTERM)
    except ProcessLookupError:pass
    process.wait(timeout=10)
    node.destroy_node();rclpy.try_shutdown()
