import json,time,math,copy
from pathlib import Path
import rclpy
from as2_msgs.msg import ControlMode,PlatformStatus,TrajectorySetpoints,TrajectoryPoint
from as2_msgs.action import Takeoff,Land
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from rclpy.qos import qos_profile_sensor_data
from uav_as2_setpoint_adapter.super_mission import SuperMission
from mars_quadrotor_msgs.msg import PositionCommand
rclpy.init();n=SuperMission();results={'source':'controlled PositionCommand producer through the real SUPER adapter'};outputs=[]
source_running=False
source_position=None
command_pub=n.create_publisher(PositionCommand,'/planning/pos_cmd',1)
def publish_command():
 if not source_running or source_position is None:return
 m=PositionCommand();m.header.stamp=n.get_clock().now().to_msg()
 m.header.frame_id=n.get_parameter('frame_id').value
 m.position=copy.deepcopy(source_position);m.trajectory_flag=1;m.yaw=0.
 command_pub.publish(m)
n.create_timer(.02,publish_command)
n.create_subscription(TrajectorySetpoints,'actuator_command/trajectory',lambda m:outputs.append((time.monotonic(),m)),qos_profile_sensor_data)
def phase(s):
 open('/evidence/fault-phase','w').write(s)
 print(s,flush=True)
def mode_is(v):return n.controller is not None and n.controller.input_control_mode.control_mode==v
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
def activate():
 global source_running,source_position
 n.mode(ControlMode.TRAJECTORY)
 n.wait(lambda:mode_is(ControlMode.TRAJECTORY),.3)
 n.service(SetBool,'super_adapter/enable',SetBool.Request(data=True))
 source_position=copy.deepcopy(n.pose.pose.position);source_running=True
 count=len(outputs);n.wait(lambda:len(outputs)>count+5,1.)
def expect_loss(name):
 phase('STOP_'+name)
 n.wait(lambda:not mode_is(ControlMode.TRAJECTORY),180.)
 spin(.4);count=len(outputs)
 phase('RESUME_'+name)
 n.wait(lambda:Path('/evidence/continue-'+name).exists(),180.)
 spin(1.5)
 assert not mode_is(ControlMode.TRAJECTORY)
 assert len(outputs)==count,(name,len(outputs),count)
 results[name]={'hover_after_loss':True,'no_automatic_resume':True}
 phase(name+'_PASS')
try:
 n.wait(lambda:n.pose is not None and n.platform is not None)
 if n.platform.status.state != PlatformStatus.FLYING:
  phase("PREFLIGHT_SETTLE")
  settle=time.monotonic()+15
  n.wait(lambda:time.monotonic()>=settle,20.)
  if not n.platform.armed:n.service(SetBool,'set_arming_state',SetBool.Request(data=True))
  n.wait(lambda:n.platform.armed)
  if not n.platform.offboard:n.service(SetBool,'set_offboard_mode',SetBool.Request(data=True))
  n.wait(lambda:n.platform.offboard)
  n.action(Takeoff,'TakeoffBehavior',Takeoff.Goal(takeoff_height=1.,takeoff_speed=.5),'takeoff_success')
  n.wait(lambda:n.platform.status.state==PlatformStatus.FLYING)
 activate()
 source_running=False
 n.wait(lambda:not mode_is(ControlMode.TRAJECTORY),2.)
 spin(.3);count=len(outputs);source_running=True;spin(1.2)
 assert not mode_is(ControlMode.TRAJECTORY) and len(outputs)==count
 results['reference_source_exit']={'hover_after_loss':True,'no_automatic_resume':True}
 phase('SOURCE_PASS')
 activate();expect_loss('ADAPTER')
 # Source replacement is explicit: adapter disabled before injecting stale samples.
 n.service(SetBool,'super_adapter/enable',SetBool.Request(data=False))
 n.mode(ControlMode.TRAJECTORY);n.wait(lambda:mode_is(ControlMode.TRAJECTORY),.3)
 ref=n.create_publisher(TrajectorySetpoints,'motion_reference/trajectory',1)
 old=TrajectorySetpoints();old.header=n.pose.header;old.header.stamp.sec-=2
 point=TrajectoryPoint();p=n.pose.pose.position
 point.position.x,point.position.y,point.position.z=p.x,p.y,p.z;point.yaw_angle=0.
 old.setpoints=[point];count=len(outputs);end=time.monotonic()+.8
 while time.monotonic()<end:ref.publish(old);spin(.02)
 n.wait(lambda:not mode_is(ControlMode.TRAJECTORY),1.)
 assert len(outputs)==count
 n.destroy_publisher(ref)
 results['expired_reference']={'rejected':True,'hover':True}
 activate();n.mode(ControlMode.HOVER)
 n.service(SetBool,'super_adapter/enable',SetBool.Request(data=False))
 spin(.3);count=len(outputs);spin(.6);assert len(outputs)==count
 results['cancel']={'old_reference_stopped':True}
 n.action(Land,'LandBehavior',Land.Goal(land_speed=.5),'land_success')
 n.wait(lambda:not n.platform.armed,15.)
 results['landed_and_disarmed']=True
 open('/evidence/fault-results.json','w').write(json.dumps(results,indent=2))
 phase('ALL_PASS')
finally:
 if n.platform is not None and n.platform.armed:
  phase('RECOVERY_LAND')
  n.mode(ControlMode.HOVER)
  n.action(Land,'LandBehavior',Land.Goal(land_speed=.5),'land_success')
  n.wait(lambda:not n.platform.armed,15.)
 n.destroy_node();rclpy.shutdown()
