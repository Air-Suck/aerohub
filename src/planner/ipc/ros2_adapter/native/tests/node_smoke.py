from ament_index_python.packages import get_package_prefix
"""No-flight actual-node session/input contract check; synthetic input only."""
import math
import sys
import subprocess
import time
import signal
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from uav_follow_reference_acro.msg import ControlSession,ControlFrame
from as2_msgs.msg import YawMode
from std_msgs.msg import String
rclpy.init();n=Node('ipc_contract_probe');frames=[]
sub=n.create_subscription(ControlFrame,'/drone0/external_acro/control',frames.append,qos_profile_sensor_data)
session=n.create_publisher(ControlSession,'/drone0/external_acro/session',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
odom=n.create_publisher(Odometry,'/drone0/navigation/odom',qos_profile_sensor_data)
cloud=n.create_publisher(PointCloud2,'/drone0/navigation/points',qos_profile_sensor_data)
configs=[];reference_strides=[]
def record_health(m):
 if m.data.startswith("solved ") and "reference_stride=" in m.data:
  values=dict(v.split("=")for v in m.data.removeprefix("solved ").split(","));reference_strides.append(int(values["reference_stride"]))
n.create_subscription(String,"/drone0/ipc/health",record_health,100)
n.create_subscription(String,'/drone0/ipc/health',lambda m:configs.append(m.data) if m.data.startswith('session_config ') else None,10)
scenario=0
child=subprocess.Popen([get_package_prefix('ipc_native_core')+'/lib/ipc_native_core/ipc_node','--ros-args','-r','__ns:=/drone0']+(['-p','planar_path:=true','-p','reference_speed_mps:=0.25'] if '--planar' in sys.argv else [])+(['-p','mpc.velocity_limit:=4.0','-p','reference_speed_mps:=3.0','-p','reference_speed_ratio:=1.0','-p','map.z_max:=12.0'] if '--fast' in sys.argv else [])+(['-p','reference_acceleration_mps2:=1.0'] if '--ramp' in sys.argv else []))
def publish_inputs():
 stamp=n.get_clock().now().to_msg()
 o=Odometry();o.header.stamp=stamp;o.header.frame_id='earth';o.child_frame_id='drone0/base_link';o.pose.pose.position.z=2.;o.pose.pose.orientation.w=1.;
 if scenario==1:o.pose.pose.orientation.w=math.sqrt(.5);o.pose.pose.orientation.z=math.sqrt(.5);o.twist.twist.linear.x=.1
 odom.publish(o)
 c=PointCloud2();c.header=o.header;c.height=1;c.width=0;c.point_step=12;c.row_step=0;c.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)for i,k in enumerate(('x','y','z'))];cloud.publish(c)
def wait(test,seconds,inputs=True):
 end=time.monotonic()+seconds
 while not test():
  if child.poll() is not None:raise RuntimeError('node exited')
  if time.monotonic()>end:raise TimeoutError('contract wait')
  if inputs:publish_inputs()
  rclpy.spin_once(n,timeout_sec=.02)
try:
 wait(lambda:odom.get_subscription_count()>0 and session.get_subscription_count()>0,10)
 cli=n.create_client(SetParameters,'/drone0/ipc_native/set_parameters');wait(cli.service_is_ready,5)
 request=SetParameters.Request(parameters=[Parameter('mpc.jerk_limit',value=1.0).to_parameter_msg()]);future=cli.call_async(request);wait(future.done,5)
 assert not future.result().results[0].successful, 'startup profile must be read-only'
 s=ControlSession();s.header.frame_id='earth';s.session_id='probe';s.revision=1;s.active=True;s.target.header.frame_id='earth';s.target.point.z=2.;s.yaw.mode=YawMode.FIXED_YAW;s.yaw.angle=.15;s.max_speed.x=s.max_speed.y=s.max_speed.z=(4. if '--fast' in sys.argv else .5);session.publish(s)
 wait(lambda:any(f.solver_ok for f in frames),10)
 if '--fast' in sys.argv:
  wait(lambda:bool(configs),5)
  assert 'velocity_limit=4.000000' in configs[-1] and 'reference_stride=6' in configs[-1] and 'planar=0' in configs[-1],configs[-1]
 f=next(f for f in frames if f.solver_ok)
 assert f.session_id=='probe' and f.revision==1 and f.body_rates.z>0 and math.isfinite(f.thrust_newtons)
 assert abs(f.thrust_newtons-.48821*15)<.1
 if '--ramp' in sys.argv:
  wait(lambda:any(v>=3 for v in reference_strides),5)
  assert reference_strides[0]==1 and max(reference_strides)<=6,reference_strides
 previous=len(frames);wait(lambda:len(frames)>previous and not frames[-1].solver_ok,3,inputs=False)
 scenario=1;s.revision=2;s.yaw.angle=math.pi/2;session.publish(s)
 wait(lambda:any(f.solver_ok and f.revision==2 for f in frames),10)
 turned=next(f for f in frames if f.solver_ok and f.revision==2)
 assert turned.body_rates.y<0 and abs(turned.body_rates.x)<1e-3, 'nonidentity body velocity rotation'
 s.active=False;session.publish(s)
 for _ in range(10):rclpy.spin_once(n,timeout_sec=.02)
 before=len(frames)
 for _ in range(10):rclpy.spin_once(n,timeout_sec=.02)
 assert len(frames)==before
 print('actual node: session, fixed yaw, thrust, stale-input rejection, inactive silence PASS')
finally:
 child.send_signal(signal.SIGINT);child.wait(timeout=5);n.destroy_node();rclpy.shutdown()
