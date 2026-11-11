"""Disarmed input discovery; never arms or publishes motion references; probes controller mode while disarmed."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from uav_follow_reference_acro.msg import ControlSession,ControlFrame
from as2_msgs.msg import YawMode
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from as2_msgs.msg import PlatformInfo,ControllerInfo,ControlMode
from as2_msgs.srv import SetControlMode
rclpy.init();n=Node('ipc_ground_input_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
counts={};samples={};platform=[];pose=[];ready=[];frames=[];frame_wall=[]
session_pub=n.create_publisher(ControlSession,'/drone0/external_acro/session',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(PoseStamped,'/drone0/self_localization/pose',pose.append,qos_profile_sensor_data)
n.create_subscription(Bool,'/drone0/ipc/ready',ready.append,1)
n.create_subscription(ControlFrame,'/drone0/external_acro/control',lambda m:(frames.append(m),frame_wall.append(time.monotonic())),qos_profile_sensor_data)
def record(k,m):
 counts[k]=counts.get(k,0)+1
 if hasattr(m,'header'):samples[k]={'frame':m.header.frame_id,'stamp':[m.header.stamp.sec,m.header.stamp.nanosec]}
 if hasattr(m,'child_frame_id'):samples[k]['child']=m.child_frame_id
 if hasattr(m,'transforms'):samples[k]=[[t.header.frame_id,t.child_frame_id]for t in m.transforms]
for topic,kind in [('sensor_measurements/odom',Odometry),('self_localization/pose',PoseStamped),('sensor_measurements/Lidar/points',PointCloud2),('navigation/odom',Odometry),('navigation/points',PointCloud2)]:
 n.create_subscription(kind,'/drone0/'+topic,lambda m,k=topic:record(k,m),qos_profile_sensor_data)
n.create_subscription(TFMessage,'/tf',lambda m:record('tf',m),qos_profile_sensor_data)
n.create_subscription(TFMessage,'/tf_static',lambda m:record('tf_static',m),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(PlatformInfo,'/drone0/platform/info',platform.append,qos_profile_sensor_data)
infos=[]
n.create_subscription(ControllerInfo,'/drone0/controller/info',lambda m:infos.append({'wall':time.monotonic(),'sim':n.get_clock().now().nanoseconds/1e9,'mode':m.input_control_mode.control_mode,'yaw':m.input_control_mode.yaw_mode}),1)
mode_cli=n.create_client(SetControlMode,'/drone0/controller/set_control_mode');mode_future=None
end=time.monotonic()+20
sent=False
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.02)
 if not sent and pose and ready and ready[-1].data:
  m=ControlSession();m.session_id='disarmed-shadow';m.revision=1;m.active=True;m.target.header=pose[-1].header;m.target.point.x=pose[-1].pose.position.x;m.target.point.y=pose[-1].pose.position.y;m.target.point.z=pose[-1].pose.position.z;m.yaw.mode=YawMode.FIXED_YAW;m.max_speed.x=m.max_speed.y=m.max_speed.z=.5
  session_pub.publish(m);sent=True
  if platform and not platform[-1].armed and mode_cli.service_is_ready():
   request=SetControlMode.Request();request.control_mode.control_mode=ControlMode.BODY_RATES;request.control_mode.yaw_mode=ControlMode.YAW_SPEED;mode_future=mode_cli.call_async(request)
session_pub.publish(ControlSession(session_id='disarmed-shadow',revision=2,active=False))
Path('/evidence/shadow-timing.json').write_text(json.dumps([{'wall':w,'stamp_ns':f.header.stamp.sec*1000000000+f.header.stamp.nanosec,'sequence':f.sequence,'solver_ok':f.solver_ok}for w,f in zip(frame_wall,frames)],indent=2)+'\n')
Path('/evidence/controller-timing.json').write_text(json.dumps(infos,indent=2)+'\n')
samples['mode_probe_success']=bool(mode_future and mode_future.done() and mode_future.result().success)
samples['shadow_frames']={'total':len(frames),'valid':sum(f.solver_ok for f in frames),'unique_solve_stamps':len({f.last_successful_solve.sec*1000000000+f.last_successful_solve.nanosec for f in frames if f.solver_ok}),'max_wall_gap_s':max((b-a for a,b in zip(frame_wall,frame_wall[1:])),default=0)}

Path('/evidence/ground-inputs.json').write_text(json.dumps({'counts':counts,'samples':samples},indent=2)+'\n')
safe=bool(platform and not platform[-1].armed)
Path('/evidence/result.json').write_text(json.dumps({'success':False,'scope':'disarmed input discovery only','landed_and_disarmed':safe})+'\n')
n.destroy_node();rclpy.shutdown()
