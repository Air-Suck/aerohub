"""Synthetic input regression against the real native SUPER node; no flight."""
import json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy,yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from mars_quadrotor_msgs.msg import PolynomialTrajectory
from tf2_ros import TransformBroadcaster
rclpy.init();node=Node('super_start_replay');actual=[-.006137986201792955,.0007000574842095375,2.7873992919921875]
odom=node.create_publisher(Odometry,'/drone0/navigation/odom',qos_profile_sensor_data)
cloud=node.create_publisher(__import__('sensor_msgs.msg',fromlist=['PointCloud2']).PointCloud2,'/drone0/navigation/points',qos_profile_sensor_data)
goal=node.create_publisher(PoseStamped,'/goal_pose',1);tf=TransformBroadcaster(node);plans=[]
sub=node.create_subscription(PolynomialTrajectory,'/planning_cmd/poly_traj',lambda m:plans.append(m),qos_profile_sensor_data)
config=yaml.safe_load(Path('/opt/super_app/planner.yaml').read_text());config['rog_map']['fix_map_origin']=[0.,0.,3.];config['rog_map']['virtual_ground_height']=-2.;config['rog_map']['virtual_ceil_height']=11.
Path('/tmp/start-replay.yaml').write_text(yaml.safe_dump(config));stream=open('/tmp/start-replay-native.log','w')
proc=subprocess.Popen(['ros2','run','super_planner','fsm_node','--ros-args','-p','use_sim_time:=false','-p','config_name:=/tmp/start-replay.yaml'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
start=time.monotonic();lastcloud=lastgoal=0
try:
 while not plans and time.monotonic()-start<18:
  stamp=node.get_clock().now().to_msg();o=Odometry();o.header.stamp=stamp;o.header.frame_id='drone0/odom';o.child_frame_id='drone0/base_link';o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=actual;o.pose.pose.orientation.w=1.;odom.publish(o)
  trans=TransformStamped();trans.header=o.header;trans.child_frame_id=o.child_frame_id;trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z=actual;trans.transform.rotation.w=1.;tf.sendTransform(trans)
  now=time.monotonic()
  if now-lastcloud>.2:
   h=Header();h.stamp=stamp;h.frame_id='drone0/odom';cloud.publish(point_cloud2.create_cloud_xyz32(h,[[x*.5,y*.5,0.] for x in range(-25,26) for y in range(-25,26)]));lastcloud=now
  if now-start>2 and now-lastgoal>.5 and goal.get_subscription_count():
   g=PoseStamped();g.header=o.header;g.pose.position.x=actual[0]+3;g.pose.position.y=actual[1]+8;g.pose.position.z=actual[2]+1;g.pose.orientation.w=1.;goal.publish(g);lastgoal=now
  rclpy.spin_once(node,timeout_sec=.02)
 assert plans,'native planner produced no trajectory; inspect /tmp/start-replay-native.log'
 p=plans[0];axes=[p.coef_pos_x,p.coef_pos_y,p.coef_pos_z];first=[axis[p.order_pos] for axis in axes];error=math.dist(first,actual)
 print(json.dumps({'actual':actual,'native_first_position':first,'start_error_m':error}),flush=True)
 assert error<1e-6, 'native start was displaced from an already non-occupied odometry position'
finally:
 os.killpg(proc.pid,signal.SIGTERM)
 try:proc.wait(timeout=5)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
 stream.close();node.destroy_node();rclpy.shutdown()
