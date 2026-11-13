"""Read-only real AirSim/PX4 ground checks. Never arms or sends motion."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleOdometry,VehicleControlMode

rclpy.init();node=Node('ego_ground_probe');rows={};subscriptions=[]
status_topic='fmu/out/vehicle_control_mode'
def receive(key,msg):
    row=rows.setdefault(key,{'count':0,'stamps':set(),'last_received':0})
    row['count']+=1;row['last_received']=time.monotonic()
    if isinstance(msg,Clock):stamp=msg.clock.sec*10**9+msg.clock.nanosec
    elif isinstance(msg,PointCloud2):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        row['frame']=msg.header.frame_id;row['points']=msg.width*msg.height
    else:
        stamp=msg.timestamp
        if isinstance(msg,VehicleControlMode):
            row['armed']=msg.flag_armed
            row['offboard']=msg.flag_control_offboard_enabled
            row['ever_armed']=row.get('ever_armed',False) or msg.flag_armed
            row['ever_offboard']=row.get('ever_offboard',False) or msg.flag_control_offboard_enabled
    row['stamps'].add(stamp)
for topic,cls in [('/clock',Clock)]+[(f'/drone{i}/{tail}',cls) for i in range(2)
    for tail,cls in [('fmu/out/vehicle_odometry',VehicleOdometry),(status_topic,VehicleControlMode),('sensor_measurements/Lidar/points',PointCloud2)]]:
    subscriptions.append(node.create_subscription(cls,topic,lambda msg,key=topic:receive(key,msg),qos_profile_sensor_data))
deadline=time.monotonic()+30
while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
now=time.monotonic();failures=[]
for topic in ['/clock']+[f'/drone{i}/{tail}' for i in range(2) for tail in ['fmu/out/vehicle_odometry',status_topic,'sensor_measurements/Lidar/points']]:
    row=rows.get(topic)
    if row is None:failures.append('missing '+topic);continue
    row['unique_stamps']=len(row.pop('stamps'));row['last_age']=now-row.pop('last_received')
    if row['unique_stamps']<20 or row['last_age']>1:failures.append('stale/low-rate '+topic)
    if 'points' in row and row['points']==0:failures.append('empty cloud '+topic)
    if row.get('ever_armed'):failures.append('armed during ground check '+topic)
    if row.get('ever_offboard'):failures.append('offboard during ground check '+topic)
result={'passed':not failures,'duration_seconds':30,'gates':{'minimum_unique_stamps':20,'maximum_last_age_seconds':1,'both_disarmed':True,'both_not_offboard':True,'nonempty_cloud':True},'topics':rows,'failures':failures,'status_source':'VehicleControlMode: same armed/offboard fields consumed by AS2 Pixhawk; does not cover all VehicleStatus fields','scope':'real AirSim/PX4 ground data only; no arming or flight'}
Path('/evidence/result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
node.destroy_node();rclpy.shutdown();raise SystemExit(bool(failures))
