"""Read-only post-recovery proof. Does not send flight commands."""
import json
import os
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).with_name('vendor')))
os.environ['MSGPACK_PUREPYTHON']='1'
from airsim_rpc import Rpc
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.msg import BehaviorStatus
from px4_msgs.msg import VehicleControlMode

rclpy.init();node=Node('dual_cleanup_observer')
latest={};counts={};keep=[]
def receive(topic,msg):
    latest[topic]=dict(status=msg.status) if isinstance(msg,BehaviorStatus) else dict(armed=msg.flag_armed,offboard=msg.flag_control_offboard_enabled)
    counts[topic]=counts.get(topic,0)+1
for i in (0,1):
    topic=f'/drone{i}/fmu/out/vehicle_control_mode'
    keep.append(node.create_subscription(VehicleControlMode,topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data))
deadline=time.monotonic()+3
while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
topics=[name for name,types in node.get_topic_names_and_types() if 'as2_msgs/msg/BehaviorStatus' in types]
for topic in topics:keep.append(node.create_subscription(BehaviorStatus,topic,lambda m,t=topic:receive(t,m),10))
deadline=time.monotonic()+4
while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
rpc=Rpc('host.docker.internal',41451,.8)
report=dict(latest=latest,counts=counts,behavior_idle_value=BehaviorStatus.IDLE,scene={})
for i in (0,1):
    report['scene'][f'drone{i}']={method:rpc.call(method,f'Drone{i}') for method in
        ('simGetVehiclePose','simGetGroundTruthKinematics','simGetCollisionInfo')}
report['both_disarmed']=all(not latest.get(f'/drone{i}/fmu/out/vehicle_control_mode',{}).get('armed',True) for i in (0,1))
report['all_observed_behaviors_idle']=bool(topics) and all(latest.get(t,{}).get('status')==BehaviorStatus.IDLE for t in topics)
Path('/evidence/post-recovery-ros-state.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['both_disarmed'] and report['all_observed_behaviors_idle'] else 1)
