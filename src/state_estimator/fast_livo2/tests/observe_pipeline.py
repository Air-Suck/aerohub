#!/usr/bin/env python3
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import PointCloud2

rclpy.init()
n=Node('fast_livo_pipeline_observer', namespace='drone0')
counts={'pose':0,'twist':0,'cloud':0}
last={}
def pose(m):
    assert m.header.frame_id=='earth'
    assert all(math.isfinite(x) for x in (m.pose.position.x,m.pose.position.y,m.pose.position.z,m.pose.orientation.w))
    counts['pose']+=1; last['pose_stamp']=[m.header.stamp.sec,m.header.stamp.nanosec]
def twist(m):
    assert m.header.frame_id=='drone0/base_link'
    assert all(math.isfinite(x) for x in (m.twist.linear.x,m.twist.linear.y,m.twist.linear.z,m.twist.angular.z))
    counts['twist']+=1
def cloud(m):
    assert m.header.frame_id=='drone0/livo_world'
    assert {'x','y','z'}.issubset({f.name for f in m.fields})
    assert len(m.data)==m.row_step*m.height and m.width>0
    counts['cloud']+=1; last['cloud_fields']=[f.name for f in m.fields]
subs=[n.create_subscription(PoseStamped,'self_localization/pose',pose,qos_profile_sensor_data),
      n.create_subscription(TwistStamped,'self_localization/twist',twist,qos_profile_sensor_data),
      n.create_subscription(PointCloud2,'perception/fast_livo2/registered_points',cloud,qos_profile_sensor_data)]
deadline=time.monotonic()+40
while time.monotonic()<deadline:
    rclpy.spin_once(n,timeout_sec=0.1)
result=dict(counts=counts,details=last,scope='synthetic three-sensor execution smoke, not accuracy or flight validation')
print(json.dumps(result,indent=2))
assert counts['pose']>20 and counts['twist']>20 and counts['cloud']>3, result
print('PASS: actual FAST-LIVO2 -> ROS1 bridge -> ROS2 -> native AS2 plugin and separate point-cloud output')
n.destroy_node(); rclpy.shutdown()
