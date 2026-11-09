#!/usr/bin/env python3
"""Synthetic stationary IMU, room scan, and texture; not an accuracy dataset."""
import math
import time
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, Image, PointField
from sensor_msgs import point_cloud2

rospy.init_node('fast_livo_synthetic_sensors')
imu_pub = rospy.Publisher('/test/imu', Imu, queue_size=100)
cloud_pub = rospy.Publisher('/test/points', point_cloud2.PointCloud2, queue_size=2)
image_pub = rospy.Publisher('/test/image', Image, queue_size=2)
points = []
for az in range(180):
    theta = az * 2 * math.pi / 180
    for ring in range(16):
        elev = (-30 + ring * 4) * math.pi / 180
        ray = [math.cos(theta)*math.cos(elev), math.sin(theta)*math.cos(elev), math.sin(elev)]
        dist = min(limit/abs(d) for limit,d in zip((5.,4.,2.),ray) if abs(d)>1e-8)
        points.append([dist*d for d in ray] + [80., (az*16+ring)*0.099/(180*16), ring])
fields = [PointField(n, i*4, PointField.FLOAT32, 1) for i,n in enumerate(('x','y','z','intensity','time'))]
fields.append(PointField('ring',20,PointField.UINT16,1))
texture = np.random.default_rng(42).integers(0,256,(480,640,3),dtype=np.uint8)
rospy.sleep(2)
rate = rospy.Rate(200)
start = time.monotonic()
for index in range(6000):
    if rospy.is_shutdown(): break
    now = rospy.Time.now()
    imu = Imu(); imu.header.stamp=now; imu.header.frame_id='imu'
    imu.linear_acceleration.z = 9.81
    imu.orientation_covariance[0] = -1.
    imu_pub.publish(imu)
    if index % 20 == 0:
        # Scan spans the preceding 99 ms; IMU coverage must include its end.
        cloud_pub.publish(point_cloud2.create_cloud(Header(stamp=now-rospy.Duration(0.1),frame_id='lidar'),fields,points))
        image=Image(); image.header.stamp=now-rospy.Duration(0.05); image.header.frame_id='camera'
        image.height=480; image.width=640; image.encoding='bgr8'; image.step=640*3
        image.data=texture.tobytes(); image_pub.publish(image)
    rate.sleep()
print('Published synthetic IMU, lidar and images for', time.monotonic()-start, 'seconds')
