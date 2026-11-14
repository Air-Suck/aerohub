"""Contract checks against the actual C++ bridge using controlled ROS time."""
import copy
import math
import signal
import struct
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init();node=rclpy.create_node('verify_swarm_navigation_bridge')
    clocks=node.create_publisher(Clock,'/clock',10)
    odometry=node.create_publisher(Odometry,'/contract/odom',10)
    clouds=node.create_publisher(PointCloud2,'/contract/cloud',10)
    output_odom=[];output_cloud=[]
    subscriptions=[node.create_subscription(Odometry,'/contract/output/odom',output_odom.append,qos_profile_sensor_data),
                   node.create_subscription(PointCloud2,'/contract/output/points',output_cloud.append,qos_profile_sensor_data)]
    process=subprocess.Popen(['ros2','run','plan_env','navigation_bridge_node','--ros-args',
        '-r','__ns:=/contract/output','-p','use_sim_time:=true','-p','source_frame:=local',
        '-p','body_frame:=body','-p','map_frame:=map','-p','origin_enu:=[1.0, 2.0, 3.0]',
        '-p',f'origin_yaw:={math.pi/2}','-p','odom_topic:=/contract/odom','-p','cloud_topic:=/contract/cloud'])

    def spin(seconds=.25):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)

    def clock(at):
        msg=Clock();msg.clock.sec=int(at);msg.clock.nanosec=round((at-int(at))*1e9)
        for _ in range(3):clocks.publish(msg);spin(.03)

    def odom(at):
        msg=Odometry();msg.header.frame_id='local';msg.child_frame_id='body'
        msg.header.stamp.sec=int(at);msg.header.stamp.nanosec=round((at-int(at))*1e9)
        msg.pose.pose.position.x=1.;msg.pose.pose.orientation.w=math.sqrt(.5);msg.pose.pose.orientation.z=math.sqrt(.5)
        msg.pose.covariance[0]=1.;msg.pose.covariance[7]=2.;msg.pose.covariance[14]=3.
        msg.twist.twist.linear.x=1.;msg.twist.twist.linear.y=2.;msg.twist.twist.linear.z=3.
        return msg

    def cloud(at):
        msg=PointCloud2();msg.header=odom(at).header;msg.height=msg.width=1
        msg.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,name in enumerate(('x','y','z','intensity'))]
        msg.point_step=msg.row_step=16;msg.data=struct.pack('<ffff',2.,3.,4.,17.);return msg

    try:
        deadline=time.monotonic()+10
        while odometry.get_subscription_count()<1 or clouds.get_subscription_count()<1:
            assert time.monotonic()<deadline,'bridge discovery failed'
            spin(.05)
        clock(100.)
        clouds.publish(cloud(100.));spin();assert not output_cloud,'cloud forwarded without odometry'
        odometry.publish(odom(100.));spin();assert output_odom
        transformed=output_odom[-1];p=transformed.pose.pose.position
        assert math.dist((p.x,p.y,p.z),(1,3,3))<1e-8
        assert abs(transformed.pose.pose.orientation.z-1.)<1e-8
        assert abs(transformed.pose.covariance[0]-2.)<1e-8 and abs(transformed.pose.covariance[7]-1.)<1e-8
        assert transformed.twist==odom(100.).twist and transformed.child_frame_id=='body'
        clouds.publish(cloud(100.));spin();assert len(output_cloud)==1
        result=output_cloud[-1]
        assert result.header.frame_id=='map' and result.header.stamp==cloud(100.).header.stamp
        assert math.dist(struct.unpack('<ffff',result.data)[:3],(-2.,4.,7.))<1e-5
        assert struct.unpack('<ffff',result.data)[3]==17.,'extra field lost'
        before=len(output_cloud)
        for bad in (cloud(100.),cloud(99.),cloud(0.),cloud(101.)):
            clouds.publish(bad);spin(.08)
        bad=cloud(100.1);bad.header.frame_id='sensor';clouds.publish(bad);spin()
        assert len(output_cloud)==before,'invalid frame/time accepted'
        clock(102.)
        clouds.publish(cloud(102.));spin();assert len(output_cloud)==before,'stale odometry accepted'
        odometry.publish(odom(102.));spin();clouds.publish(cloud(102.));spin();assert len(output_cloud)==before+1
        clock(50.)
        clouds.publish(cloud(50.));spin();assert len(output_cloud)==before+1,'old odometry reused across clock reset'
        odometry.publish(odom(50.));spin();clouds.publish(cloud(50.));spin();assert len(output_cloud)==before+2
        print('PASS rigid transform, covariance, body twist, intensity, time/frame validation, stale state, clock reset and recovery')
    finally:
        process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
