"""Finite, stamped local-state fixture for planning-only smoke tests."""
import argparse,time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped

parser=argparse.ArgumentParser();parser.add_argument('--vehicles',nargs='+',default=['drone0','drone1','drone2','drone3']);parser.add_argument('--height',type=float,default=2.)
args=parser.parse_args();rclpy.init();node=rclpy.create_node('cat_ora_pose_fixture');executor=SingleThreadedExecutor();executor.add_node(node)
publishers={v:node.create_publisher(PoseStamped,'/'+v+'/self_localization/pose',1) for v in args.vehicles}
def publish():
    for vehicle,publisher in publishers.items():
        m=PoseStamped();m.header.frame_id=vehicle+'/odom';m.header.stamp=node.get_clock().now().to_msg();m.pose.position.z=args.height;m.pose.orientation.w=1.;publisher.publish(m)
node.create_timer(.05,publish);until=time.monotonic()+12
try:
    while time.monotonic()<until:executor.spin_once(timeout_sec=.05)
finally:executor.shutdown();node.destroy_node();rclpy.shutdown()
