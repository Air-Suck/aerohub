"""Protocol regression: live proxy gates cached trajectories on map heartbeats.

This isolated test supplies test messages, unlike verify_airsim_cloud.py which
uses the real AirSim sensor and native map.
"""
import json
import socket
import subprocess
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from traj_utils.msg import PolyTraj


def main():
    rclpy.init();node=rclpy.create_node('verify_swarm_proxy_health')
    health=[node.create_publisher(Bool,f'/drone_{i}_ego_planner_node/grid_map/input_healthy',10) for i in range(7)]
    trajectory=[node.create_publisher(PolyTraj,f'/drone_{i}_planning/trajectory',10) for i in range(7)]
    goals=[]
    subscriptions=[node.create_subscription(PoseStamped,'/traj_start_trigger',goals.append,10) for _ in range(7)]
    process=subprocess.Popen(['ros2','run','ego_planner','ros2_proxy.py','--ros-args','-p','publish_odometry:=false'])
    payload={'task_id':'health-contract','vehicles':[{'id':f'drone{i}','position_enu':[float(i),0.,1.]} for i in range(7)],
             'goal_center_enu':[0.,0.,1.],'lead':.1,'sample_dt':.1,'horizon':.2}

    def spin(duration=.1):
        until=time.monotonic()+duration
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)

    def request():
        with socket.create_connection(('127.0.0.1',28501),timeout=2) as sock:
            sock.sendall((json.dumps(payload)+'\n').encode())
            return json.loads(sock.makefile('rb').readline())

    def refresh(skip=None):
        for i,publisher in enumerate(health):
            if i!=skip:publisher.publish(Bool(data=True))
        spin()

    def polys():
        for i,publisher in enumerate(trajectory):
            msg=PolyTraj();msg.drone_id=i;msg.order=5;msg.traj_id=1;msg.start_time=node.get_clock().now().to_msg()
            msg.duration=[5.];msg.coef_x=[0.,0.,0.,0.,0.,float(i)];msg.coef_y=[0.]*6;msg.coef_z=[0.,0.,0.,0.,0.,1.]
            publisher.publish(msg)
        spin()

    try:
        deadline=time.monotonic()+15
        while True:
            try:
                reply=request();break
            except OSError:
                assert time.monotonic()<deadline,'proxy startup timeout'
                spin()
        assert not reply['ok'],'live proxy accepted without fresh real inputs'
        while not goals:
            assert time.monotonic()<deadline,'goal discovery timeout'
            refresh();request();spin()
        refresh();polys();assert request()['ok']
        health[6].publish(Bool(data=False));spin()
        assert not request()['ok'],'unhealthy map did not gate cached trajectories'
        refresh();polys();assert request()['ok']
        until=time.monotonic()+1.3
        while time.monotonic()<until:refresh(skip=6)
        assert not request()['ok'],'expired map heartbeat did not gate cached trajectories'
        refresh();polys();assert request()['ok']
        assert not node.get_publishers_info_by_topic('/drone_0_visual_slam/odom'),'live proxy still publishes zero-velocity odometry'
        print('PASS missing/unhealthy/expired map gating, fresh-input recovery and no synthetic live odometry')
    finally:
        process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
