"""Board-local smoke checks for persistent libraries. No vehicle commands or simulation."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time

ROOT=Path('/data/aerohub')


class Check:
    def __init__(self, component):
        self.component=component
        self.root=ROOT/'as2' if component=='as2' else ROOT/'lib'/component
        self.output=self.root/'runtime/verification'
        self.output.mkdir(parents=True,exist_ok=True)
        (self.output/'result.json').unlink(missing_ok=True)
        self.children=[]
        self.result={'component':component,'root':str(self.root),'machine':os.uname().machine,
                     'scope':'board-local startup and minimal interface; no flight or real sensor acceptance'}
    def start(self,name,command):
        stream=(self.output/(name+'.log')).open('w')
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        stream.close()
        self.children.append(process)
        return process
    def alive(self):
        for process in self.children:
            if process.poll() is not None: raise RuntimeError('process exited: '+str(process.returncode))
    def stop(self):
        for process in self.children:
            try: os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError: pass
        for process in self.children:
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired: pass
            try: os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError: pass
            process.wait(timeout=3)
        self.result['test_processes_stopped']=all(p.poll() is not None for p in self.children)
    def wait(self,node,predicate,timeout=25):
        import rclpy
        deadline=time.monotonic()+timeout
        while not predicate():
            self.alive()
            if time.monotonic()>deadline: raise TimeoutError('waiting for minimal interface')
            rclpy.spin_once(node,timeout_sec=.05)
    def as2(self):
        import as2_core
        from as2_msgs.msg import PlatformInfo
        from rclpy.serialization import serialize_message,deserialize_message
        encoded=serialize_message(PlatformInfo())
        deserialize_message(encoded,PlatformInfo)
        binary=self.root/'install/as2_state_estimator/lib/as2_state_estimator/as2_state_estimator_node'
        self.start('state-estimator',[str(binary),'--ros-args',
            '--params-file',str(self.root/'install/as2_state_estimator/share/as2_state_estimator/config/state_estimator_default.yaml'),
            '--params-file',str(self.root/'install/as2_state_estimator/share/as2_state_estimator/plugins/raw_odometry/config/plugin_default.yaml'),
            '-r','__ns:=/aerohub_verify',
            '-p','plugin_name:=raw_odometry','-p','use_sim_time:=false'])
        time.sleep(5)
        self.alive()
        self.result.update(as2_core_import=True,platform_info_cdr_bytes=len(encoded),state_estimator_alive_seconds=5)
    def catora(self):
        import rclpy
        import uav_cat_ora_adapter.node
        import uav_planner_framework.ros_node
        self.result['application_modules_import']=True
        from geometry_msgs.msg import Point
        from mrs_formation_reshaping.srv import SrvGetReshapingTrajectories as Service
        directory=self.root/'install/mrs_formation_reshaping/lib/mrs_formation_reshaping'
        self.start('native',[str(directory/'cat_ora_node')])
        self.start('proxy',[str(directory/'ros2_proxy.py')])
        node=rclpy.create_node('catora_deployment_check')
        try:
            client=node.create_client(Service,'/mrs_formation_reshaping/get_reshaping_trajectories')
            self.wait(node,client.service_is_ready)
            request=Service.Request(initial_configurations=[Point(x=0.,y=0.,z=1.)],
                goal_configurations=[Point(x=1.,y=0.,z=1.)],trajectory_dt=.2,max_velocity=.8,max_acceleration=.5)
            future=client.call_async(request)
            self.wait(node,future.done)
            response=future.result()
            assert response.success,response.message
            assert len(response.trajectories)==1
            points=response.trajectories[0].waypoints
            assert len(points)>=2 and all(math.isfinite(v) for p in points for v in (p.x,p.y,p.z))
            assert math.dist([points[-1].x,points[-1].y,points[-1].z],[1,0,1])<1e-6
            payload={'starts_enu':[[0,0,1]],'goals_enu':[[1,0,1]],'trajectory_dt':.2,'max_velocity':.8,'max_acceleration':.5}
            deadline=time.monotonic()+20
            while True:
                try:
                    with socket.create_connection(('127.0.0.1',28500),timeout=2) as connection:
                        connection.sendall((json.dumps(payload)+'\n').encode())
                        connection.settimeout(20)
                        reply=json.loads(connection.makefile('rb').readline())
                    break
                except ConnectionRefusedError:
                    if time.monotonic()>deadline: raise
                    self.alive()
                    time.sleep(.1)
            assert reply['ok'],reply
            actual=reply['trajectories_enu'][0]
            assert len(actual)==len(points)
            assert all(math.dist(p,[q.x,q.y,q.z])<1e-9 for p,q in zip(actual,points))
            self.result.update(native_service=True,json_proxy=True,trajectory_count=1,samples=len(points),endpoint_error_m=math.dist(actual[-1],[1,0,1]))
        finally: node.destroy_node()
    def swarm(self):
        import rclpy
        import yaml
        import uav_swarm_formation_adapter.node
        import uav_planner_framework.ros_node
        self.result['application_modules_import']=True
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2,PointField
        from geometry_msgs.msg import PoseStamped
        from traj_utils.msg import PolyTraj
        from rclpy.qos import qos_profile_sensor_data
        prefix='/aerohub_swarm_check'
        config=yaml.safe_load((self.root/'install/ego_planner/share/ego_planner/config/humble.yaml').read_text())
        config.update({'use_sim_time':False,'fsm.flight_type':1,'manager.drone_id':0,
            'grid_map.map_size_x':12.,'grid_map.map_size_y':12.,'grid_map.map_size_z':4.,
            'grid_map.resolution':.2,'grid_map.input_timeout':1.,'optimization.record_opt':False,
            'fsm.result_file':str(self.output/'planner-results.txt')})
        filename=self.output/'smoke.yaml'
        filename.write_text(yaml.safe_dump({'/**':{'ros__parameters':config}}))
        node=rclpy.create_node('swarm_deployment_check')
        received=[]
        odom_pub=node.create_publisher(Odometry,prefix+'/odom',qos_profile_sensor_data)
        cloud_pub=node.create_publisher(PointCloud2,prefix+'/cloud',qos_profile_sensor_data)
        goal_pub=node.create_publisher(PoseStamped,prefix+'/goal',10)
        subscription=node.create_subscription(PolyTraj,prefix+'/trajectory',received.append,10)
        directory=self.root/'install/ego_planner/lib/ego_planner'
        self.start('native',[str(directory/'ego_planner_node'),'--ros-args','--params-file',str(filename),
            '-r','__node:=swarm_smoke','-r','~/odom_world:='+prefix+'/odom','-r','~/grid_map/odom:='+prefix+'/odom',
            '-r','~/grid_map/cloud:='+prefix+'/cloud','-r','/goal:='+prefix+'/goal',
            '-r','~/planning/trajectory:='+prefix+'/trajectory'])
        try:
            start=time.monotonic()
            while not received and time.monotonic()-start<30:
                self.alive()
                stamp=node.get_clock().now().to_msg()
                odom=Odometry()
                odom.header.stamp=stamp; odom.header.frame_id='world'; odom.child_frame_id='base_link'
                odom.pose.pose.position.x=-2.; odom.pose.pose.position.z=1.; odom.pose.pose.orientation.w=1.
                odom_pub.publish(odom)
                cloud=PointCloud2()
                cloud.header.stamp=stamp; cloud.header.frame_id='world'; cloud.height=cloud.width=1
                cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
                cloud.point_step=cloud.row_step=12; cloud.is_dense=True; cloud.data=struct.pack('fff',4.,4.,1.)
                cloud_pub.publish(cloud)
                if time.monotonic()-start>2 and goal_pub.get_subscription_count():
                    goal=PoseStamped();goal.header.stamp=stamp;goal.header.frame_id='world'
                    goal.pose.position.x=2.;goal.pose.position.z=1.;goal.pose.orientation.w=1.
                    goal_pub.publish(goal)
                rclpy.spin_once(node,timeout_sec=.05)
                time.sleep(.02)
            assert received,'native planner did not publish PolyTraj'
            message=received[0]
            assert message.order==5 and message.drone_id==0
            assert message.duration and all(math.isfinite(t) and t>0 for t in message.duration)
            for values in (message.coef_x,message.coef_y,message.coef_z):
                assert len(values)==6*len(message.duration) and all(math.isfinite(x) for x in values)
            self.result.update(native_planner=True,input='static odometry + one-point synthetic cloud + manual goal',
                               trajectory_order=message.order,polynomial_pieces=len(message.duration),
                               coefficients_per_axis=len(message.coef_x),real_sensor_validation=False)
        finally: node.destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('component',choices=['as2','catora','swarm-formation'])
    args=parser.parse_args()
    check=Check(args.component)
    start=time.monotonic()
    def interrupted(*_): raise InterruptedError('verification interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    import rclpy
    rclpy.init()
    try:
        getattr(check,{'as2':'as2','catora':'catora','swarm-formation':'swarm'}[args.component])()
        check.result['passed']=True
    except Exception as error:
        check.result.update(passed=False,error=repr(error))
    finally:
        check.stop()
        rclpy.shutdown()
        check.result['wall_seconds']=time.monotonic()-start
        check.result['ros_domain_id']=os.environ.get('ROS_DOMAIN_ID')
        (check.output/'result.json').write_text(json.dumps(check.result,indent=2)+'\n')
        print(json.dumps(check.result,indent=2),flush=True)
    return 0 if check.result['passed'] else 1

if __name__=='__main__': sys.exit(main())
