#!/usr/bin/env python3
"""Real native planners -> real AS2 FollowPath; synthetic controller/inputs.

This exercises middleware, Action updates/results and actual downstream PVA.
It does not start AirSim/PX4 and must not be reported as a flight test.
"""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from as2_msgs.msg import ControlMode, ControllerInfo, PlatformInfo, PlatformStatus, TrajectorySetpoints
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped,TwistStamped,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header,String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from quadrotor_msgs.msg import GoalSet
from traj_utils.msg import PolyTraj
from uav_ego_planner_adapter.conversion import nanoseconds,set_ns

project=Path(__file__).resolve().parents[5]

class Harness(Node):
    def __init__(self, scoped=False):
        super().__init__('ego_as2_contract_test')
        self.vehicles=['drone0','drone1'];self.pubs={};self.state={};self.refs={v:[] for v in self.vehicles}
        self.polys={v:[] for v in self.vehicles};self.observed={v:0 for v in self.vehicles}
        self.status={};self.skip_cloud=False;self.input_count=0;self.pva_max=[0.,0.]
        self.stamps={};self.odom_errors=[];self.tf=TransformBroadcaster(self)
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub=self.create_publisher(GoalSet,'/ego/goal',qos) if not scoped else None
        self.cancel=self.create_client(Trigger,'/ego/cancel') if not scoped else None
        self.bad_pub=self.create_publisher(PolyTraj,'/drone0/ego_planner/planning/trajectory',10)
        if not scoped:self.create_subscription(String,'/ego/status',lambda m:setattr(self,'status',json.loads(m.data)),10)
        for index,v in enumerate(self.vehicles):
            self.state[v]={'p':[0.,0.,2.],'v':[0.,0.,0.],'mode':ControlMode.POSITION,'yaw':index*math.pi/2,
                           'control':ControlMode(control_mode=ControlMode.POSITION,yaw_mode=ControlMode.YAW_ANGLE)}
            self.pubs[v]={key:self.create_publisher(cls,f'/{v}/{topic}',10 if key in ['platform','controller'] else qos_profile_sensor_data)
                          for key,cls,topic in [('odom',Odometry,'navigation/odom'),('cloud',PointCloud2,'navigation/points'),
                            ('pose',PoseStamped,'self_localization/pose'),('twist',TwistStamped,'self_localization/twist'),
                            ('platform',PlatformInfo,'platform/info'),('controller',ControllerInfo,'controller/info')]}
            self.create_service(SetControlMode,f'/{v}/controller/set_control_mode',lambda req,res,vehicle=v:self.set_mode(vehicle,req,res))
            self.create_subscription(TrajectorySetpoints,f'/{v}/motion_reference/trajectory',lambda m,vehicle=v:self.reference(vehicle,m),qos_profile_sensor_data)
            self.create_subscription(PoseStamped,f'/{v}/motion_reference/pose',lambda m,vehicle=v:self.position(vehicle,m),qos_profile_sensor_data)
            self.create_subscription(PolyTraj,f'/{v}/ego_planner/planning/trajectory',lambda m,vehicle=v:self.polys[vehicle].append(m),10)
            native_odom=f'/{v}/ego/native/odom' if scoped else f'/ego/{v}/odom'
            self.create_subscription(Odometry,native_odom,lambda m,vehicle=v:self.native_odom(vehicle,m),qos_profile_sensor_data)
        self.create_timer(.02,self.inputs)

    def set_mode(self,vehicle,request,response):
        self.state[vehicle]['mode']=request.control_mode.control_mode
        self.state[vehicle]['control']=copy.deepcopy(request.control_mode)
        if request.control_mode.control_mode == ControlMode.HOVER:self.state[vehicle]['v']=[0.,0.,0.]
        response.success=True;return response

    def position(self,vehicle,msg):
        if self.state[vehicle]['mode'] != ControlMode.TRAJECTORY:
            self.state[vehicle]['p']=[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
            self.state[vehicle]['v']=[0.,0.,0.]

    def reference(self,vehicle,msg):
        assert msg.header.frame_id==vehicle+'/odom','wrong execution frame'
        p=msg.setpoints[0]
        self.state[vehicle]['p']=[p.position.x,p.position.y,p.position.z]
        self.state[vehicle]['v']=[p.twist.x,p.twist.y,p.twist.z]
        self.refs[vehicle].append({'stamp':nanoseconds(msg.header.stamp),'p':self.state[vehicle]['p'],
                                   'v':self.state[vehicle]['v'],'a':[p.acceleration.x,p.acceleration.y,p.acceleration.z]})
        self.pva_max[0]=max(self.pva_max[0],math.dist([0,0,0],self.state[vehicle]['v']))
        self.pva_max[1]=max(self.pva_max[1],math.dist([0,0,0],[p.acceleration.x,p.acceleration.y,p.acceleration.z]))

    def native_odom(self,vehicle,msg):
        expected=self.stamps.get((vehicle,nanoseconds(msg.header.stamp)))
        if expected is None:return
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        self.odom_errors.append(max(math.dist([p.x,p.y,p.z],expected[0]),math.dist([v.x,v.y,v.z],expected[1])))
        if math.dist([0,0,0],expected[1])>.01:self.observed[vehicle]+=1

    def inputs(self):
        self.input_count+=1
        for index,v in enumerate(self.vehicles):
            s=self.state[v];stamp=self.get_clock().now().to_msg()
            pose=PoseStamped();pose.header.frame_id=v+'/odom';pose.header.stamp=stamp
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=s['p']
            pose.pose.orientation.z=math.sin(s['yaw']/2);pose.pose.orientation.w=math.cos(s['yaw']/2)
            self.pubs[v]['pose'].publish(pose)
            odom=Odometry();odom.header=copy.deepcopy(pose.header);odom.child_frame_id=v+'/base_link';odom.pose.pose=pose.pose
            vx,vy,vz=s['v'];c=math.cos(s['yaw']);sn=math.sin(s['yaw'])
            odom.twist.twist.linear.x=c*vx+sn*vy;odom.twist.twist.linear.y=-sn*vx+c*vy;odom.twist.twist.linear.z=vz
            self.pubs[v]['odom'].publish(odom)
            self.stamps[(v,nanoseconds(stamp))]=([s['p'][0],s['p'][1]+index*4,s['p'][2]],
                [odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z])
            if self.input_count%5==0 and not (self.skip_cloud and index==1):
                cloud=point_cloud2.create_cloud_xyz32(pose.header,self.cloud_points(v,index,s))
                self.pubs[v]['cloud'].publish(cloud)
            twist=TwistStamped();twist.header.stamp=stamp;twist.header.frame_id='earth'
            twist.twist.linear.x,twist.twist.linear.y,twist.twist.linear.z=s['v'];self.pubs[v]['twist'].publish(twist)
            platform=PlatformInfo();platform.status.state=PlatformStatus.FLYING;self.pubs[v]['platform'].publish(platform)
            controller=ControllerInfo();controller.header.stamp=stamp;controller.input_control_mode=s['control']
            self.pubs[v]['controller'].publish(controller)
            origin=TransformStamped();origin.header.stamp=stamp;origin.header.frame_id='earth';origin.child_frame_id=v+'/odom'
            origin.transform.translation.y=index*4.;origin.transform.rotation.w=1.
            body=TransformStamped();body.header=copy.deepcopy(pose.header);body.child_frame_id=v+'/base_link'
            body.transform.translation.x,body.transform.translation.y,body.transform.translation.z=s['p'];body.transform.rotation=pose.pose.orientation
            self.tf.sendTransform([origin,body])

    def cloud_points(self,vehicle,index,state):
        return [(5.,5.,2.)]

    def until(self,predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.005)
            if predicate():return
        raise AssertionError('timeout: '+json.dumps(self.status))

    def goals(self,y=0.):
        for index,v in enumerate(self.vehicles):
            goal=GoalSet(drone_id=index);goal.goal=[1.5,index*4.+y,2.];self.goal_pub.publish(goal)

    def run(self):
        self.until(lambda:self.goal_pub.get_subscription_count()>0,15)
        self.goals()
        self.until(lambda:(self.status.get('revision',0)>=3 and all(len(x)>=60 for x in self.refs.values()))
                   or self.status.get('state')=='FAILED',25)
        assert self.status['state']!='FAILED',self.status
        assert self.status['counts']['trajectories']>=4
        if mode=='complete':
            self.goals(.2)
            self.until(lambda:self.status.get('state') in ['SUCCEEDED','FAILED'],50)
            assert self.status['state']=='SUCCEEDED',self.status
            assert self.status['stop_confirmed']
            for vehicle in self.vehicles:
                assert math.dist(self.state[vehicle]['p'],[1.5,.2,2.])<.04
        elif mode=='cancel':
            future=self.cancel.call_async(Trigger.Request())
            self.until(future.done,5);assert future.result().success
        elif mode=='stale':self.skip_cloud=True
        elif mode=='infeasible':
            rows=(project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/unsafe-baseline.jsonl').read_text().splitlines()
            row=next(json.loads(x) for x in rows if json.loads(x)['kind']=='trajectory')
            bad=PolyTraj(drone_id=0,traj_id=999,order=5);bad.start_time=self.get_clock().now().to_msg()
            for field in ['duration','coef_x','coef_y','coef_z']:setattr(bad,field,[float(x) for x in row[field]])
            self.bad_pub.publish(bad)
        else:raise ValueError(mode)
        if mode!='complete':
            self.until(lambda:self.status.get('state')=='FAILED' and self.status.get('stop_confirmed'),10)
            expected={'cancel':'CANCELED','stale':'STALE_INPUT','infeasible':'INVALID_TRAJECTORY'}[mode]
            assert self.status['reason'].startswith(expected),self.status
            count=sum(map(len,self.refs.values()))
            deadline=time.monotonic()+.5
            while time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.01)
            assert sum(map(len,self.refs.values()))==count,'references continued after confirmed stop'
        assert max(self.odom_errors,default=999)<1e-9
        assert all(n>10 for n in self.observed.values()),self.observed
        assert self.pva_max[0]<=.2415 and self.pva_max[1]<=.3675,self.pva_max
        assert all(len(v)>50 for v in self.refs.values())

def main():
    global out,mode
    out=Path(sys.argv[1]).resolve()
    if out.exists() and any(out.iterdir()):raise ValueError('evidence directory must be empty')
    out.mkdir(parents=True,exist_ok=True)
    mode=sys.argv[2]
    rclpy.init();node=Harness();processes=[];logs=[];passed=False;error=''
    try:
        config=project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/dual.yaml'
        commands=[['ros2','launch','uav_ego_planner_adapter','ego_as2.launch.py',f'resolved_config:={config}']]
        for v in node.vehicles:
            commands.append(['ros2','run','uav_follow_path_execution','follow_path_extended_node','--ros-args',
                '--params-file',str(Path(get_package_share_directory('as2_behaviors_motion'))/'follow_path_behavior/config/config_default.yaml'),
                '-r',f'__ns:=/{v}','-p','plugin_name:=follow_path_plugin_position','-p',f'vehicle_id:={v}',
                '-p','follow_path_speed:=0.2415','-p','follow_path_threshold:=0.2','-p','tf_timeout_threshold:=0.1'])
        for i,command in enumerate(commands):
            stream=(out/f'process-{i}.log').open('w');logs.append(stream)
            processes.append(subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
        node.run();passed=True
    except Exception as exc:
        error=repr(exc)
    finally:
        for process in processes:
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
        cleanup=[]
        for process in processes:
            try:cleanup.append(process.wait(timeout=8))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait();cleanup.append('forced kill')
        for stream in logs:stream.close()
        report={'passed':passed and 'forced kill' not in cleanup,'mode':mode,'error':error,'status':node.status,
                'trajectory_counts':{v:len(x) for v,x in node.polys.items()},'motion_reference_counts':{v:len(x) for v,x in node.refs.items()},
                'max_execution_velocity_acceleration':node.pva_max,'max_input_conversion_error':max(node.odom_errors,default=None),
                'nonzero_velocity_observations':node.observed,'cleanup':cleanup,
                'scope':'real EGO + adapter + compiled AS2 FollowPath + ROS Action/motion reference consumption; synthetic controller/state/cloud, no PX4/AirSim'}
        (out/'result.json').write_text(json.dumps(report,indent=2));(out/'references.json').write_text(json.dumps(node.refs))
        (out/'polynomials.json').write_text(json.dumps({v:[{'start_ns':nanoseconds(p.start_time),
            'duration':list(p.duration),'coef_x':list(p.coef_x),'coef_y':list(p.coef_y),'coef_z':list(p.coef_z)}
            for p in rows] for v,rows in node.polys.items()}))
        node.destroy_node();rclpy.shutdown();print(json.dumps(report,indent=2))
        sys.exit(0 if report['passed'] else 1)

if __name__ == '__main__':main()
