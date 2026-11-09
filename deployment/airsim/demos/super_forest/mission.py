"""Isolated PC forest8 mission through SUPER committed plans and AS2 Behavior."""
import collections
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import Takeoff,Land
from as2_msgs.msg import ControllerInfo,ControlMode,PlatformInfo,PlatformStatus,TrajectorySetpoints
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleCommandAck
from std_msgs.msg import String
from std_srvs.srv import SetBool
import yaml
from airsim_rpc import Rpc


class Mission(Node):
    def __init__(self):
        super().__init__('super_forest_migration',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.root=Path('/opt/super_app');self.out=Path('/evidence')
        self.task=json.loads((self.root/'source-task.json').read_text())
        self.rpc=Rpc('127.0.0.1',43472,timeout=5)
        self.phase='initializing';self.odom=None;self.platform=None;self.controller=None
        self.state={};self.acks={};self.counts=collections.Counter();self.flags=collections.Counter()
        self.received={};self._mission_clients=[];self.children=[];self.origin=None
        self.max_speed=0.;self.bound_violations=0;self.collision_stamp=None
        self.mode_lost_at=None
        self.last_collision=0.;self.started=None;self.result={'execution_backend':'follow_path'}
        self.samples=(self.out/'flight.jsonl').open('w')
        self.create_subscription(Odometry,'/drone0/sensor_measurements/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(PlatformInfo,'/drone0/platform/info',lambda m:setattr(self,'platform',m),qos_profile_sensor_data)
        self.create_subscription(ControllerInfo,'/drone0/controller/info',lambda m:self.on_controller(m),qos_profile_sensor_data)
        self.create_subscription(String,'/drone0/super_adapter/state',lambda m:setattr(self,'state',json.loads(m.data)),10)
        self.create_subscription(VehicleCommandAck,f'/drone0/fmu/out/vehicle_command_ack_v{VehicleCommandAck.MESSAGE_VERSION}',
            lambda m:self.acks.__setitem__(m.command,m.result),qos_profile_sensor_data)
        self.create_subscription(Clock,'/clock',lambda m:self.touch('clock'),qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoints,'/drone0/motion_reference/trajectory',self.on_reference,qos_profile_sensor_data)
        from mars_quadrotor_msgs.msg import PositionCommand,PolynomialTrajectory
        self.create_subscription(PositionCommand,'/planning/pos_cmd',self.on_command,qos_profile_sensor_data)
        self.create_subscription(PolynomialTrajectory,'/planning_cmd/poly_traj',self.on_plan,qos_profile_sensor_data)
        for key,topic in [('cloud','/drone0/navigation/points'),('map','/rog_map/occ'),('inflated','/rog_map/inf_occ')]:
            self.create_subscription(PointCloud2,topic,lambda m,k=key:self.on_cloud(k,m),qos_profile_sensor_data)
        self.goal_pub=self.create_publisher(PoseStamped,'/goal_pose',1)
        self.create_timer(.1,self.record)

    def touch(self,key):
        self.counts[key]+=1;self.received[key]=time.monotonic()

    def now_seconds(self):return self.get_clock().now().nanoseconds/1e9

    def on_odom(self,msg):
        if msg.header.frame_id!='drone0/odom':raise RuntimeError('unexpected odometry frame')
        self.odom=msg;self.touch('odom')
        v=msg.twist.twist.linear
        speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
        if self.phase=='route':self.max_speed=max(self.max_speed,speed)

    def on_controller(self,msg):self.controller=msg;self.touch('controller')

    def on_cloud(self,key,msg):
        if msg.width*msg.height:self.touch(key)

    def on_reference(self,msg):
        if self.phase!='route':return
        self.touch('reference')
        if msg.header.frame_id!='drone0/odom' or len(msg.setpoints)!=1:raise RuntimeError('invalid Behavior reference')
        p=msg.setpoints[0]
        vectors=[p.position,p.twist,p.acceleration]
        if not all(math.isfinite(v) for vector in vectors for v in (vector.x,vector.y,vector.z)):
            raise RuntimeError('nonfinite Behavior PVA')
        self.samples.write(json.dumps({'kind':'reference','stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
            'p':[p.position.x,p.position.y,p.position.z], 'v':[p.twist.x,p.twist.y,p.twist.z],
            'a':[p.acceleration.x,p.acceleration.y,p.acceleration.z]})+'\n')

    def on_plan(self,msg):
        if not msg.type & msg.POSITION_TRAJ:return
        self.samples.write(json.dumps({'kind':'plan','generated':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
            'start':msg.start_wt_pos,'durations':list(msg.time_pos),'order':msg.order_pos,
            'axes':[list(msg.coef_pos_x),list(msg.coef_pos_y),list(msg.coef_pos_z)]})+'\n')

    def on_command(self,msg):
        if self.phase=='route':
            self.flags[str(msg.trajectory_flag)]+=1
            self.samples.write(json.dumps({'kind':'source_command','flag':msg.trajectory_flag,
                'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9})+'\n')

    def position(self):
        p=self.odom.pose.pose.position
        return [p.x,p.y,p.z]

    def local(self):return [a-b for a,b in zip(self.position(),self.origin)]

    def record(self):
        if not self.odom:return
        self.samples.write(json.dumps({'phase':self.phase,'sim_time':self.now_seconds(),
            'position':self.position(),'adapter':self.state})+'\n');self.samples.flush()

    def healthy(self):
        for name,child,_ in self.children:
            if self.phase in ('stopping','landing') and name!='as2':continue
            if child.poll() is not None:raise RuntimeError(name+' exited: '+str(child.returncode))
        if self.phase=='route':
            if self.state.get('error'):raise RuntimeError(self.state['error'])
            if time.monotonic()-self.received.get('controller',0)>1.5:
                raise RuntimeError('controller feedback timeout')
            if any(time.monotonic()-self.received.get(k,0)>.5 for k in ('clock','odom')):
                raise RuntimeError('state/clock timeout')
            if (self.controller.input_control_mode.control_mode!=ControlMode.TRAJECTORY and not self.state.get('complete')
                and not (self.state.get('settling') and self.controller.input_control_mode.control_mode==ControlMode.POSITION)):
                if self.mode_lost_at is None:self.mode_lost_at=time.monotonic()
                if time.monotonic()-self.mode_lost_at>.3:raise RuntimeError('Behavior trajectory mode lost')
            else:self.mode_lost_at=None
            p=self.local()
            if any(v<a or v>b for v,a,b in zip(p,self.task['bounds_enu_min_m'],self.task['bounds_enu_max_m'])):
                self.bound_violations+=1  # Original task policy is observe.
            if time.monotonic()-self.last_collision>.2:
                collision=self.rpc.call('simGetCollisionInfo','PX4');self.last_collision=time.monotonic()
                if collision['has_collided'] and collision['time_stamp']!=self.collision_stamp:
                    self.result['collision']=collision
                    raise RuntimeError('new AirSim collision')

    def wait(self,predicate,timeout=15):
        deadline=time.monotonic()+timeout
        while not predicate():
            self.healthy()
            if time.monotonic()>deadline:raise TimeoutError('timeout during '+self.phase)
            rclpy.spin_once(self,timeout_sec=.02)

    def start(self,name,command):
        stream=(self.out/(name+'.log')).open('w')
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.children.append((name,child,stream))

    def service(self,kind,name,request):
        client=self.create_client(kind,name);self._mission_clients.append(client)
        self.wait(client.service_is_ready)
        future=client.call_async(request);self.wait(future.done)
        if not future.result().success:raise RuntimeError(name+' rejected request: '+str(future.result()))

    def platform_service(self,name,command,value):
        self.acks.pop(command,None)
        self.service(SetBool,name,SetBool.Request(data=value))
        self.wait(lambda:command in self.acks)
        if self.acks[command]!=VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError('PX4 rejected command '+str(command)+': '+str(self.acks[command]))

    def action(self,kind,name,goal,field):
        client=ActionClient(self,kind,name);self._mission_clients.append(client)
        self.wait(client.server_is_ready)
        future=client.send_goal_async(goal);self.wait(future.done)
        handle=future.result()
        if not handle.accepted:raise RuntimeError(name+' rejected goal')
        result=handle.get_result_async()
        try:self.wait(result.done,90)
        except BaseException:handle.cancel_goal_async();raise
        if not getattr(result.result().result,field):raise RuntimeError(name+' failed')

    def prepare(self):
        os.environ.update(AS2_EXECUTION_BACKEND='follow_path',AS2_FLEET_FILE='/evidence/fleet.yaml')
        self.start('as2',['python3','/opt/onboard_computer/run.py'])
        self.start('navigation',['ros2','run','as2_navigation_inputs','navigation_inputs_node','--ros-args',
            '-r','__ns:=/drone0','--params-file',str(self.root/'navigation.yaml')])
        self.wait(lambda:self.odom is not None and self.platform is not None and self.controller is not None and self.counts['cloud']>=3,90)
        settled=self.now_seconds();self.wait(lambda:self.now_seconds()-settled>=10,30)
        self.origin=self.position()
        target=[a+b for a,b in zip(self.origin,self.task['waypoints_enu_m'][-1])]
        config=yaml.safe_load((self.root/'planner.yaml').read_text())
        historical=json.loads((self.root/'origin.json').read_text())['task_to_odom_translation_m']
        config['rog_map']['fix_map_origin']=[a+b-c for a,b,c in zip(config['rog_map']['fix_map_origin'],self.origin,historical)]
        config['rog_map']['virtual_ground_height']=self.task['virtual_ground_height_m']+self.origin[2]
        config['rog_map']['virtual_ceil_height']=self.task['virtual_ceil_height_m']+self.origin[2]
        (self.out/'planner.yaml').write_text(yaml.safe_dump(config))
        (self.out/'origin-measured.json').write_text(json.dumps({'task_to_odom_translation_m':self.origin,'target':target},indent=2)+'\n')
        self.start('super',['ros2','run','super_planner','fsm_node','--ros-args','-p','use_sim_time:=true','-p','config_name:=/evidence/planner.yaml'])
        self.start('adapter',['ros2','run','uav_super_adapter','super_timed_path_adapter','--ros-args',
            '-r','__ns:=/drone0','-p','use_sim_time:=true','-p','goal:='+str(target),
            '-p','goal_tolerance:='+str(float(self.task['goal_tolerance_m']))])
        self.wait(lambda:self.counts['map']>=3 and self.counts['inflated']>=3,60)
        return target

    def run(self):
        try:
            self.phase='preparing';target=self.prepare()
            self.phase='arming'
            self.platform_service('/drone0/set_arming_state',400,True)
            self.wait(lambda:self.platform.armed)
            self.platform_service('/drone0/set_offboard_mode',176,True)
            self.wait(lambda:self.platform.offboard)
            self.phase='takeoff'
            self.action(Takeoff,'/drone0/TakeoffBehavior',Takeoff.Goal(takeoff_height=2.,takeoff_speed=.5),'takeoff_success')
            self.wait(lambda:self.platform.status.state==PlatformStatus.FLYING)
            self.phase='holding';hold=self.now_seconds();self.wait(lambda:self.now_seconds()-hold>=6,20)
            self.collision_stamp=self.rpc.call('simGetCollisionInfo','PX4')['time_stamp']
            self.service(SetBool,'/drone0/super_adapter/enable',SetBool.Request(data=True))
            self.wait(lambda:self.goal_pub.get_subscription_count()>0)
            goal=PoseStamped();goal.header.frame_id='drone0/odom';goal.header.stamp=self.get_clock().now().to_msg()
            goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=target
            goal.pose.orientation=self.odom.pose.pose.orientation
            self.goal_pub.publish(goal)
            def executing():
                if self.state.get('error'):raise RuntimeError(self.state['error'])
                return self.state.get('revision',0)>0 and self.controller.input_control_mode.control_mode==ControlMode.TRAJECTORY
            self.wait(executing,15)
            self.phase='route';self.started=self.now_seconds();stable=None
            def arrived():
                nonlocal stable
                v=self.odom.twist.twist.linear
                speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
                if math.dist(self.position(),target)<=self.task['goal_tolerance_m'] and speed<.3:
                    if stable is None:stable=self.now_seconds()
                else:stable=None
                return stable is not None and self.now_seconds()-stable>=2 and self.state.get('complete',False)
            self.wait(arrived,self.task['leg_timeout_s'])
            self.result.update(arrived=True,execution_completed=bool(self.state.get('complete')),arrival_error_m=math.dist(self.position(),target),
                route_sim_seconds=self.now_seconds()-self.started,final_task_enu_m=self.local())
        except BaseException as exc:
            self.result.update(error=str(exc),failed_phase=self.phase)
            self.get_logger().error(str(exc))
        finally:
            self.phase='stopping'
            if self.platform is not None and self.platform.armed:
                try:
                    self.service(SetBool,'/drone0/super_adapter/enable',SetBool.Request(data=False))
                    if self.state.get('task_id'):
                        self.wait(lambda:self.state.get('stop_confirmed'),10)
                        self.result['behavior_stop_confirmed']=True
                    else:self.result['behavior_stop_not_needed']=True
                except BaseException as exc:
                    self.result['stop_error']=str(exc)
                    request=SetControlMode.Request();request.control_mode.control_mode=ControlMode.HOVER
                    request.control_mode.yaw_mode=ControlMode.YAW_ANGLE
                    try:self.service(SetControlMode,'/drone0/controller/set_control_mode',request)
                    except BaseException as failure:self.result['hover_error']=str(failure)
                try:
                    self.phase='landing'
                    if self.platform.status.state in (PlatformStatus.TAKING_OFF,PlatformStatus.FLYING,PlatformStatus.LANDING):
                        self.action(Land,'/drone0/LandBehavior',Land.Goal(land_speed=.5),'land_success')
                    else:
                        self.platform_service('/drone0/set_arming_state',400,False)
                    self.wait(lambda:not self.platform.armed,20)
                    self.result['landed_and_disarmed']=True
                except BaseException as exc:self.result['recovery_error']=str(exc)
            self.result.update(success=bool(self.result.get('arrived') and self.result.get('behavior_stop_confirmed') and
                 self.result.get('landed_and_disarmed') and not self.result.get('error')),
                 max_speed_m_s=self.max_speed,bounds_observation_count=self.bound_violations,
                 source_flags=dict(self.flags),counts=dict(self.counts),adapter=self.state)
            (self.out/'result.json').write_text(json.dumps(self.result,indent=2)+'\n')
            print(json.dumps(self.result,indent=2),flush=True)

    def close(self):
        for _,child,_ in self.children:
            if child.poll() is None:
                try:os.killpg(child.pid,signal.SIGTERM)
                except ProcessLookupError:pass
        for _,child,stream in self.children:
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
            stream.close()
        self.samples.close();self.destroy_node()


def main():
    rclpy.init();node=Mission()
    try:node.run()
    finally:node.close();rclpy.try_shutdown()
    if not node.result.get('success'):raise SystemExit(1)

if __name__=='__main__':main()
