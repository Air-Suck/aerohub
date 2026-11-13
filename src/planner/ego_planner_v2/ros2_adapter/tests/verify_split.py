"""Two real planners/Adapters/AS2 hosts; synthetic sensors/controller, no flight.

Run inside an isolated network namespace/container. Checks local goals, Action
updates, cancellation/fault isolation, peer mask and original-time PVA.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from quadrotor_msgs.msg import GoalSet
from traj_utils.msg import MINCOTraj
from uav_planner_interfaces.msg import TimedTrajectorySegment, PlannerFailure
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from uav_ego_planner_adapter.conversion import Polynomial
from verify_as2 import Harness, project, nanoseconds
from tail_reference_check import compare_references, segment_rows


class SplitHarness(Harness):
    def __init__(self):
        super().__init__(scoped=True)
        self.status={v:{} for v in self.vehicles}
        self.goals_by_vehicle={};self.cancels={};self.segments={v:[] for v in self.vehicles}
        self.failures={v:[] for v in self.vehicles};self.stops={v:0 for v in self.vehicles}
        self.native_clouds={v:[] for v in self.vehicles};self.broadcast_ids=set()
        self.checks={};self.graph={}
        self.pva_errors={}
        self.hold_reference_counts={}
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for v in self.vehicles:
            self.goals_by_vehicle[v]=self.create_publisher(GoalSet,f'/{v}/ego/goal',qos)
            self.cancels[v]=self.create_client(Trigger,f'/{v}/ego/cancel')
            self.create_subscription(String,f'/{v}/ego/status',lambda m,v=v:self.status.update({v:json.loads(m.data)}),10)
            self.create_subscription(Empty,f'/{v}/ego_planner/mandatory_stop',lambda m,v=v:self.stops.update({v:self.stops[v]+1}),10)
            self.create_subscription(PlannerFailure,f'/{v}/planner/failure',lambda m,v=v:self.failures[v].append(m.reason),10)
            self.create_subscription(TimedTrajectorySegment,f'/{v}/planner/trajectory',lambda m,v=v:self.segments[v].append(m),10)
            self.create_subscription(PointCloud2,f'/{v}/ego/native/points',lambda m,v=v:self.native_clouds[v].append(
                list(point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True))),qos_profile_sensor_data)
        self.create_subscription(MINCOTraj,'/ego_swarm/trajectory',lambda m:self.broadcast_ids.add(m.drone_id),10)

    def cloud_points(self,vehicle,index,state):
        peer='drone1' if index==0 else 'drone0'
        p=self.state[peer]['p']
        return [(5.,5.,2.),(p[0],p[1]+(4. if index==0 else -4.),p[2])]

    def check(self,name,condition):
        self.checks[name]=bool(condition)
        assert condition,name+': '+json.dumps(self.status)

    def settle(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.005)

    def send(self,v,identity=None):
        index=self.vehicles.index(v)
        self.goals_by_vehicle[v].publish(GoalSet(drone_id=index if identity is None else identity,
                                               goal=[1.5,index*4.,2.]))

    def run_split(self,mode):
        self.until(lambda:all(self.status[v] and self.goals_by_vehicle[v].get_subscription_count()==1
                             and self.cancels[v].service_is_ready() and len(self.native_clouds[v])>5
                             for v in self.vehicles),20)
        self.settle(1)
        for v in self.vehicles:
            publishers=self.get_publisher_names_and_types_by_node('ego_as2_adapter','/'+v)
            subscribers=self.get_subscriber_names_and_types_by_node('ego_as2_adapter','/'+v)
            services=self.get_service_names_and_types_by_node('ego_as2_adapter','/'+v)
            self.graph[v]=dict(publishers=publishers,subscribers=subscribers,services=services)
            pubs={name for name,_ in publishers};subs={name for name,_ in subscribers}
            other='drone1' if v=='drone0' else 'drone0'
            self.check(v+'_ownership',self.status[v]['vehicles']==[v] and self.status[v]['interface_scope']==v)
            self.check(v+'_local_output',f'/{v}/planner/trajectory' in pubs and f'/{v}/ego/status' in pubs
                       and not any(name.startswith('/'+other+'/') for name in pubs))
            self.check(v+'_peer_readonly',f'/{other}/navigation/odom' in subs
                       and f'/{other}/ego_planner/planning/trajectory' not in subs)
        all_names={n for n,_ in self.get_topic_names_and_types()}
        self.check('no_global_control_topics',not all_names & {'/ego/goal','/ego/status','/planner/failure','/planner/trajectory'})
        self.check('no_global_cancel',not self.count_services('/ego/cancel'))
        # Only Drone0 gets a goal: it must execute without waiting for Drone1.
        self.send('drone0')
        self.until(lambda:len(self.refs['drone0'])>=25 or self.status['drone0'].get('state')=='FAILED',20)
        self.check('independent_start',self.status['drone0']['state']=='RUNNING' and not self.refs['drone1']
                   and self.status['drone1']['counts']['goals']==0 and self.status['drone1']['state']=='WAITING')
        self.send('drone1',identity=0)
        self.until(lambda:self.status['drone1'].get('rejected',{}).get('goal',0)>=1,5)
        self.check('wrong_id_rejected',self.status['drone1']['counts']['goals']==0 and not self.refs['drone1'])
        self.send('drone1')
        self.until(lambda:all(len(self.refs[v])>=60 and self.status[v]['revision']>=2 for v in self.vehicles)
                   or any(s.get('state')=='FAILED' for s in self.status.values()),25)
        self.check('both_execute',all(self.status[v]['state']=='RUNNING' and len(self.refs[v])>=60 for v in self.vehicles))
        target='drone0' if mode=='cancel' else 'drone1'
        other='drone1' if target=='drone0' else 'drone0'
        if mode in ('cancel','stale'):
            if mode=='cancel':
                future=self.cancels[target].call_async(Trigger.Request())
                self.until(future.done,5)
                self.check('cancel_response_local',future.result().success and f'/{target}/ego/status' in future.result().message)
            else:self.skip_cloud=True
            self.until(lambda:self.status[target]['state']=='FAILED' and self.status[target]['stop_confirmed'],8)
            reason='CANCELED' if mode=='cancel' else 'STALE_INPUT'
            self.check('local_failure',self.status[target]['reason'].startswith(reason) and self.failures[target]==[reason])
            self.check('peer_not_stopped',self.status[other]['state']=='RUNNING' and not self.failures[other] and self.stops[other]==0)
            self.settle(.5)
            old_refs=len(self.refs[target]);peer_refs=len(self.refs[other]);old_goals=self.status[target]['counts']['goals']
            self.skip_cloud=False
            rejected=self.status[target]['rejected'].get('goal',0)
            self.send(target)
            self.until(lambda:self.status[target]['rejected'].get('goal',0)>rejected and len(self.refs[other])>peer_refs+25,5)
            self.check('closed_local_task',len(self.refs[target])==old_refs and self.status[target]['counts']['goals']==old_goals)
            self.until(lambda:self.status[other]['state'] in ('SUCCEEDED','FAILED'),45)
            self.check('peer_completes',self.status[other]['state']=='SUCCEEDED' and self.status[other]['stop_confirmed'])
        else:
            self.until(lambda:all(s['state'] in ('SUCCEEDED','FAILED') for s in self.status.values()),50)
            self.check('both_complete',all(s['state']=='SUCCEEDED' and s['stop_confirmed'] for s in self.status.values()))
        for index,v in enumerate(self.vehicles):
            self.check(v+'_segment_ownership',len(self.segments[v])>=2 and all([x.vehicle_id for x in s.vehicle_trajectories]==[v] for s in self.segments[v]))
            self.check(v+'_peer_mask',self.status[v]['counts'].get('peer_points_removed_'+v,0)>10
                       and all(len(points)==1 for points in self.native_clouds[v]))
            sources=[Polynomial(p,index,.2415,.3675) for p in self.polys[v]]
            self.pva_errors[v],self.hold_reference_counts[v]=compare_references(
                sources,self.refs[v],self.segments[v],v,[0,index*4.,0],[1.5,index*4.,2.])
            self.check(v+'_original_time_pva',bool(self.refs[v]) and self.pva_errors[v]<=1e-4)
        self.check('shared_native_broadcast',self.broadcast_ids=={0,1})
        self.check('input_frames_time_twist',max(self.odom_errors,default=999)<1e-9 and all(n>10 for n in self.observed.values()))
        self.check('velocity_acceleration_bounds',self.pva_max[0]<=.2415 and self.pva_max[1]<=.3675)

    def count_services(self,name):
        return sum(n==name for n,_ in self.get_service_names_and_types())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path);parser.add_argument('mode',choices=('complete','cancel','stale'))
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    config=yaml.safe_load((project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/dual.yaml').read_text())
    config['peer_body_mask_radius_m']=.75
    (out/'config.yaml').write_text(yaml.safe_dump(config))
    rclpy.init();node=SplitHarness();processes=[];logs=[];passed=False;error=''
    commands=[]
    for v in node.vehicles:
        commands.append(['ros2','launch','uav_ego_planner_adapter','ego_as2.launch.py',
                         f'resolved_config:={out}/config.yaml',f'vehicle:={v}'])
        commands.append(['ros2','run','uav_follow_path_execution','follow_path_extended_node','--ros-args',
            '--params-file',str(Path(get_package_share_directory('as2_behaviors_motion'))/'follow_path_behavior/config/config_default.yaml'),
            '-r',f'__ns:=/{v}','-p','plugin_name:=follow_path_plugin_position','-p',f'vehicle_id:={v}',
            '-p','follow_path_speed:=0.2415','-p','follow_path_threshold:=0.2','-p','tf_timeout_threshold:=0.1'])
    (out/'commands.json').write_text(json.dumps(commands,indent=2))
    try:
        for i,cmd in enumerate(commands):
            stream=(out/f'process-{i}.log').open('w');logs.append(stream)
            processes.append(subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
        node.run_split(args.mode);passed=True
    except Exception as exc:error=repr(exc)
    finally:
        # Keep synthetic controller feedback alive while draining cancellations;
        # never tear down an AS2 action server during its pending result callback.
        observed_status=copy.deepcopy(node.status)
        for v in node.vehicles:
            if not node.status[v].get('stop_confirmed') and node.cancels[v].service_is_ready():
                node.cancels[v].call_async(Trigger.Request())
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and not all(s.get('stop_confirmed') for s in node.status.values()):
            rclpy.spin_once(node,timeout_sec=.01)
        cleanup_status=copy.deepcopy(node.status)
        for p in processes[::2]:
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and any(p.poll() is None for p in processes[::2]):
            rclpy.spin_once(node,timeout_sec=.01)
        node.settle(.5)
        for p in processes[1::2]:
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
        cleanup=[]
        for p in processes:
            try:cleanup.append(p.wait(timeout=10))
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL);p.wait();cleanup.append('forced kill')
        for stream in logs:stream.close()
        child_logs=[(out/f'process-{i}.log').read_text() for i in (0,2)]
        node.checks['clean_child_exit']=all(s.count('process has finished cleanly')==2 and
            not any(text in s for text in ('process has died','escalating','SIGKILL','Traceback')) for s in child_logs)
        result=dict(passed=passed and all(c==0 for c in cleanup),mode=args.mode,error=error,
            checks=node.checks,status=observed_status,cleanup_status=cleanup_status,cleanup=cleanup,stop_counts=node.stops,failures=node.failures,
            reference_counts={v:len(rows) for v,rows in node.refs.items()},
            original_time_pva_error=node.pva_errors,
            hold_reference_counts=node.hold_reference_counts,
            scope='real planners, two Adapters and AS2 hosts; synthetic controller/sensors; no flight or RoboPi deployment')
        result['passed']=result['passed'] and all(node.checks.values())
        for name,data in [('result',result),('graph',node.graph),('references',node.refs),
                          ('segments',{v:segment_rows(rows) for v,rows in node.segments.items()}),
                          ('polynomials',{v:[{'start_ns':nanoseconds(p.start_time),'duration':list(p.duration),
                           'coef_x':list(p.coef_x),'coef_y':list(p.coef_y),'coef_z':list(p.coef_z)} for p in rows]
                           for v,rows in node.polys.items()})]:
            (out/(name+'.json')).write_text(json.dumps(data,indent=2))
        node.destroy_node();rclpy.shutdown();print(json.dumps(result,indent=2))
        return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
