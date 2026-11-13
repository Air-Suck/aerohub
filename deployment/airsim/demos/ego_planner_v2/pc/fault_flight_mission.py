"""Live cancellation/fault acceptance; recovery policy is stop, hold, native land.

Inputs are gated only at the planner boundary. AS2 localization-loss recovery is
outside this test's scope. A new goal must not reopen a closed task.
"""
import argparse
import json
import math
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray
from as2_msgs.msg import ControlMode, BehaviorStatus
from as2_msgs.srv import SetControlMode
from px4_msgs.msg import OffboardControlMode
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from quadrotor_msgs.msg import GoalSet
from dual_flight_mission import Fleet


class FaultMission(Fleet):
    def __init__(self, case, fleet):
        self.minimum_initial_separation=3.
        self.check_independent_route_tube=False
        self.expect_failure=False
        self.injected=None
        self.injector={}
        self.timeline=[]
        self.next_sample=0.
        self.action_status={};self.behavior={};self.px4_modes={}
        super().__init__('dual')
        self.active=[0,1] if fleet=='dual' else [0]
        self.case=case
        self.report.update(stage='fault-'+fleet,case=case,active=self.active,
            recovery_policy='adapter fleet stop -> confirmed stationary hover -> mission native AS2 Land -> disarm',
            input_fault_scope='drone0 planner-only DDS input; AS2 localization stays live')
        self.report['gates'].update(detection_wall_s=1.5,stop_confirmation_wall_s=5.,
            physical_stop_wall_s=5.,stop_speed_m_s=.1,stable_sim_s=1.,
            post_stop_hold_sim_s=3.,max_residual_displacement_m=.75,
            max_post_stop_drift_m=.3,goal_distance_m=2.,max_horizontal_excursion_m=4.,
            expected_failure='CANCELED' if case=='cancel' else 'STALE_INPUT',
            no_old_task_resume=True,planner_input_timeout_wall_s=.75)
        (self.out/'criteria.json').write_text(json.dumps(self.report['gates'],indent=2))
        node=self.nodes[0]
        self.keep=[node.create_subscription(String,'/acceptance/injector_status',self.injector_status,10)]
        for i,n in enumerate(self.nodes):
            n.report['gates']['max_horizontal_excursion_m']=4.
            self.keep.extend([
                node.create_subscription(OffboardControlMode,f'/drone{i}/fmu/in/offboard_control_mode',
                    lambda m,k=i:self.px4_modes.update({k:m}),qos_profile_sensor_data),
                node.create_subscription(GoalStatusArray,f'/drone{i}/follow_timed_path/_action/status',
                    lambda m,k=i:self.action_status.update({k:[s.status for s in m.status_list]}),10),
                node.create_subscription(BehaviorStatus,f'/drone{i}/follow_timed_path/_behavior/behavior_status',
                    lambda m,k=i:self.behavior.update({k:m.status}),10)])

    def injector_status(self,m):
        self.injector=json.loads(m.data)

    def status(self,m):
        super().status(m)
        if self.injected is not None:
            elapsed=time.monotonic()-self.injected
            if self.ego_state.get('state')=='FAILED':self.report.setdefault('detection_wall_s',elapsed)
            if self.ego_state.get('stop_confirmed'):self.report.setdefault('stop_confirmation_wall_s',elapsed)

    def wait(self,predicate,timeout=15):
        end=time.monotonic()+timeout
        while True:
            self.executor.spin_once(timeout_sec=.005)
            if not self.recovering:
                for n in self.nodes:n.health()
                if self.initialized:
                    distance=math.dist(*[n.world(n.position()) for n in self.nodes])
                    self.min_separation=min(self.min_separation,distance)
                    if distance<2.:raise RuntimeError('minimum fleet separation violated')
                    if not self.expect_failure and self.ego_state.get('state')=='FAILED':
                        raise RuntimeError('unexpected EGO failure: '+self.ego_state.get('reason',''))
                if self.injected is not None and time.monotonic()>=self.next_sample:
                    self.next_sample=time.monotonic()+.05
                    self.timeline.append(dict(wall=time.time(),elapsed=time.monotonic()-self.injected,
                        sim=self.nodes[0].now_s(),positions=[n.position() for n in self.nodes],
                        speeds=[n.speed() for n in self.nodes],state=self.ego_state.copy(),
                        behavior=self.behavior.copy(),actions=self.action_status.copy(),
                        references=[n.counts.get('reference_trajectory',0) for n in self.nodes]))
            if predicate():return
            if time.monotonic()>end:raise TimeoutError('fault mission wait timeout; phases='+str([n.phase for n in self.nodes]))

    def prepare(self):
        super().prepare()
        self.wait(lambda:all(self.injector.get('counts',{}).get(k,{}).get('forwarded',0)>=10 for k in ('cloud','odom')),15)
        self.report['input_graph']={topic:dict(
            publishers=[e.node_name for e in self.nodes[0].get_publishers_info_by_topic(topic)],
            subscribers=[e.node_name for e in self.nodes[0].get_subscriptions_info_by_topic(topic)])
            for topic in ('/drone0/navigation/odom','/drone0/navigation/points',
                          '/acceptance/drone0/odom','/acceptance/drone0/points')}
        for i in self.active:
            n=self.nodes[i]
            c=n.create_client(SetControlMode,f'/drone{i}/controller/set_control_mode')
            self.wait(c.service_is_ready,5)
            f=c.call_async(SetControlMode.Request(control_mode=ControlMode(control_mode=ControlMode.HOVER)))
            self.wait(f.done,5);assert f.result().success
            self.wait(lambda:n.latest['controller'].output_control_mode.control_mode==ControlMode.SPEED,5)

    def arm(self):
        super().arm()
        for i in self.active:
            n=self.nodes[i];before=n.counts.get('px4_setpoint',0)
            mode=n.latest['controller'].output_control_mode
            assert mode.control_mode==ControlMode.SPEED
            c=n.create_client(SetControlMode,f'/drone{i}/set_platform_control_mode')
            self.wait(c.service_is_ready,5);f=c.call_async(SetControlMode.Request(control_mode=mode))
            self.wait(f.done,5);assert f.result().success
            self.wait(lambda:self.px4_modes.get(i) and self.px4_modes[i].velocity
                and not self.px4_modes[i].body_rate and n.counts.get('px4_setpoint',0)>=before+20,5)
            n.report['post_offboard_mode_restored']=True

    def hover(self):
        for i in self.active:self.nodes[i].phase='settling'
        stable=[None]
        def settled():
            if not all(self.nodes[i].speed()<=.15 and abs(self.nodes[i].position()[2]-self.nodes[i].origin[2]-2)<=.15 for i in self.active):
                stable[0]=None;return False
            if stable[0] is None:stable[0]=self.nodes[0].now_s()
            return self.nodes[0].now_s()-stable[0]>=2.
        self.wait(settled,20)
        super().hover()

    def routes(self):
        starts=np.array([n.world(n.position()) for n in self.nodes])
        baseline=starts[0,:2]-starts[1,:2]
        tangent=np.array([-baseline[1],baseline[0],0.]);tangent/=np.linalg.norm(tangent)
        for direction in [tangent,-tangent,np.array([1.,0,0]),np.array([-1.,0,0]),np.array([0,1.,0]),np.array([0,-1.,0])]:
            goals=starts.copy()
            goals[self.active]+=2*direction
            routes=[np.linspace(starts[i],goals[i],51) for i in (0,1)]
            if np.linalg.norm(routes[0][:,None,:]-routes[1][None,:,:],axis=2).min()<3.:continue
            clear=[]
            for i in self.active:
                n=self.nodes[i]
                if len(n.clouds)<10:raise RuntimeError('insufficient actual cloud frames')
                points=np.concatenate(n.clouds);points[:,0]-=4*i
                delta=goals[i]-starts[i]
                t=np.clip((points-starts[i])@delta/np.dot(delta,delta),0,1)
                clear.append(float(np.linalg.norm(points-(starts[i]+t[:,None]*delta),axis=1).min()))
            if min(clear)<.6:continue
            return starts,goals,clear
        raise RuntimeError('no clear separated 2 m route')

    def invoke(self,service,block=None):
        kind=Trigger if block is None else SetBool
        c=self.nodes[0].create_client(kind,service);self.wait(c.service_is_ready,5)
        req=Trigger.Request() if block is None else SetBool.Request(data=block)
        f=c.call_async(req);self.wait(f.done,5)
        response=f.result();assert response.success,response.message
        self.nodes[0].event(service=service,success=response.success,message=response.message)
        return response.message

    def ego(self):
        starts,goals,clear=self.routes()
        self.report['routes']=dict(starts_world_enu=starts.tolist(),goals_world_enu=goals.tolist(),clearances_m=clear)
        for i in self.active:
            n=self.nodes[i]
            n.report['ego_collision_before']=n.collision_checkpoint('before_ego')
            n.target=goals[i].copy();n.target[0]+=4*i
            n.report.update(ego_start_position=n.position(),goal_world_enu=goals[i].tolist())
            n.start_counts=n.counts.copy();n.phase='ego'
            self.goal_pub.publish(GoalSet(drone_id=i,goal=goals[i].tolist()))
        self.wait(lambda:self.ego_state.get('state')=='RUNNING' and all(
            len(self.nodes[i].references)>=20 and self.nodes[i].speed()>=.1
            and math.dist(self.nodes[i].world(self.nodes[i].position()),starts[i])>=.25 for i in self.active),30)
        self.report['pre_fault']=dict(ego_state=self.ego_state.copy(),
            positions=[n.position() for n in self.nodes],counts=[n.counts.copy() for n in self.nodes],injector=self.injector.copy())
        self.expect_failure=True;self.injected=time.monotonic()
        self.report['injection_wall']=time.time();self.report['injection_sim']=self.nodes[0].now_s()
        service={'cancel':'/ego/cancel','planner_exit':'/acceptance/exit_planner',
                 'cloud':'/acceptance/block_cloud','odom':'/acceptance/block_odom'}[self.case]
        self.report['injection_response']=self.invoke(service,True if self.case in ('cloud','odom') else None)
        self.wait(lambda:self.ego_state.get('state')=='FAILED',1.5)
        assert self.ego_state['reason'].startswith(self.report['gates']['expected_failure']+':'),self.ego_state
        assert self.report['detection_wall_s']<=1.5
        self.wait(lambda:self.ego_state.get('stop_confirmed'),5.)
        assert self.report['stop_confirmation_wall_s']<=5.
        stable=[None]
        def stopped():
            if not all(self.nodes[i].speed()<=.1 for i in self.active):stable[0]=None;return False
            if stable[0] is None:stable[0]=(self.nodes[0].now_s(),time.monotonic()-self.injected)
            return self.nodes[0].now_s()-stable[0][0]>=1.
        self.wait(stopped,5.)
        self.report['physical_stop_wall_s']=stable[0][1]
        assert stable[0][1]<=5.
        self.report['stopped_positions']=[n.position() for n in self.nodes]
        self.report['residual_displacement_m']=[max(math.dist(s['positions'][i],self.report['pre_fault']['positions'][i]) for s in self.timeline) for i in self.active]
        assert max(self.report['residual_displacement_m'])<=.75
        for i in self.active:self.nodes[i].phase='holding'
        # Drain in-flight DDS samples, then require no trajectory references.
        start=self.nodes[0].now_s();self.wait(lambda:self.nodes[0].now_s()-start>=.5,5.)
        reference_counts=[n.counts.get('reference_trajectory',0) for n in self.nodes]
        closed_revision=self.ego_state['revision'];closed_segments=self.ego_state['counts']['segments']
        hold_start=len(self.timeline)
        start=self.nodes[0].now_s();self.wait(lambda:self.nodes[0].now_s()-start>=3.,10.)
        assert all(self.behavior.get(i)==BehaviorStatus.IDLE for i in self.active),self.behavior
        assert all(self.action_status.get(i) and self.action_status[i][-1] in (5,6) for i in self.active),self.action_status
        self.report['terminal_action_status']=self.action_status.copy()
        if self.case in ('cloud','odom'):
            self.report['blocked_input_evidence']=self.injector.copy()
            counts=self.injector['counts'][self.case]
            assert counts['dropped']>=10,counts
            before=counts['forwarded']
            self.invoke(service,False)
            self.wait(lambda:self.injector['counts'][self.case]['forwarded']>=before+10,5.)
        rejected=self.ego_state.get('rejected',{}).get('goal',0)
        for i in self.active:self.goal_pub.publish(GoalSet(drone_id=i,goal=goals[i].tolist()))
        self.wait(lambda:self.ego_state.get('rejected',{}).get('goal',0)>=rejected+len(self.active),5.)
        start=self.nodes[0].now_s();self.wait(lambda:self.nodes[0].now_s()-start>=2.,8.)
        assert self.ego_state['state']=='FAILED' and self.ego_state['stop_confirmed']
        assert self.ego_state['revision']==closed_revision and self.ego_state['counts']['segments']==closed_segments
        assert all(n.counts.get('reference_trajectory',0)==reference_counts[i] for i,n in enumerate(self.nodes) if i in self.active)
        self.report['post_stop_drift_m']=[max(math.dist(s['positions'][i],self.report['stopped_positions'][i]) for s in self.timeline[hold_start:]) for i in self.active]
        assert max(self.report['post_stop_drift_m'])<=.3
        self.report['no_old_task_resume']=True
        self.report['final_injector']=self.injector.copy()
        for i in self.active:
            n=self.nodes[i]
            n.report['ego_collision_after']=n.collision_checkpoint('after_fault_stop')
            assert n.report['ego_collision_before']==n.report['ego_collision_after']
        self.report['fault_assertions_passed']=True

    def audit(self):
        for i in self.active:
            n=self.nodes[i];errors=[]
            for ref in n.references:
                errors.append(min((max(math.dist(p,ref['position']),math.dist(v,ref['velocity']),math.dist(a,ref['acceleration']))
                    for poly in n.polys if poly.start_ns<=ref['stamp_ns']<=poly.end_ns
                    for p,v,a in [poly.sample(ref['stamp_ns'])]),default=math.inf))
            n.report['max_prefault_pva_error']=max(errors,default=math.inf)
            assert errors and max(errors)<=1e-4,(i,max(errors,default=math.inf))

    def finish(self):
        (self.out/'fault-timeline.json').write_text(json.dumps(self.timeline,indent=2,default=float))
        self.report['final_injector']=self.injector
        super().finish()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--case',choices=['cancel','planner_exit','cloud','odom'],required=True)
    parser.add_argument('--fleet',choices=['single','dual'],required=True)
    args=parser.parse_args();rclpy.init();mission=FaultMission(args.case,args.fleet)
    try:mission.run()
    except BaseException as exc:
        import traceback;traceback.print_exc()
        mission.report.update(passed=False,error=repr(exc))
        try:mission.recover()
        except BaseException as recovery:mission.report['recovery_error']=repr(recovery)
    finally:mission.finish();rclpy.shutdown()
    raise SystemExit(0 if mission.report['passed'] else 1)
