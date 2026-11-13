"""Single-board fault acceptance using production EGO, AS2 and native landing.

Criteria and injection sequence follow the existing PC fault acceptance.
AS2 navigation remains live throughout planner-input faults.
"""
import json
import math
import time
from pathlib import Path
from action_msgs.msg import GoalStatusArray
from as2_msgs.msg import BehaviorStatus
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from quadrotor_msgs.msg import GoalSet
from ego_flight_mission import EgoMission


class FaultMission(EgoMission):
    def __init__(self,case):
        self.case=case
        self.expect_failure=False
        self.injected=None
        self.injector={}
        self.timeline=[]
        self.next_sample=0.
        self.action_status=[]
        self.behavior=None
        super().__init__(require_peer=False)
        self.report.update(mode='board-single-fault',case=case,
            recovery_policy='adapter stop -> stationary hover -> mission native Land -> disarm',
            input_fault_scope='planner-side only; AS2 navigation remains live')
        self.report['gates'].update(detection_wall_s=1.5,stop_confirmation_wall_s=5.,
            physical_stop_wall_s=5.,stop_speed_m_s=.1,stable_sim_s=1.,
            post_stop_hold_sim_s=3.,max_residual_displacement_m=.75,
            max_post_stop_drift_m=.3,goal_distance_m=2.,max_horizontal_excursion_m=4.,
            expected_failure='CANCELED' if case=='cancel' else 'STALE_INPUT',
            no_old_task_resume=True,planner_input_timeout_wall_s=.75)
        (self.out/'criteria.json').write_text(json.dumps(self.report['gates'],indent=2))
        self.fault_services = {
            '/ego/cancel':self.cancel_client,
            '/acceptance/exit_planner':self.create_client(Trigger,'/acceptance/exit_planner'),
            '/acceptance/block_cloud':self.create_client(SetBool,'/acceptance/block_cloud'),
            '/acceptance/block_odom':self.create_client(SetBool,'/acceptance/block_odom')}
        self.keep += [
            self.create_subscription(String,'/acceptance/injector_status',self.injector_status,10),
            self.create_subscription(GoalStatusArray,'/drone0/follow_timed_path/_action/status',self.timed_action_status,10),
            self.create_subscription(BehaviorStatus,'/drone0/follow_timed_path/_behavior/behavior_status',self.behavior_status,10)]
    def timed_action_status(self,msg):
        self.action_status=[s.status for s in msg.status_list]
        self.event(timed_action_status=self.action_status)

    def behavior_status(self,msg):
        self.behavior=msg.status
        self.event(timed_behavior_status=msg.status)

    def injector_status(self,msg):
        self.injector=json.loads(msg.data)

    def ego_status(self,msg):
        self.ego_state=json.loads(msg.data)
        self.event(ego_state=self.ego_state)
        if self.injected is not None:
            elapsed=time.monotonic()-self.injected
            if self.ego_state.get('state')=='FAILED':self.report.setdefault('detection_wall_s',elapsed)
            if self.ego_state.get('stop_confirmed'):self.report.setdefault('stop_confirmation_wall_s',elapsed)
        if self.phase=='ego' and not self.expect_failure and self.ego_state.get('state')=='FAILED':
            self.error='Unexpected EGO failure: '+self.ego_state.get('reason','')

    def health(self):
        super().health()
        if self.recovering:return
        if self.phase=='fault_hold' and (not self.latest['mode'].flag_armed or not self.latest['mode'].flag_control_offboard_enabled):
            raise RuntimeError('Armed/offboard feedback lost during fault hold')
        if self.injected is not None and time.monotonic()>=self.next_sample:
            self.next_sample=time.monotonic()+.05
            self.timeline.append(dict(wall=time.time(),elapsed=time.monotonic()-self.injected,
                sim=self.now_s(),position=self.position(),speed=self.speed(),state=self.ego_state.copy(),
                behavior=self.behavior,actions=self.action_status.copy(),
                references=self.counts.get('reference_trajectory',0)))

    def run(self):
        self.wait(lambda:all(self.injector.get('counts',{}).get(k,{}).get('forwarded',0)>=10 for k in ('odom','cloud')),20)
        self.wait(lambda:all(c.service_is_ready() for c in self.fault_services.values()),10)
        topics=('/drone0/navigation/odom','/drone0/navigation/points','/acceptance/drone0/odom','/acceptance/drone0/points')
        self.report['input_graph']={topic:dict(
            publishers=[e.node_name for e in self.get_publishers_info_by_topic(topic)],
            subscribers=[e.node_name for e in self.get_subscriptions_info_by_topic(topic)]) for topic in topics}
        for topic in topics[2:]:
            graph=self.report['input_graph'][topic]
            if graph['publishers']!=['acceptance_fault_injector'] or 'ego_as2_adapter' not in graph['subscribers']:
                raise RuntimeError('Planner-only input gate topology is not established')
        super().run()

    def invoke(self,service,block=None):
        kind=Trigger if block is None else SetBool
        client=self.fault_services[service]
        self.wait(client.service_is_ready,5)
        future=client.call_async(Trigger.Request() if block is None else SetBool.Request(data=block))
        self.wait(future.done,5)
        response=future.result()
        if not response.success:raise RuntimeError(service+': '+response.message)
        self.event(service=service,success=response.success,message=response.message)
        return response.message

    def after_hover(self):
        position=self.position()
        points=list(point_cloud2.read_points(self.latest['cloud'],field_names=('x','y','z'),skip_nans=True))
        candidates=[]
        for dx,dy in ((1.,0.),(0.,1.),(-1.,0.),(0.,-1.)):
            clearance=math.inf
            for q in points:
                along=max(0.,min(2.,(float(q[0])-position[0])*dx+(float(q[1])-position[1])*dy))
                clearance=min(clearance,math.dist([float(v) for v in q],[position[0]+along*dx,position[1]+along*dy,position[2]]))
            candidates.append(dict(direction=[dx,dy],cloud_clearance_m=clearance))
        chosen=next((c for c in candidates if c['cloud_clearance_m']>=.6),None)
        self.report['goal_candidates']=candidates
        if chosen is None:raise RuntimeError('No observed clear 2 m route')
        self.target=[position[0]+2*chosen['direction'][0],position[1]+2*chosen['direction'][1],position[2]]
        self.report.update(goal_enu=self.target,ego_start_position=position)
        self.wait(lambda:self.ego_state.get('state')=='WAITING' and self.goal_pub.get_subscription_count()>0,10)
        self.phase='ego'
        self.goal_pub.publish(GoalSet(drone_id=0,goal=self.target))
        self.wait(lambda:self.ego_state.get('state')=='RUNNING' and len(self.ego_refs)>=20 and
            self.speed()>=.1 and math.dist(self.position(),position)>=.25,30)
        self.report['pre_fault']=dict(ego_state=self.ego_state.copy(),position=self.position(),speed=self.speed(),
            counts=self.counts.copy(),injector=self.injector.copy())
        self.expect_failure=True
        self.injected=time.monotonic()
        self.report.update(injection_wall=time.time(),injection_sim=self.now_s())
        service={'cancel':'/ego/cancel','planner_exit':'/acceptance/exit_planner',
                 'cloud':'/acceptance/block_cloud','odom':'/acceptance/block_odom'}[self.case]
        self.report['injection_response']=self.invoke(service,True if self.case in ('cloud','odom') else None)
        self.wait(lambda:self.ego_state.get('state')=='FAILED',1.5)
        if not self.ego_state['reason'].startswith(self.report['gates']['expected_failure']+':') or self.report['detection_wall_s']>1.5:
            raise RuntimeError('Fault detection criteria failed: '+str(self.ego_state))
        self.wait(lambda:self.ego_state.get('stop_confirmed'),5)
        if self.report['stop_confirmation_wall_s']>5:raise RuntimeError('Stop confirmation exceeded 5 seconds')
        stable=[None]
        def stopped():
            if self.speed()>.1:stable[0]=None;return False
            if stable[0] is None:stable[0]=(self.now_s(),time.monotonic()-self.injected)
            return self.now_s()-stable[0][0]>=1.
        self.wait(stopped,5)
        self.report['physical_stop_wall_s']=stable[0][1]
        self.report['stopped_position']=self.position()
        self.report['residual_displacement_m']=max(math.dist(r['position'],self.report['pre_fault']['position']) for r in self.timeline)
        if stable[0][1]>5 or self.report['residual_displacement_m']>.75:raise RuntimeError('Physical stop criteria failed')
        self.phase='fault_hold'
        start=self.now_s();self.wait(lambda:self.now_s()-start>=.5,5)
        reference_count=self.counts.get('reference_trajectory',0)
        revision=self.ego_state['revision'];segments=self.ego_state['counts']['segments']
        hold_start=len(self.timeline)
        start=self.now_s();self.wait(lambda:self.now_s()-start>=3,10)
        if self.behavior!=BehaviorStatus.IDLE or not self.action_status or self.action_status[-1] not in (5,6):
            raise RuntimeError('Execution action/behavior did not terminate')
        self.report['terminal_action_status']=self.action_status.copy()
        if self.case in ('cloud','odom'):
            self.report['blocked_input_evidence']=self.injector.copy()
            counts=self.injector['counts'][self.case]
            if counts['dropped']<10:raise RuntimeError('Input was not actually blocked')
            before=counts['forwarded']
            self.invoke(service,False)
            self.wait(lambda:self.injector['counts'][self.case]['forwarded']>=before+10,5)
        rejected=self.ego_state.get('rejected',{}).get('goal',0)
        self.goal_pub.publish(GoalSet(drone_id=0,goal=self.target))
        self.wait(lambda:self.ego_state.get('rejected',{}).get('goal',0)>rejected,5)
        start=self.now_s();self.wait(lambda:self.now_s()-start>=2,8)
        if (self.ego_state['state']!='FAILED' or not self.ego_state['stop_confirmed'] or
                self.ego_state['revision']!=revision or self.ego_state['counts']['segments']!=segments or
                self.counts.get('reference_trajectory',0)!=reference_count):
            raise RuntimeError('Closed task resumed or emitted further references')
        self.report['post_stop_drift_m']=max(math.dist(r['position'],self.report['stopped_position']) for r in self.timeline[hold_start:])
        if self.report['post_stop_drift_m']>.3:raise RuntimeError('Post-stop drift exceeded 0.3 m')
        errors=[]
        for ref in self.ego_refs:
            errors.append(min((max(math.dist(p,ref['position']),math.dist(v,ref['velocity']),math.dist(a,ref['acceleration']))
                for poly in self.polys if poly.start_ns<=ref['stamp_ns']<=poly.end_ns for p,v,a in [poly.sample(ref['stamp_ns'])]),default=math.inf))
        self.report['max_prefault_pva_error']=max(errors,default=math.inf)
        if not errors or max(errors)>1e-4:raise RuntimeError('Native-to-AS2 PVA mismatch before stop')
        self.report.update(no_old_task_resume=True,fault_assertions_passed=True,final_injector=self.injector.copy())
        # Parent Mission performs its stationary collision checkpoint, then native Land.
        self.phase='ego'
