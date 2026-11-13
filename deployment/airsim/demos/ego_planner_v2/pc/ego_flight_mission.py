"""Validated basic flight plus one short real EGO/AS2 trajectory."""
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from quadrotor_msgs.msg import GoalSet
from traj_utils.msg import PolyTraj
from uav_ego_planner_adapter.conversion import Polynomial, peak, nanoseconds
from flight_mission import Mission
from uav_planner_interfaces.msg import TimedTrajectorySegment
from ego_reference_check import compare_references, segment_rows


class EgoMission(Mission):
    def __init__(self, require_peer=True):
        super().__init__(require_peer=require_peer)
        baseline = json.loads(Path('/baseline/result.json').read_text())
        if not baseline['passed'] or not baseline['final_disarmed'] or baseline['recovery']:
            raise RuntimeError('successful native basic flight is required')
        self.report['mode'] = 'ego-single-short'
        self.report['gates'].update(goal_distance_m=1., goal_error_m=.25, goal_timeout_s=60,
                                    reference_max_velocity=.2415, reference_max_acceleration=.3675,
                                    preflight_cloud_corridor_radius_m=.6)
        (self.out/'criteria.json').write_text(json.dumps(self.report['gates'],indent=2))
        self.ego_state = {}
        self.polys = []
        self.poly_records = []
        self.ego_refs = []
        self.segments = []
        self.peak_v = self.peak_a = 0.
        self.ego_start_counts = {}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(GoalSet,'/ego/goal',qos)
        self.cancel_client = self.create_client(Trigger,'/ego/cancel')
        self.keep.append(self.create_subscription(String,'/ego/status',self.ego_status,10))
        self.keep.append(self.create_subscription(PolyTraj,'/drone0/ego_planner/planning/trajectory',self.poly,10))
        self.keep.append(self.create_subscription(TimedTrajectorySegment,'/planner/trajectory',self.segments.append,10))

    def run(self):
        self.wait(lambda:self.ego_state.get('state')=='WAITING',30)
        super().run()

    def ego_status(self, msg):
        self.ego_state = json.loads(msg.data)
        self.event(ego_state=self.ego_state)
        if self.phase == 'ego' and self.ego_state.get('state') == 'FAILED':
            self.error = 'EGO execution failed: ' + self.ego_state.get('reason','')

    def poly(self, msg):
        try:
            source = Polynomial(msg,0,.2415,.3675)
            self.polys.append(source)
            self.peak_v = max(self.peak_v,max(peak(c,d,1) for c,d in zip(source.coefficients,source.durations)))
            self.peak_a = max(self.peak_a,max(peak(c,d,2) for c,d in zip(source.coefficients,source.durations)))
            self.poly_records.append({'start_ns':source.start_ns,'traj_id':msg.traj_id,
                'duration':[float(v) for v in msg.duration],
                **{k:[float(v) for v in getattr(msg,k)] for k in ('coef_x','coef_y','coef_z')}})
        except Exception as exc:
            self.error = 'invalid native polynomial: ' + repr(exc)

    def observe(self, key, msg):
        super().observe(key,msg)
        if self.phase == 'ego' and key == 'reference_trajectory' and msg.setpoints:
            p = msg.setpoints[0]
            velocity = [p.twist.x,p.twist.y,p.twist.z]
            acceleration = [p.acceleration.x,p.acceleration.y,p.acceleration.z]
            if math.sqrt(sum(v*v for v in velocity))>.2415+1e-8 or math.sqrt(sum(a*a for a in acceleration))>.3675+1e-8:
                self.error = 'AS2 motion reference exceeds EGO constraints'
            self.ego_refs.append({'stamp_ns':nanoseconds(msg.header.stamp),
                'position':[p.position.x,p.position.y,p.position.z], 'velocity':velocity,'acceleration':acceleration,
                'actual':self.position()})

    def after_hover(self):
        self.wait(lambda:self.ego_state.get('state')=='WAITING' and self.goal_pub.get_subscription_count()>0,15)
        position = self.position()
        points = list(point_cloud2.read_points(self.latest['cloud'],field_names=('x','y','z'),skip_nans=True))
        candidates = []
        for dx,dy in ((1.,0.),(0.,1.),(-1.,0.),(0.,-1.)):
            clearance = math.inf
            for q in points:
                along = max(0.,min(1.,(float(q[0])-position[0])*dx+(float(q[1])-position[1])*dy))
                closest = [position[0]+along*dx,position[1]+along*dy,position[2]]
                clearance = min(clearance,math.dist([float(v) for v in q],closest))
            candidates.append({'direction':[dx,dy], 'cloud_clearance_m':clearance})
        chosen = next((c for c in candidates if c['cloud_clearance_m']>=.6),None)
        self.report['goal_candidates'] = candidates
        if chosen is None:
            raise RuntimeError('no clear one-metre candidate in observed cloud')
        self.target = [position[0]+chosen['direction'][0],position[1]+chosen['direction'][1],position[2]]
        self.report['goal_enu'] = self.target
        self.report['ego_start_position'] = position
        self.ego_start_counts = dict(self.counts)
        self.phase = 'ego'
        self.goal_pub.publish(GoalSet(drone_id=0,goal=[float(x) for x in self.target]))
        self.wait(lambda:self.ego_state.get('state')=='SUCCEEDED',60)
        if not self.ego_state.get('stop_confirmed'):
            raise RuntimeError('EGO stop not confirmed')
        self.report['goal_error_m'] = math.dist(self.position(),self.target)
        self.report['ego_state'] = self.ego_state
        self.report['native_polynomial_peaks'] = [self.peak_v,self.peak_a]
        self.report['ego_reference_count'] = len(self.ego_refs)
        self.report['ego_downstream_counts'] = {k:self.counts.get(k,0)-self.ego_start_counts.get(k,0)
                                              for k in ('reference_trajectory','actuator','actuator_twist','px4_setpoint')}
        if self.report['goal_error_m']>.25 or len(self.polys)<1 or len(self.ego_refs)<20:
            raise RuntimeError('insufficient EGO execution or goal accuracy')
        if self.report['ego_downstream_counts']['px4_setpoint']<20:
            raise RuntimeError('missing EGO downstream consumption')
        error, holds = compare_references(self.polys,
            [dict(stamp=r['stamp_ns'],p=r['position'],v=r['velocity'],a=r['acceleration']) for r in self.ego_refs],
            self.segments,'drone0',[0,0,0],self.target)
        self.report['max_polynomial_reference_error'] = error
        self.report['explicit_terminal_hold_references'] = holds
        if error>1e-4:
            raise RuntimeError('AS2 references differ from native polynomial PVA')

    def recover(self):
        if self.phase == 'ego':
            self.recovering = True
            try:
                self.wait(self.cancel_client.service_is_ready,5)
                future=self.cancel_client.call_async(Trigger.Request())
                self.wait(future.done,5)
                self.wait(lambda:self.ego_state.get('stop_confirmed'),10)
                self.report['ego_stop_confirmed_after_failure']=True
            except Exception as exc:
                # A failed cancellation must not skip the independent Land attempt.
                self.report['recovery'].append('EGO cancellation failed: '+repr(exc))
        super().recover()


if __name__ == '__main__':
    rclpy.init()
    mission = EgoMission()
    try:
        mission.run()
    except BaseException as exc:
        mission.report['error']=repr(exc)
        try: mission.recover()
        except BaseException as recovery: mission.report['recovery_error']=repr(recovery)
    finally:
        mission.report.update(ego_state=mission.ego_state, counts=mission.counts,
            final_phase=mission.phase, final_disarmed=bool(mission.latest.get('mode') and not mission.latest['mode'].flag_armed),
            drone1_disarmed=bool(mission.latest.get('drone1_mode') and not mission.latest['drone1_mode'].flag_armed))
        mission.report['passed']=mission.report['passed'] and mission.report['final_disarmed'] and mission.report['drone1_disarmed']
        (mission.out/'result.json').write_text(json.dumps(mission.report,indent=2,default=float))
        (mission.out/'polynomials.json').write_text(json.dumps(mission.poly_records,default=float))
        (mission.out/'references.json').write_text(json.dumps(mission.ego_refs,default=float))
        print(json.dumps(mission.report,indent=2,default=float),flush=True)
        mission.log.close();mission.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if mission.report['passed'] else 1)
