"""Two real PX4/AS2 vehicles; shared executor, no simulator control calls.

Stages: basic1 verifies Drone1 alone; dual runs concurrent native actions and
one independent EGO goal per vehicle. All positions below are ENU unless noted.
"""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from quadrotor_msgs.msg import GoalSet
from traj_utils.msg import PolyTraj
from uav_ego_planner_adapter.conversion import Polynomial, peak, nanoseconds
from flight_mission import Mission, Takeoff, Land, PlatformStatus


class Vehicle(Mission):
    def __init__(self, identity, fleet):
        self.fleet = fleet
        self.polys, self.poly_records, self.references = [], [], []
        self.clouds = []
        super().__init__(identity, '/evidence/drone'+str(identity), allow_peer_armed=fleet.stage=='dual' or identity==0)
        self.keep.append(self.create_subscription(PolyTraj,
            f'/drone{identity}/ego_planner/planning/trajectory', self.poly, 10))

    def world(self, local):
        return [local[0]-4*self.drone_id, local[1], local[2]]

    def wait(self, predicate, timeout=15):
        self.fleet.wait(predicate, timeout)

    def observe(self, key, msg):
        super().observe(key, msg)
        if key == 'cloud' and self.phase == 'holding':
            points = np.array([(float(p[0]),float(p[1]),float(p[2])) for p in
                point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)])
            if len(points):
                self.clouds.append(points)
                self.clouds = self.clouds[-20:]
        if self.phase=='ego' and key=='reference_trajectory' and msg.setpoints:
            s=msg.setpoints[0]
            p=[s.position.x,s.position.y,s.position.z]
            v=[s.twist.x,s.twist.y,s.twist.z]
            a=[s.acceleration.x,s.acceleration.y,s.acceleration.z]
            if msg.header.frame_id != f'drone{self.drone_id}/odom':
                self.error='wrong reference frame: '+msg.header.frame_id
            if math.dist(v,[0,0,0])>.2415+1e-8 or math.dist(a,[0,0,0])>.3675+1e-8:
                self.error='reference v/a limit exceeded'
            self.references.append(dict(stamp_ns=nanoseconds(msg.header.stamp),
                position=self.world(p),velocity=v,acceleration=a,actual=self.world(self.position())))
            if self.speed()>.5:
                self.error='actual EGO speed exceeds predeclared 0.5 m/s test envelope'

    def poly(self,msg):
        try:
            source=Polynomial(msg,self.drone_id,.2415,.3675)
            self.polys.append(source)
            self.poly_records.append(dict(start_ns=source.start_ns,traj_id=msg.traj_id,
                drone_id=msg.drone_id,duration=list(msg.duration),
                **{k:list(getattr(msg,k)) for k in ('coef_x','coef_y','coef_z')}))
            self.event(native_polynomial=self.poly_records[-1])
        except Exception as exc:
            self.error='invalid native polynomial: '+repr(exc)

    def finish_report(self):
        self.report.update(counts=self.counts,final_phase=self.phase,
            final_disarmed=bool(self.latest.get('mode') and not self.latest['mode'].flag_armed),
            drone_id=self.drone_id)
        for name,data in [('result',self.report),('polynomials',self.poly_records),('references',self.references)]:
            (self.out/(name+'.json')).write_text(json.dumps(data,indent=2,default=float))
        self.log.flush()


class Fleet:
    def __init__(self,stage,preferred_direction='east'):
        self.stage=stage
        self.preferred_direction=preferred_direction
        self.action_results=[]
        self.active=[1] if stage=='basic1' else [0,1]
        self.recovering=False
        self.ego_state={}
        self.out=Path('/evidence')
        self.report=dict(passed=False,stage=stage,active=self.active,recovery=[],preferred_direction=preferred_direction,
            collision_observation='stationary checkpoints; no collision RPC while moving or landing',
            topology='PC AirSim + fixed flight-board PX4; fixed onboard AS2/EGO; separate workstation mission client',
            gates=dict(takeoff_height_m=2.,hover_seconds=5.,hover_height_error_m=.3,
                hover_horizontal_drift_m=.5,hover_speed_m_s=.3,landing_height_error_m=.2,
                min_separation_m=2.,goal_distance_m=1.,goal_error_m=.25,goal_timeout_wall_s=60,
                reference_max_velocity=.2415,reference_max_acceleration=.3675,
                max_pva_error=1e-4,data_timeout_wall_s=1.,actual_ego_speed_envelope_m_s=.5,
                airborne_speed_envelope_m_s=2.,max_horizontal_excursion_m=3.,
                observed_route_clearance_m=.6,route_tube_radius_m=.5,
                minimum_simultaneous_motion_s=.5,scene_odom_alignment_error_m=.5))
        self.report['gates']['max_landing_physics_scene_z_gap_m']=.3
        (self.out/'criteria.json').write_text(json.dumps(self.report['gates'],indent=2))
        self.nodes=[Vehicle(i,self) for i in range(2)]
        self.executor=SingleThreadedExecutor()
        for node in self.nodes:self.executor.add_node(node)
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub=self.nodes[0].create_publisher(GoalSet,'/ego/goal',qos)
        self.cancel_client=self.nodes[0].create_client(Trigger,'/ego/cancel')
        self.status_sub=self.nodes[0].create_subscription(String,'/ego/status',self.status,10)
        self.min_separation=math.inf
        self.moving_samples=[]
        self.initialized=False

    def status(self,msg):
        self.ego_state=json.loads(msg.data)
        self.nodes[0].event(ego_state=self.ego_state)

    def wait(self,predicate,timeout=15):
        deadline=time.monotonic()+timeout
        while True:
            self.executor.spin_once(timeout_sec=.005)
            if not self.recovering:
                for n in self.nodes:n.health()
                if self.initialized:
                    distance=math.dist(*[n.world(n.position()) for n in self.nodes])
                    self.min_separation=min(self.min_separation,distance)
                    if distance<2.:raise RuntimeError('minimum fleet separation violated')
                    if self.stage=='dual' and all(n.phase=='ego' for n in self.nodes):
                        if self.ego_state.get('state')=='FAILED':
                            raise RuntimeError('EGO failed: '+self.ego_state.get('reason',''))
                        if all(n.speed()>.05 for n in self.nodes):
                            self.moving_samples.append(self.nodes[0].now_s())
                        for n in self.nodes if getattr(self,'check_independent_route_tube',True) else []:
                            start=np.array(n.report['ego_start_position']);end=np.array(n.target)
                            direction=end-start
                            along=np.clip(np.dot(np.array(n.position())-start,direction)/np.dot(direction,direction),0,1)
                            if np.linalg.norm(np.array(n.position())-(start+along*direction))>.5:
                                raise RuntimeError(n.vehicle_name+' left assigned independent route tube')
            if predicate():return
            if time.monotonic()>deadline:raise TimeoutError('fleet wait timeout; phases='+str([n.phase for n in self.nodes]))

    def prepare(self):
        keys=('platform','mode','drone1_mode','odom','cloud','controller','clock')
        self.wait(lambda:all(all(k in n.latest for k in keys) and n.counts.get('odom',0)>30
            and hasattr(n,'scene_pose') and n.takeoff.server_is_ready() and n.land.server_is_ready()
            for n in self.nodes),45)
        if any(n.latest['mode'].flag_armed for n in self.nodes):raise RuntimeError('must start disarmed')
        if any(n.latest['platform'].status.state not in (PlatformStatus.DISARMED,PlatformStatus.LANDED) for n in self.nodes):
            raise RuntimeError('AS2 platform must be DISARMED/LANDED before arming; recover its state first')
        scene_world=[]
        for n in self.nodes:
            n.origin=n.position()
            p=n.scene_pose['position']
            n.truth_origin=[p['x_val'],p['y_val'],p['z_val']]
            n.collision_stamp=n.rpc.call('simGetCollisionInfo',n.vehicle_name)['time_stamp']
            n.report.update(origin_enu=n.origin,truth_origin_ned=n.truth_origin,collision_stamp=n.collision_stamp)
            # AirSim vehicle pose is local NED; apply the documented spawn origin.
            scene_world.append([p['y_val']-4*n.drone_id,p['x_val'],-p['z_val']])
        odom_delta=np.array(self.nodes[0].world(self.nodes[0].position()))-self.nodes[1].world(self.nodes[1].position())
        scene_delta=np.array(scene_world[0])-scene_world[1]
        # Compare horizontal offsets, as estimator ground-height bias is separate.
        alignment=float(np.linalg.norm(odom_delta[:2]-scene_delta[:2]))
        self.report['initial_scene_odom_relative_xy_error_m']=alignment
        if alignment>.5:raise RuntimeError('fleet origin/scene relative XY alignment failed')
        self.initialized=True
        self.report['initial_positions_world_enu']=[n.world(n.position()) for n in self.nodes]
        if math.dist(*self.report['initial_positions_world_enu'])<getattr(self,'minimum_initial_separation',4.):
            raise RuntimeError('insufficient separation for independent 1 m routes')
        if self.stage=='dual':
            baseline=json.loads(Path('/baseline/result.json').read_text())
            if not baseline.get('passed') or baseline.get('recovery') or not baseline.get('both_disarmed'):
                raise RuntimeError('Drone1 native basic flight prerequisite missing')
            self.wait(lambda:self.ego_state.get('state')=='WAITING' and self.goal_pub.get_subscription_count()>0,30)

    def arm(self):
        for i in self.active:
            n=self.nodes[i];n.phase='arming';n.report['motion_requested']=True
            n.service('set_arming_state',True,400)
            self.wait(lambda:n.latest['platform'].armed and n.latest['mode'].flag_armed)
            n.service('set_offboard_mode',True,176)
            self.wait(lambda:n.latest['platform'].offboard and n.latest['mode'].flag_control_offboard_enabled)
            stable=[None]
            def fresh():
                ok=all(time.monotonic()-n.received.get(k,0)<.3 for k in ('odom','platform','controller','cloud','clock'))
                if not ok:stable[0]=None;return False
                if stable[0] is None:stable[0]=time.monotonic()
                return time.monotonic()-stable[0]>=.5
            self.wait(fresh,10)
            n.report['navigation_fresh_after_offboard']=True

    def actions(self,kind):
        field='takeoff_success' if kind=='takeoff' else 'land_success'
        goals=[]
        for i in self.active:
            n=self.nodes[i]
            n.phase='taking_off' if kind=='takeoff' else 'landing'
            client=n.takeoff if kind=='takeoff' else n.land
            goal=Takeoff.Goal(takeoff_height=2.,takeoff_speed=.4) if kind=='takeoff' else Land.Goal(land_speed=.3)
            goals.append((n,client.send_goal_async(goal)))
        self.wait(lambda:all(f.done() for _,f in goals),15)
        results=[]
        for n,f in goals:
            h=f.result()
            if not h.accepted:raise RuntimeError(n.vehicle_name+' '+kind+' rejected')
            n.handles.append(h);results.append((n,h.get_result_async()))
        self.action_results=results
        try:
            self.wait(lambda:all(f.done() for _,f in results),75)
        finally:
            for n,f in results:
                if f.done():
                    r=f.result()
                    n.report['actions'].append(dict(result=field,success=bool(getattr(r.result,field)),status=r.status))
        for n,f in results:
            r=f.result()
            if not getattr(r.result,field) or r.status!=4:raise RuntimeError(n.vehicle_name+' '+kind+' failed')

    def hover(self):
        self.wait(lambda:all(self.nodes[i].latest['platform'].status.state==PlatformStatus.FLYING for i in self.active))
        for i in self.active:self.nodes[i].phase='settling'
        start=self.nodes[0].now_s();self.wait(lambda:self.nodes[0].now_s()-start>=3,15)
        for i in self.active:self.nodes[i].phase='holding'
        start=self.nodes[0].now_s();self.wait(lambda:self.nodes[0].now_s()-start>=5,20)
        for i in self.active:
            n=self.nodes[i];samples=n.hold_samples
            if len(samples)<20:raise RuntimeError('missing hover samples')
            metrics=dict(samples=len(samples),max_height_error_m=max(abs(s['position'][2]-n.origin[2]-2) for s in samples),
                max_horizontal_drift_m=max(math.dist(s['position'][:2],n.origin[:2]) for s in samples),
                max_speed_m_s=max(s['speed'] for s in samples))
            n.report['hover']=metrics
            if metrics['max_height_error_m']>.3 or metrics['max_horizontal_drift_m']>.5 or metrics['max_speed_m_s']>.3:
                raise RuntimeError(n.vehicle_name+' hover failed')

    def ego(self):
        candidates=[]
        for n in self.nodes:
            if len(n.clouds)<10:raise RuntimeError('insufficient real cloud frames')
            points=np.concatenate(n.clouds)
            np.save(n.out/'preflight-cloud.npy',points)
            p=np.array(n.position());options=[]
            sign=1. if self.preferred_direction=='east' else -1.
            for dx,dy in [(sign,0.),(0.,1.),(-sign,0.),(0.,-1.)]:
                direction=np.array([dx,dy,0.])
                along=np.clip((points-p)@direction,0,1)
                clearance=float(np.linalg.norm(points-(p+along[:,None]*direction),axis=1).min())
                goal=p+direction
                world=n.world(goal)
                if clearance>=.6 and max(abs(world[0]),abs(world[1]))<10.5:
                    options.append(dict(local=goal.tolist(),world=world,clearance=clearance))
            if not options:raise RuntimeError(n.vehicle_name+' no clear route')
            candidates.append(options)
        starts=[n.world(n.position()) for n in self.nodes]
        selected=None
        for a in candidates[0]:
            for b in candidates[1]:
                s0=np.linspace(starts[0],a['world'],101);s1=np.linspace(starts[1],b['world'],101)
                separation=float(np.linalg.norm(s0[:,None,:]-s1[None,:,:],axis=2).min())
                if separation>=3.:
                    selected=[a,b];break
            if selected:break
        if not selected:raise RuntimeError('no disjoint route tubes with 2 m clearance')
        self.report['routes']=dict(starts_world_enu=starts,goals=selected,minimum_route_separation_m=separation)
        (self.out/'route.json').write_text(json.dumps(self.report['routes'],indent=2))
        for n,g in zip(self.nodes,selected):
            n.report['ego_collision_before']=n.collision_checkpoint('before_ego')
            n.target=g['local'];n.start_counts=dict(n.counts);n.phase='ego'
            n.report.update(goal_enu=n.target,goal_world_enu=g['world'],ego_start_position=n.position())
        for n,g in zip(self.nodes,selected):self.goal_pub.publish(GoalSet(drone_id=n.drone_id,goal=g['world']))
        self.wait(lambda:self.ego_state.get('state')=='SUCCEEDED',60)
        if not self.ego_state.get('stop_confirmed') or sorted(self.ego_state.get('completed',[]))!=['drone0','drone1']:
            raise RuntimeError('both execution results and stop confirmation required')
        for n in self.nodes:
            n.report['ego_collision_after']=n.collision_checkpoint('after_ego')
            if n.report['ego_collision_after'] != n.report['ego_collision_before']:
                raise RuntimeError(n.vehicle_name+' new EGO collision timestamp')
            n.report['goal_error_m']=math.dist(n.position(),n.target)
            n.report['ego_downstream_counts']={k:n.counts.get(k,0)-n.start_counts.get(k,0)
                for k in ('reference_trajectory','actuator','actuator_twist','px4_setpoint')}
            if n.report['goal_error_m']>.25 or len(n.references)<20 or not n.polys:
                raise RuntimeError('goal accuracy or native/reference output missing')
            if n.report['ego_downstream_counts']['px4_setpoint']<20 or n.report['ego_downstream_counts']['actuator']<20:
                raise RuntimeError('missing downstream control')
        duration=sum(min(.05,max(0.,b-a)) for a,b in zip(self.moving_samples,self.moving_samples[1:]))
        self.report['simultaneous_motion_sim_s']=duration
        if duration<.5:raise RuntimeError('no simultaneous two-vehicle motion evidence')

    def land(self):
        self.actions('land')
        self.wait(lambda:all(not self.nodes[i].latest['mode'].flag_armed and
            not self.nodes[i].latest['platform'].armed for i in self.active),20)
        for i in self.active:
            n=self.nodes[i];n.phase='landed'
            p=n.rpc.call('simGetVehiclePose',n.vehicle_name)['position']
            n.report['landing_height_error_m']=abs(p['z_val']-n.truth_origin[2])
            if n.report['landing_height_error_m']>.2:raise RuntimeError('landing height failed')
            if n.report.get('max_landing_physics_scene_z_gap_m',math.inf)>.3:
                raise RuntimeError('landing physics/scene divergence')
            if n.counts.get('px4_setpoint',0)<30 or n.counts.get('actuator_twist',0)+n.counts.get('actuator',0)<30:
                raise RuntimeError('missing basic control evidence')
            n.report['passed']=True

    def audit(self):
        # CPU-intensive curve comparison happens after both aircraft land.
        if self.stage!='dual':return
        common_end=max(p.end_ns for n in self.nodes for p in n.polys)
        for n in self.nodes:
            errors=[]
            for ref in n.references:
                best=math.inf
                for poly in reversed(n.polys):
                    if poly.start_ns<=ref['stamp_ns']<=poly.end_ns:
                        p,v,a=poly.sample(ref['stamp_ns'])
                        best=min(best,max(math.dist(p,ref['position']),math.dist(v,ref['velocity']),math.dist(a,ref['acceleration'])))
                        if best<1e-5:break
                    elif poly.end_ns<ref['stamp_ns']<=common_end and poly.final(n.world(n.target)):
                        # The fleet executor holds a finished, stationary native
                        # endpoint until the other vehicle's common end epoch.
                        p=poly.sample(poly.end_ns)[0]
                        best=min(best,max(math.dist(p,ref['position']),math.dist([0,0,0],ref['velocity']),math.dist([0,0,0],ref['acceleration'])))
                errors.append(best)
            n.report.update(max_polynomial_reference_error=max(errors,default=math.inf),
                native_polynomial_peaks=[max(peak(c,d,k) for s in n.polys for c,d in zip(s.coefficients,s.durations)) for k in (1,2)])
            if not errors or max(errors)>1e-4:raise RuntimeError(n.vehicle_name+' polynomial/reference mismatch')

    def recover(self):
        self.recovering=True
        for n in self.nodes:n.recovering=True
        for n in self.nodes:
            for handle in n.handles:
                if handle.status not in (4,5,6):
                    try:
                        future=handle.cancel_goal_async();self.wait(future.done,5)
                        self.report['recovery'].append(dict(drone=n.drone_id,action_cancel_return_code=future.result().return_code))
                    except Exception as exc:self.report['recovery'].append('action cancel error: '+repr(exc))
        if any(n.phase=='ego' for n in self.nodes):
            try:
                self.wait(self.cancel_client.service_is_ready,5)
                future=self.cancel_client.call_async(Trigger.Request());self.wait(future.done,5)
                self.wait(lambda:self.ego_state.get('stop_confirmed'),10)
                self.report['recovery'].append('EGO stop confirmed')
            except Exception as exc:self.report['recovery'].append('EGO stop error: '+repr(exc))
        # Try each vehicle independently, even if recovery of its peer fails.
        for i in self.active:
            n=self.nodes[i]
            try:n.recover()
            except Exception as exc:n.report['recovery_error']=repr(exc)
            self.report['recovery'].append(dict(drone=i,steps=n.report['recovery'],error=n.report.get('recovery_error')))

    def run(self):
        self.prepare();self.arm();self.actions('takeoff');self.hover()
        if self.stage=='dual':self.ego()
        self.land();self.audit();self.report['passed']=True

    def finish(self):
        self.report.update(ego_state=self.ego_state,minimum_odom_separation_m=self.min_separation,
            both_disarmed=all(n.latest.get('mode') and not n.latest['mode'].flag_armed for n in self.nodes))
        self.report['passed']=bool(self.report['passed'] and self.report['both_disarmed'])
        for n in self.nodes:n.finish_report()
        (self.out/'result.json').write_text(json.dumps(self.report,indent=2,default=float))
        print(json.dumps(self.report,indent=2,default=float),flush=True)
        self.executor.shutdown()
        for n in self.nodes:n.log.close();n.destroy_node()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--stage',choices=['basic1','dual'],required=True)
    parser.add_argument('--preferred-direction',choices=['east','west'],default='east')
    args=parser.parse_args();rclpy.init();mission=Fleet(args.stage,args.preferred_direction)
    try:mission.run()
    except BaseException as exc:
        mission.report['passed']=False;mission.report['error']=repr(exc)
        try:mission.recover()
        except BaseException as recovery:mission.report['recovery_error']=repr(recovery)
    finally:mission.finish();rclpy.shutdown()
    raise SystemExit(0 if mission.report['passed'] else 1)
