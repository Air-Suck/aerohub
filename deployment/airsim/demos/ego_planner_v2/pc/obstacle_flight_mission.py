"""One final goal behind a real, surveyed obstacle; no intermediate waypoints."""
import json,math,time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from quadrotor_msgs.msg import GoalSet
from ego_flight_mission import EgoMission


def xyz(message):
    data=point_cloud2.read_points(message,field_names=('x','y','z'),skip_nans=True)
    if isinstance(data,np.ndarray) and data.dtype.names:
        return np.column_stack([data[k] for k in ('x','y','z')]).astype(float)
    return np.asarray([[float(v) for v in p] for p in data],dtype=float)


class ObstacleMission(EgoMission):
    def __init__(self):
        super().__init__()
        self.report['mode']='ego-single-real-obstacle'
        self.report['gates'].update(goal_distance_m='selected behind surveyed obstacle',goal_timeout_s=150,
            max_horizontal_excursion_m=12.,minimum_straight_blocking_points=5,straight_blocking_radius_m=.25,
            minimum_observed_point_clearance_m=.35,minimum_crossing_lateral_detour_m=.6,
            goal_clearance_m=.6,obstacle_search_box_xy=[[5.4,4.6],[6.9,6.3]],
            minimum_ego_map_messages=10,maximum_pva_error=1e-4)
        (self.out/'criteria.json').write_text(json.dumps(self.report['gates'],indent=2))
        self.cloud_cache=[];self.track=[];self.live_clearance=math.inf
        self.occupancy_counts={'map':0,'inflated':0};self.map_snapshots={}
        for key,suffix in [('map','occupancy'),('inflated','occupancy_inflate')]:
            self.keep.append(self.create_subscription(PointCloud2,'/drone0/ego_planner/grid_map/'+suffix,
                lambda m,k=key:self.map_input(k,m),qos_profile_sensor_data))

    def map_input(self,key,msg):
        if msg.width*msg.height:
            self.occupancy_counts[key]+=1
            self.map_snapshots[key]=msg

    def observe(self,key,msg):
        super().observe(key,msg)
        if key=='cloud' and self.phase=='holding':
            self.cloud_cache.append(xyz(msg));self.cloud_cache=self.cloud_cache[-20:]
        if key=='odom' and self.phase=='ego':
            self.track.append([self.now_s(),*self.position()])
        if key=='cloud' and self.phase=='ego':
            points=xyz(msg)
            if len(points):
                clearance=float(np.linalg.norm(points-np.asarray(self.position()),axis=1).min())
                self.live_clearance=min(self.live_clearance,clearance)
                if clearance<.35:self.error='observed obstacle clearance below 0.35 m'

    def after_hover(self):
        self.wait(lambda:self.ego_state.get('state')=='WAITING' and len(self.cloud_cache)>=10 and
                  self.goal_pub.get_subscription_count()>0 and all(n>=10 for n in self.occupancy_counts.values()),20)
        position=np.asarray(self.position());points=np.concatenate(self.cloud_cache)
        _,ids=np.unique(np.round(points/.08).astype(int),axis=0,return_index=True);points=points[ids]
        region=points[(points[:,0]>=5.4)&(points[:,0]<=6.9)&(points[:,1]>=4.6)&(points[:,1]<=6.3)&
                      (abs(points[:,2]-position[2])<=.4)]
        if len(region)<10:raise RuntimeError('surveyed obstacle not observed at flight height')
        center=(region[:,:2].min(axis=0)+region[:,:2].max(axis=0))/2
        direction=center-position[:2];distance=float(np.linalg.norm(direction));direction/=distance
        self.target=[*(center+2.*direction),float(position[2])]
        vector=np.asarray(self.target)-position;length=float(np.linalg.norm(vector));unit=vector/length
        along=np.clip((points-position)@unit,0,length)
        straight_dist=np.linalg.norm(points-(position+along[:,None]*unit),axis=1)
        blocked=points[(straight_dist<=.25)&(along>1)&(along<length-1)]
        goal_clearance=float(np.linalg.norm(points-np.asarray(self.target),axis=1).min())
        self.report.update(ego_start_position=position.tolist(),goal_enu=self.target,
            obstacle_center_xy=center.tolist(),obstacle_distance_m=distance,
            straight_line_distance_m=length,straight_blocking_points=len(blocked),
            straight_min_point_distance_m=float(straight_dist.min()),goal_cloud_clearance_m=goal_clearance,
            obstacle_points_at_flight_height=len(region))
        np.savez_compressed(self.out/'preflight-cloud.npz',points=points,obstacle=region,blocked=blocked,
                            start=position,goal=np.asarray(self.target))
        for key,msg in self.map_snapshots.items():np.save(self.out/(key+'-before-goal.npy'),xyz(msg))
        if len(blocked)<5 or goal_clearance<.6 or length>10 or any(abs(x)>10.5 for x in self.target[:2]):
            raise RuntimeError('blocked-route selection did not satisfy predeclared gates')
        self.report['selected_route_before_goal']=True
        (self.out/'route.json').write_text(json.dumps(self.report,indent=2,default=float))
        self.ego_start_counts=dict(self.counts);self.phase='ego'
        print(json.dumps({'phase':'ego','goal':self.target,'straight_blocking_points':len(blocked)},default=float),flush=True)
        self.goal_pub.publish(GoalSet(drone_id=0,goal=[float(x) for x in self.target]))
        self.wait(lambda:self.ego_state.get('state')=='SUCCEEDED',150)
        self.report['goal_error_m']=math.dist(self.position(),self.target)
        self.report['ego_state']=self.ego_state
        self.report['ego_downstream_counts']={k:self.counts.get(k,0)-self.ego_start_counts.get(k,0)
            for k in ('reference_trajectory','actuator','actuator_twist','px4_setpoint')}
        if not self.ego_state.get('stop_confirmed') or self.report['goal_error_m']>.25:
            raise RuntimeError('EGO completion/stop/goal accuracy failed')
        if len(self.ego_refs)<100 or self.report['ego_downstream_counts']['px4_setpoint']<100:
            raise RuntimeError('insufficient real downstream trajectory execution')
        self.report['minimum_live_point_clearance_m']=self.live_clearance
        actual=np.asarray(self.track)[:,1:]
        projection=(actual[:,:2]-position[:2])@direction
        perpendicular=np.array([-direction[1],direction[0]])
        lateral=abs((actual[:,:2]-position[:2])@perpendicular)
        crossing=lateral[abs(projection-distance)<.5]
        self.report['max_lateral_detour_m']=float(lateral.max())
        self.report['obstacle_crossing_samples']=len(crossing)
        self.report['minimum_crossing_lateral_detour_m']=float(crossing.min()) if len(crossing) else None
        if not len(crossing) or crossing.min()<.6 or self.live_clearance<.35:
            raise RuntimeError('actual path does not prove the required obstacle detour')

    def audit_after_landing(self):
        errors=[]
        for reference in self.ego_refs:
            best=math.inf
            for poly in reversed(self.polys):
                if poly.start_ns<=reference['stamp_ns']<=poly.end_ns:
                    p,v,a=poly.sample(reference['stamp_ns'])
                    best=min(best,max(math.dist(p,reference['position']),math.dist(v,reference['velocity']),math.dist(a,reference['acceleration'])))
                    if best<1e-5:break
            errors.append(best)
        self.report.update(native_polynomial_peaks=[self.peak_v,self.peak_a],ego_reference_count=len(self.ego_refs),
                           max_polynomial_reference_error=max(errors,default=math.inf),ego_map_messages=self.occupancy_counts)
        if not errors or max(errors)>1e-4:raise RuntimeError('offline native-to-AS2 PVA audit failed')


if __name__=='__main__':
    rclpy.init();mission=ObstacleMission()
    try:
        mission.run();mission.audit_after_landing()
    except BaseException as exc:
        mission.report['passed']=False;mission.report['error']=repr(exc)
        try:mission.recover()
        except BaseException as recovery:mission.report['recovery_error']=repr(recovery)
    finally:
        mission.report.update(ego_state=mission.ego_state,counts=mission.counts,final_phase=mission.phase,
            final_disarmed=bool(mission.latest.get('mode') and not mission.latest['mode'].flag_armed),
            drone1_disarmed=bool(mission.latest.get('drone1_mode') and not mission.latest['drone1_mode'].flag_armed),
            ego_map_messages=mission.occupancy_counts)
        mission.report['passed']=mission.report['passed'] and mission.report['final_disarmed'] and mission.report['drone1_disarmed']
        for name,value in [('result',mission.report),('polynomials',mission.poly_records),('references',mission.ego_refs),('actual-track',mission.track)]:
            (mission.out/(name+'.json')).write_text(json.dumps(value,indent=2,default=float))
        print(json.dumps(mission.report,indent=2,default=float),flush=True)
        mission.log.close();mission.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if mission.report['passed'] else 1)
