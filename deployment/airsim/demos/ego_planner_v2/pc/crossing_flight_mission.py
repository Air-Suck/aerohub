"""Two simultaneous final goals, native EGO swarm avoidance, AS2 execution.

No simulator motion calls, waypoint schedules or trajectory retiming.
"""
import json, math, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from as2_msgs.srv import SetControlMode
from as2_msgs.msg import ControlMode
from px4_msgs.msg import OffboardControlMode
from traj_utils.msg import MINCOTraj
from quadrotor_msgs.msg import GoalSet
from dual_flight_mission import Fleet


class Crossing(Fleet):
    def __init__(self):
        self.minimum_initial_separation=3.5
        self.check_independent_route_tube=False
        super().__init__('dual')
        self.report['stage']='dual-crossing'
        self.report['gates'].update(goal_distance_m='equal-length crossing diagonals',
            goal_timeout_wall_s=150,max_horizontal_excursion_m=9.,route_tube_radius_m=None,
            straight_nominal_min_separation_max_m=.2,initial_height_difference_max_m=.2,
            swarm_clearance_per_vehicle_m=1.,minimum_lateral_detour_m=.5,
            native_virtual_height_bounds_m=[1.2,2.6],actual_relative_height_upper_m=3.2,
            measured_peer_body_mask_radius_m=.75,
            settling_stable_seconds=2.,settling_speed_m_s=.15,settling_height_error_m=.15,
            observed_route_clearance_m=.6,minimum_simultaneous_motion_s=2.)
        (self.out/'criteria.json').write_text(json.dumps(self.report['gates'],indent=2))
        for n in self.nodes:n.report['gates']['max_horizontal_excursion_m']=9.
        self.swarm=[]
        self.swarm_sub=self.nodes[0].create_subscription(MINCOTraj,'/ego_swarm/trajectory',self.minco,qos_profile_sensor_data)
        self.px4_modes={}
        self.mode_subs=[self.nodes[0].create_subscription(OffboardControlMode,
            f'/drone{i}/fmu/in/offboard_control_mode',lambda m,k=i:self.px4_modes.update({k:m}),qos_profile_sensor_data)
            for i in (0,1)]

    def minco(self,m):
        row=dict(sim=self.nodes[0].now_s(),drone_id=m.drone_id,traj_id=m.traj_id,
            start=m.start_time.sec+m.start_time.nanosec/1e9,clearance=m.des_clearance,
            duration=list(m.duration),start_p=list(m.start_p),end_p=list(m.end_p))
        self.swarm.append(row)
        self.nodes[0].event(swarm_trajectory=row)

    def prepare(self):
        super().prepare()
        node=self.nodes[0]
        def endpoints():
            return {kind:[dict(namespace=e.node_namespace,name=e.node_name,type=e.topic_type)
                for e in getattr(node,method)('/ego_swarm/trajectory')]
                for kind,method in [('publishers','get_publishers_info_by_topic'),('subscribers','get_subscriptions_info_by_topic')]}
        self.wait(lambda:len([e for e in endpoints()['subscribers'] if e['name']=='ego_planner'])==2,15)
        graph=endpoints()
        for kind in ('publishers','subscribers'):
            assert {e['namespace'] for e in graph[kind] if e['name']=='ego_planner'}=={'/drone0','/drone1'},graph
        parameters={}
        for i in (0,1):
            client=node.create_client(GetParameters,f'/drone{i}/ego_planner/get_parameters')
            self.wait(client.service_is_ready,10)
            future=client.call_async(GetParameters.Request(names=['optimization.swarm_clearance','manager.drone_id','use_sim_time',
                'grid_map.virtual_ground','grid_map.virtual_ceil']))
            self.wait(future.done,10)
            values=future.result().values
            parameters[str(i)]=[values[0].double_value,values[1].integer_value,values[2].bool_value,
                values[3].double_value,values[4].double_value]
            assert parameters[str(i)]==[1.,i,True,1.2,2.6],parameters
        self.report['swarm_graph']=graph
        self.report['native_parameters']=parameters
        (self.out/'ground-communication.json').write_text(json.dumps(dict(graph=graph,parameters=parameters),indent=2))
        self.wait(lambda:all(self.ego_state.get('counts',{}).get('peer_mask_frames_drone'+str(i),0)>=10 for i in (0,1)),15)
        # Establish a stationary reference while disarmed; a fresh controller
        # can still be UNSET and has no output mode to restore after Offboard.
        for n in self.nodes:
            client=n.create_client(SetControlMode,f'/drone{n.drone_id}/controller/set_control_mode')
            self.wait(client.service_is_ready,5)
            future=client.call_async(SetControlMode.Request(control_mode=ControlMode(control_mode=ControlMode.HOVER)))
            self.wait(future.done,5)
            assert future.result().success,'ground controller hover initialization failed'
            self.wait(lambda:n.latest['controller'].output_control_mode.control_mode==ControlMode.SPEED,5)
            n.report['ground_controller_mode_initialized']=True

    def arm(self):
        super().arm()
        # Fixed Pixhawk ownSetOffboardControl resets its internal mask to body
        # rates even if the controller negotiated SPEED before the handshake.
        # Reapply the actual negotiated mode through the existing AS2 service.
        for n in self.nodes:
            mode=n.latest['controller'].output_control_mode
            assert mode.control_mode==ControlMode.SPEED,'unexpected takeoff controller output mode'
            before=n.counts.get('px4_setpoint',0)
            client=n.create_client(SetControlMode,f'/drone{n.drone_id}/set_platform_control_mode')
            self.wait(client.service_is_ready,5)
            future=client.call_async(SetControlMode.Request(control_mode=mode))
            self.wait(future.done,5)
            assert future.result().success,'platform rejected negotiated mode restoration'
            def restored():
                mask=self.px4_modes.get(n.drone_id)
                return mask and mask.velocity and not mask.body_rate and n.counts.get('px4_setpoint',0)>=before+20
            self.wait(restored,5)
            n.report['post_offboard_mode_restored']=dict(control_mode=mode.control_mode,yaw_mode=mode.yaw_mode,
                px4_setpoints=n.counts.get('px4_setpoint',0)-before,velocity_mask=True,body_rate_mask=False)

    def hover(self):
        for n in self.nodes:n.phase='settling'
        stable=[None]
        began=time.monotonic()
        def settled():
            ok=all(n.speed()<=.15 and abs(n.position()[2]-n.origin[2]-2.)<=.15 for n in self.nodes)
            if not ok:stable[0]=None;return False
            if stable[0] is None:stable[0]=self.nodes[0].now_s()
            return self.nodes[0].now_s()-stable[0]>=2.
        self.wait(settled,20)
        self.report['settling_wall_seconds']=time.monotonic()-began
        # Keep the original five-second scored hover and its original limits.
        super().hover()

    def ego(self):
        starts=np.array([n.world(n.position()) for n in self.nodes])
        assert abs(starts[0,2]-starts[1,2])<=.2,'initial height mismatch'
        baseline=starts[0,:2]-starts[1,:2]
        assert 3.5<=np.linalg.norm(baseline)<=4.5,'crossing requires spawn separation near 4 m'
        shift=4.*np.array([-baseline[1],baseline[0]])/np.linalg.norm(baseline)
        goals=starts[::-1].copy();goals[:,:2]+=shift;goals[:,2]=starts[:,2].mean()
        selected=[]
        for n,start,g in zip(self.nodes,starts,goals):
            if len(n.clouds)<10:raise RuntimeError('insufficient actual cloud frames')
            points=np.concatenate(n.clouds);np.save(n.out/'preflight-cloud.npy',points)
            world_points=points.copy();world_points[:,0]-=4*n.drone_id
            direction=g-start
            along=np.clip((world_points-start)@direction/np.dot(direction,direction),0,1)
            clearance=float(np.linalg.norm(world_points-(start+along[:,None]*direction),axis=1).min())
            if clearance<.6:raise RuntimeError(f'Drone{n.drone_id} crossing corridor contains static obstacle: {clearance:.3f} m')
            local=g.copy();local[0]+=4*n.drone_id
            selected.append(dict(local=local.tolist(),world=g.tolist(),clearance=clearance))
        nominal=float(np.linalg.norm((starts[0]+goals[0])/2-(starts[1]+goals[1])/2))
        assert nominal<=.2,'routes do not meet at equal progress'
        self.report['routes']=dict(starts_world_enu=starts.tolist(),goals=selected,
            minimum_route_separation_m=nominal,straight_synchronous_min_separation_m=nominal,
            kind='same-height equal-distance X crossing; one final goal each')
        (self.out/'route.json').write_text(json.dumps(self.report['routes'],indent=2))
        for n,g in zip(self.nodes,selected):
            n.report['ego_collision_before']=n.collision_checkpoint('before_ego')
            n.target=g['local'];n.start_counts=dict(n.counts);n.phase='ego'
            n.report.update(goal_enu=n.target,goal_world_enu=g['world'],ego_start_position=n.position())
        self.report['goal_publish_times']=[]
        for n,g in zip(self.nodes,selected):
            self.report['goal_publish_times'].append(self.nodes[0].now_s())
            self.goal_pub.publish(GoalSet(drone_id=n.drone_id,goal=g['world']))
        self.wait(lambda:self.ego_state.get('state')=='SUCCEEDED',150)
        if not self.ego_state.get('stop_confirmed') or sorted(self.ego_state.get('completed',[]))!=['drone0','drone1']:
            raise RuntimeError('both execution results and stop confirmation required')
        for n in self.nodes:
            n.report['ego_collision_after']=n.collision_checkpoint('after_ego')
            if n.report['ego_collision_after']!=n.report['ego_collision_before']:
                raise RuntimeError(n.vehicle_name+' new EGO collision timestamp')
            n.report['goal_error_m']=math.dist(n.position(),n.target)
            n.report['ego_downstream_counts']={k:n.counts.get(k,0)-n.start_counts.get(k,0)
                for k in ('reference_trajectory','actuator','actuator_twist','px4_setpoint')}
            if n.report['goal_error_m']>.25 or len(n.references)<20 or not n.polys:
                raise RuntimeError('goal accuracy or native/reference output missing')
            if min(n.report['ego_downstream_counts'][k] for k in ('px4_setpoint','actuator'))<20:
                raise RuntimeError('missing downstream control')
        self.report['simultaneous_motion_sim_s']=sum(min(.05,max(0.,b-a)) for a,b in zip(self.moving_samples,self.moving_samples[1:]))
        assert self.report['simultaneous_motion_sim_s']>=2.,'no concurrent flight evidence'
        assert all(sum(r['drone_id']==i for r in self.swarm)>=2 for i in (0,1)),'missing bidirectional broadcast'

    def finish(self):
        try:
            (self.out/'swarm.json').write_text(json.dumps(self.swarm,indent=2,default=float))
        finally:
            super().finish()


if __name__=='__main__':
    rclpy.init();mission=Crossing()
    try:mission.run()
    except BaseException as exc:
        mission.report.update(passed=False,error=repr(exc))
        try:mission.recover()
        except BaseException as recovery:mission.report['recovery_error']=repr(recovery)
    finally:mission.finish();rclpy.shutdown()
    raise SystemExit(0 if mission.report['passed'] else 1)
