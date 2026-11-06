"""Three independent Swarm planners with real LiDAR; reuse the verified AS2 flight harness."""
import json,os,signal,sys
from pathlib import Path
import yaml
import rclpy
from std_msgs.msg import String
sys.path.insert(0,str(Path(__file__).parent/'common'))
from flight_mission import Mission as BaseMission


class Mission(BaseMission):
    def __init__(self):
        super().__init__()
        self.subscriptions.append(self.node.create_subscription(String,'/planner/swarm_input_state',self.swarm_state,10))

    def run(self):
        if os.environ.get('CATORA_PREFLIGHT_ONLY')!='1':
            return super().run()
        import time
        if os.environ.get('CATORA_METRICS_SCRIPT'):
            self.start('metrics',['python3',os.environ['CATORA_METRICS_SCRIPT']])
        self.start('as2',['python3',os.environ['AS2_LAUNCHER']])
        self.start('input-adapter',[])
        self.phase='waiting_for_localization'
        self.wait(lambda:len(self.local_poses)==len(self.platforms)==len(self.modes)==len(self.vehicles),90)
        self.screenshot('ground')
        until=time.monotonic()+15;self.wait(lambda:time.monotonic()>=until)
        self.align_origins();self.measure_links()
        initial={v:dict(s['counts']) for v,s in self.input_state.items()}
        self.phase='real_cloud_ground_probe'
        begin=time.monotonic();self.wait(lambda:time.monotonic()-begin>=20,25)
        final={v:dict(s['counts']) for v,s in self.input_state.items()}
        delta={v:{k:final[v][k]-initial[v][k] for k in initial[v]} for v in self.vehicles}
        publishers=self.node.get_publishers_info_by_topic('/swarm_formation/broadcast')
        subscribers=self.node.get_subscriptions_info_by_topic('/swarm_formation/broadcast')
        assert len(publishers)==len(subscribers)==len(self.vehicles),(publishers,subscribers)
        assert all(s['healthy'] for s in self.input_state.values()),self.input_state
        assert all(c['cloud']>=60 and c['odom']>=300 for c in delta.values()),delta
        self.snapshot_processes()
        result={'passed':True,'armed':any(m.flag_armed for m in self.modes.values()),
            'probe_wall_seconds':time.monotonic()-begin,'real_navigation_input_counts':delta,
            'input_state':self.input_state,'native_broadcast_publishers':[p.node_name for p in publishers],
            'native_broadcast_subscribers':[p.node_name for p in subscribers],'communication_rtt':self.link_metrics,
            'scope':'real LiDAR registration, native map health and DDS endpoints; no motion request',
            'execution_arch':os.uname().machine}
        assert not result['armed']
        (self.output/'ground-probe.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)

    def pose_topic(self,vehicle):return f'/{vehicle}/swarm/local_pose'
    def ping_topic(self,vehicle):return f'/{vehicle}/swarm/ping'

    def swarm_state(self,msg):
        state=json.loads(msg.data);self.input_state[state['vehicle']]=state
        self.samples.write(json.dumps({'unix_time':__import__('time').time(),'swarm_input':state})+'\n')

    def start(self,name,command):
        if name in ('cat-ora','proxy'):return
        if name=='input-adapter':
            self.start_navigation()
            command=['ros2','run','uav_swarm_formation_adapter','swarm_input_adapter_node','--ros-args',
                     '-p','vehicle_id:=drone0','-p','use_sim_time:=true']
        super().start(name,command)

    def start_navigation(self):
        super().start('navigation',['ros2','run','as2_navigation_inputs','navigation_inputs_node','--ros-args',
            '-r','__ns:=/drone0','-r','/tf:=/drone0/tf','-r','/tf_static:=/drone0/tf_static',
            '-p','use_sim_time:=true','-p','target_frame:=drone0/odom','-p','body_frame:=drone0/base_link'])

    def measure_links(self):
        self.phase="waiting_for_real_maps"
        self.wait(lambda:len(self.input_state)==len(self.vehicles) and all(s.get("healthy",False) for s in self.input_state.values()),30)
        super().measure_links()

    def align_origins(self):
        super().align_origins()
        fleet={'map_frame':'swarm_map','input_timeout':1.0,'vehicles':{}}
        for v,d in self.deployment.items():
            n,e,down=d['px4_origin_in_fleet_ned']
            fleet['vehicles'][v]={'source_frame':v+'/odom','body_frame':v+'/base_link','origin_enu':[float(e),float(n),float(-down)],'origin_yaw':0.0}
        path=self.output/'nav-fleet.yaml';path.write_text(yaml.safe_dump(fleet))
        super().start('native-planner',['ros2','launch','ego_planner','seven_planners.launch.py','input_mode:=navigation',
            'vehicle_count:=3','local_vehicle:=drone0','start_proxy:=false','fleet_config:='+str(path),
            'parameters_file:='+str(self.root/'planner.yaml')])
        super().start('swarm-proxy',['ros2','run','ego_planner','ros2_proxy.py','--ros-args','-p','use_sim_time:=true',
            '-p','vehicle_count:=3','-p','publish_odometry:=false','-p','relay_broadcast:=false','-p','map_frame:=swarm_map'])


def main():
    rclpy.init();mission=Mission()
    def interrupted(*_):raise InterruptedError('mission interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    try:mission.run()
    except Exception as exc:
        (mission.output/'failure.json').write_text(json.dumps({'phase':mission.phase,'error':str(exc),
            'input_state':mission.input_state,'bridge_state':mission.bridge_state,
            'planner_status':None if mission.status is None else mission.status.overall_status},indent=2)+'\n')
        if any(m.flag_armed for m in mission.modes.values()):mission.stop_after_failure()
        raise
    finally:
        mission.close();rclpy.try_shutdown()


if __name__=='__main__':main()
