"""Config-sized mixed Swarm planners with one board UAV and a PC ground station."""
import json,os,signal,sys
from pathlib import Path
import yaml
import rclpy
from as2_msgs.action import Land
from as2_msgs.msg import ControlMode
from std_msgs.msg import String
from uav_planner_interfaces.msg import BehaviorResult
sys.path.insert(0,str(Path(__file__).parent/'common'))
from flight_mission import Mission as BaseMission

VEHICLE_PROCESSES=('as2','input-adapter','navigation','native-planner','swarm-proxy','cat-ora','proxy')


class Mission(BaseMission):
    def __init__(self):
        super().__init__()
        self.role=os.environ.get('AEROHUB_ROLE','')
        self.local_vehicle=os.environ['AEROHUB_SWARM_BOARD_VEHICLE']
        self.vehicle_count=int(os.environ['AEROHUB_SWARM_VEHICLE_COUNT'])
        self.coordination_scenario=os.environ['AEROHUB_SWARM_SCENARIO']
        self.fault_scenario=os.environ.get('AEROHUB_SWARM_FAULT','none')
        if self.role=='ground' and self.fault_scenario!='none':
            os.environ['CATORA_SCENARIO']=self.fault_scenario
        self.subscriptions.append(self.node.create_subscription(String,'/planner/swarm_input_state',self.swarm_state,10))

    def run(self):
        if self.role=='board_uav':
            self._run_board_uav();return
        if self.role=='ground' and os.environ.get('CATORA_PREFLIGHT_ONLY')=='1':
            self._run_ground_preflight();return
        if os.environ.get('CATORA_PREFLIGHT_ONLY')!='1':
            return super().run()
        self._run_combined_preflight()

    def run_stop_scenario(self,client,scenario):
        if not scenario.startswith('swarm_'):
            return super().run_stop_scenario(client,scenario)
        self.wait(lambda:all(count>=3 for count in self.reference_counts.values()),10)
        self.phase='fault_ready'
        (self.output/'fault-ready.json').write_text(json.dumps({
            'scenario':scenario,'task_id':self.task_id,'vehicles':self.vehicles})+'\n')
        self.wait(lambda:(self.output/'fault-injected.json').exists(),30)
        injection=json.loads((self.output/'fault-injected.json').read_text())
        self.phase='fault_detection'
        self.wait(lambda:self.status.overall_status==BehaviorResult.FAILED,20)
        self.trajectory_mode_claimed=False
        self.wait_as2_stop()
        self.wait(lambda:all(
            self.controllers[vehicle].input_control_mode.control_mode
            in (ControlMode.HOVER,ControlMode.POSITION) for vehicle in self.vehicles),5)
        self.screenshot('fault-hover')
        self.phase='landing'
        for vehicle in self.vehicles:
            self.action(vehicle,Land,'LandBehavior',Land.Goal(land_speed=.5),'land_success')
            self.wait(lambda v=vehicle:not self.modes[v].flag_armed)
        self.screenshot('landed')
        result={'scenario':scenario,'planner_status':'FAILED',
            'failure_reason':self.status.failure_reason,'fault':injection,
            'as2_hover_confirmed':True,'stop_state':self.bridge_state,
            'landed_and_disarmed':True,'execution_arch':os.uname().machine,
            'as2_trajectory_reference_counts':self.reference_counts}
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)

    def _localized(self):
        return len(self.local_poses)==len(self.platforms)==len(self.modes)==len(self.vehicles)

    def _run_board_uav(self):
        if os.environ.get('CATORA_METRICS_SCRIPT'):
            self.start('metrics',['python3',os.environ['CATORA_METRICS_SCRIPT']])
        self.start('as2',['python3',os.environ['AS2_LAUNCHER']])
        self.start('input-adapter',[])
        self.phase='waiting_for_localization'
        self.wait(self._localized,90)
        self.align_origins()
        self.phase='board_uav_ready'
        (self.output/'board-uav.json').write_text(json.dumps({'vehicle':self.local_vehicle,'center':self.coordination_scenario,'ready':True})+'\n')
        while True:
            self.executor.spin_once(timeout_sec=.2)
            for name,process,_ in self.children:
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited: {process.returncode}')

    def _run_ground_preflight(self):
        import time
        self.phase='waiting_for_localization'
        self.wait(self._localized,120)
        self.screenshot('ground')
        until=time.monotonic()+15;self.wait(lambda:time.monotonic()>=until)
        self.measure_links()
        self.phase='waiting_for_native_planners'
        topic='/swarm_formation/broadcast'
        self.wait(lambda:len(self.node.get_publishers_info_by_topic(topic))==len(self.vehicles),60)
        self._write_ground_probe(time)

    def _run_combined_preflight(self):
        import time
        if os.environ.get('CATORA_METRICS_SCRIPT'):
            self.start('metrics',['python3',os.environ['CATORA_METRICS_SCRIPT']])
        self.start('as2',['python3',os.environ['AS2_LAUNCHER']])
        self.start('input-adapter',[])
        self.phase='waiting_for_localization'
        self.wait(self._localized,90)
        self.screenshot('ground')
        until=time.monotonic()+15;self.wait(lambda:time.monotonic()>=until)
        self.align_origins();self.measure_links()
        self._write_ground_probe(time)

    def _write_ground_probe(self,time):
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
            'placement':self.role or 'combined','execution_arch':os.uname().machine}
        assert not result['armed']
        (self.output/'ground-probe.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)

    def pose_topic(self,vehicle):return f'/{vehicle}/swarm/local_pose'
    def ping_topic(self,vehicle):return f'/{vehicle}/swarm/ping'

    def swarm_state(self,msg):
        state=json.loads(msg.data);self.input_state[state['vehicle']]=state
        self.samples.write(json.dumps({'unix_time':__import__('time').time(),'swarm_input':state})+'\n')

    def start(self,name,command):
        if self.role=='ground' and name in VEHICLE_PROCESSES:return
        if name in ('cat-ora','proxy'):return
        if name=='input-adapter':
            self.start_navigation()
            command=['ros2','run','uav_swarm_formation_adapter','swarm_input_adapter_node','--ros-args',
                     '-p','vehicle_id:='+self.local_vehicle,'-p','use_sim_time:=true']
        super().start(name,command)

    def start_navigation(self):
        super().start('navigation',['ros2','run','as2_navigation_inputs','navigation_inputs_node','--ros-args',
            '-r','__ns:=/'+self.local_vehicle,'-r','/tf:=/'+self.local_vehicle+'/tf','-r','/tf_static:=/'+self.local_vehicle+'/tf_static',
            '-p','use_sim_time:=true','-p','target_frame:='+self.local_vehicle+'/odom','-p','body_frame:='+self.local_vehicle+'/base_link'])

    def measure_links(self):
        self.phase="waiting_for_real_maps"
        self.wait(lambda:len(self.input_state)==len(self.vehicles) and all(s.get("healthy",False) for s in self.input_state.values()),30)
        super().measure_links()

    def align_origins(self):
        if self.role=='ground':return
        super().align_origins()
        fleet={'map_frame':'swarm_map','input_timeout':1.0,'vehicles':{}}
        for v,d in self.deployment.items():
            n,e,down=d['px4_origin_in_fleet_ned']
            fleet['vehicles'][v]={'source_frame':v+'/odom','body_frame':v+'/base_link','origin_enu':[float(e),float(n),float(-down)],'origin_yaw':0.0}
        path=self.output/'nav-fleet.yaml';path.write_text(yaml.safe_dump(fleet))
        super().start('native-planner',['ros2','launch','ego_planner','seven_planners.launch.py','input_mode:=navigation',
            'vehicle_count:='+str(self.vehicle_count),'local_vehicle:='+self.local_vehicle,'start_proxy:=false','fleet_config:='+str(path),
            'parameters_file:='+str(self.root/'planner.yaml')])
        if self.coordination_scenario=='board_center':
            proxy=self.root/'ros2_proxy.py'
            command=['python3',str(proxy),'--ros-args','-p','use_sim_time:=true','-p','bind_host:=0.0.0.0',
                '-p','vehicle_count:='+str(self.vehicle_count),'-p','publish_odometry:=false','-p','relay_broadcast:=false','-p','map_frame:=swarm_map']
            super().start('swarm-proxy',command)


def main():
    rclpy.init();mission=Mission()
    def interrupted(*_):
        # stop_board sends SIGKILL if this process is still exiting after 8s.
        # The flight result is already recorded, so leave child reaping to stop_board.
        if os.environ.get('AEROHUB_ROLE')=='board_uav':os._exit(0)
        raise InterruptedError('mission interrupted')
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
