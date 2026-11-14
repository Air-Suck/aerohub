#!/usr/bin/env python3
"""Run a config-sized Swarm with one RoboPi UAV and remaining PC UAVs."""
import argparse,fcntl,hashlib,json,os,select,shlex,socket,socketserver,subprocess,sys,tarfile,threading,time
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[4];APP=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'deployment/airsim/demos/catora_open_field'))
sys.path.insert(0,str(ROOT))
from sim_startup import launch_vehicles
from orchestrate import Workers
from deployment.swarm.topology import load_config
from hybrid_plan import BOARD_RPC_RELAY_PORT,NATIVE_PARAMETERS,PREFIX,RPC_PORT,build_plan,write_plan

def execute(cmd,**kw):return subprocess.run(list(map(str,cmd)),check=True,**kw)
def manager(action,out,profile):return [sys.executable,ROOT/'simulator/manager/manager.py',action,'--profile',profile,'--run-dir',out/'scene']

def plan_from_files(config,scenario):
    return build_plan(config,scenario,yaml.safe_load((APP/'task.yaml').read_text()),
        yaml.safe_load((APP/'planner.yaml').read_text()),yaml.safe_load(NATIVE_PARAMETERS.read_text()))

def cleanup(out,steps):
    """Run every cleanup step even if an earlier one fails; keep each failure as evidence."""
    errors=[]
    for name,step in steps:
        try:step()
        except BaseException as exc:errors.append({'step':name,'error':f'{type(exc).__name__}: {exc}'})
    if errors:(out/'cleanup-errors.json').write_text(json.dumps(errors,indent=2)+'\n')
    return errors

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--preflight-only',action='store_true');parser.add_argument('--compute-image',default='uav/swarm-hybrid-compute:local')
    parser.add_argument('--swarm-config',type=Path,default=ROOT/'deployment/swarm/configs/swarm_open_field.yaml')
    parser.add_argument('--coordination-scenario',choices=('board_center','board_worker'),required=True)
    parser.add_argument('--fault-scenario',choices=(
        'none','swarm_worker_planner_exit','swarm_center_proxy_exit'),default='none')
    args=parser.parse_args()
    if args.preflight_only and args.fault_scenario!='none':
        raise ValueError('fault injection requires a full flight')
    required={'swarm_worker_planner_exit':'board_center',
        'swarm_center_proxy_exit':'board_worker'}
    if args.fault_scenario in required and args.coordination_scenario!=required[args.fault_scenario]:
        raise ValueError(args.fault_scenario+' requires '+required[args.fault_scenario])
    config=load_config(args.swarm_config);plan=plan_from_files(config,args.coordination_scenario)
    topology=plan['topology'];scenario=plan['scenario'];containers=plan['containers']
    count=topology['vehicle_count'];board_vehicle=topology['board_vehicle'];pc_vehicles=topology['pc_vehicles']
    NAMES=[name for group in containers.values() for name in group]
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if any((out/n).exists() for n in ('mission.log','result.json','failure.json','topology.json')):raise RuntimeError('choose fresh output directory')
    profile_data=json.loads((ROOT/'simulator/airsim/scenarios/swarm_open_field/profile.json').read_text())
    paths=write_plan(plan,out,profile_data,config['airsim']['camera'])
    settings_path=paths['settings'];profile_path=paths['profile']
    lock_path=ROOT/'log/airsim/robopi-runtime/cli.lock';lock_path.parent.mkdir(parents=True,exist_ok=True)
    lock=lock_path.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    loaded=subprocess.check_output(['bash','-c','source "$1/tools/robopi/load_env.sh"; env -0','env',str(ROOT)])
    os.environ.update(dict(item.decode().split('=',1) for item in loaded.split(b'\0') if b'=' in item))
    sys.path.insert(0,str(ROOT/'tools/robopi'));from robopi import board,hdc,clock_offset,profile
    image=os.environ['FLIGHT_BOARD_IMAGE'];remote='/data/aerohub/lib/swarm-formation/validation/run-'+str(time.time_ns())
    for name in NAMES+[PREFIX+'vendor']:
        if subprocess.run(['docker','inspect',name],capture_output=True).returncode==0:raise RuntimeError('existing application container: '+name)
    offset=clock_offset()
    if abs(offset)>.5:raise RuntimeError(f'Board clock offset {offset}; sync-time while idle')
    (out/'images.json').write_text(json.dumps({n:subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',v],text=True).strip() for n,v in [('compute',args.compute_image),('flight_board',image)]},indent=2)+'\n')
    execute([sys.executable,ROOT/'deployment/robopi/deploy_lib.py','--component','swarm-formation'])
    (out/'planner-configuration.json').write_text(json.dumps({'parameters':yaml.safe_load((APP/'planner.yaml').read_text()),'sha256':hashlib.sha256((APP/'planner.yaml').read_bytes()).hexdigest()},indent=2)+'\n')
    board('mkdir -p '+remote)
    profile(out/'pc-dds.xml',os.environ['PC_IP']);profile(out/'board-dds.xml',os.environ['ROBOPI_IP'])
    vendor=out/'vendor';vendor.mkdir()
    execute(['docker','create','--name',PREFIX+'vendor',image],stdout=subprocess.DEVNULL)
    try:execute(['docker','cp',PREFIX+'vendor:/usr/lib/python3/dist-packages/msgpack',vendor])
    finally:execute(['docker','rm',PREFIX+'vendor'],stdout=subprocess.DEVNULL)
    for binary in vendor.rglob('*.so'):binary.unlink()
    with tarfile.open(out/'app.tar.gz','w:gz',dereference=True,format=tarfile.GNU_FORMAT) as archive:
        for name,source in [('mission.py',APP/'mission.py'),('task.yaml',paths['task']),('deployment.yaml',paths['deployment']),('settings.json',settings_path),('planner.yaml',APP/'planner.yaml')]:archive.add(source,arcname=name)
        for source,target in [('deployment/airsim/demos/catora_open_field/mission.py','common/flight_mission.py'),('deployment/airsim/demos/catora_open_field/stop_board.py','stop_board.py'),
            ('deployment/airsim/demos/catora_open_field/hybrid/metrics.py','metrics.py'),('deployment/airsim/as2/run.py','run_as2.py'),('deployment/airsim/as2/selected_behaviors.launch.py','selected_behaviors.launch.py'),
            ('deployment/airsim/as2/scoped_tf.launch.py','scoped_tf.launch.py'),('simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py','airsim_rpc.py'),
            ('deployment/airsim/config/as2/config.yaml','config.yaml'),('deployment/airsim/config/as2/pid_speed_controller.yaml','pid.yaml'),('src/planner/swarm_formation/ros2_adapter/native/plan_manage/scripts/ros2_proxy.py','ros2_proxy.py')]:archive.add(ROOT/source,arcname=target)
        archive.add(paths['board_fleet'],arcname='fleet.yaml');archive.add(out/'board-dds.xml',arcname='dds.xml');archive.add(vendor/'msgpack',arcname='msgpack')
    hdc('file','send',str(out/'app.tar.gz'),remote+'/app.tar.gz')
    digest=hashlib.sha256((out/'app.tar.gz').read_bytes()).hexdigest()
    board('python3 -c '+shlex.quote(f'import hashlib;assert hashlib.sha256(open({remote+"/app.tar.gz"!r},"rb").read()).hexdigest()=={digest!r}'))
    board(f'tar -xzf {remote}/app.tar.gz -C {remote} && mkdir -p {remote}/results {remote}/runtime')
    values={'ROS_DOMAIN_ID':'91','ROS_LOCALHOST_ONLY':'0','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','FASTRTPS_DEFAULT_PROFILES_FILE':remote+'/dds.xml',
        'CATORA_APP_ROOT':remote,'CATORA_OUTPUT':remote+'/results','AIRSIM_HOST':os.environ['PC_IP'],'AIRSIM_PORT':str(BOARD_RPC_RELAY_PORT),
        'AS2_LAUNCHER':remote+'/run_as2.py','AS2_CONFIG_FILE':remote+'/config.yaml','AS2_CONTROLLER_CONFIG_FILE':remote+'/pid.yaml',
        'AS2_FLEET_FILE':remote+'/fleet.yaml','AS2_RUNTIME_DIR':remote+'/runtime','AS2_BEHAVIORS':'takeoff,land','AS2_TF_SCOPE':'per-vehicle',
        'AS2_RESET_OOM_SCORE':'1','AS2_MAX_RSS_MB':'3072','PLANNER_STARTUP_TIMEOUT':'30.0','CATORA_PREFLIGHT_ONLY':'1' if args.preflight_only else '0',
        'PLANNER_APP_WALL_TIMEOUT':'180','PLANNER_STATE_SOURCE':'Swarm input adapter checked registered navigation odometry in the same local frame as AS2 execution',
        'AEROHUB_SWARM_VEHICLE_COUNT':str(count),'AEROHUB_SWARM_SCENARIO':scenario['name'],
        'AEROHUB_SWARM_CENTER':scenario['center_vehicle'],'AEROHUB_SWARM_BOARD_VEHICLE':board_vehicle['name'],
        'CATORA_MIXED':'1','CATORA_LOCAL_VEHICLE':board_vehicle['name'],
        'CATORA_COMPUTE_ID':'robopi-'+board_vehicle['name'],
        'CATORA_METRICS_SCRIPT':remote+'/metrics.py','CATORA_METRICS_FILE':remote+'/results/metrics.jsonl',
        'PLANNER_REGISTRY':'/data/aerohub/lib/swarm-formation/install/uav_planner_framework/share/uav_planner_framework/registry',
        'AEROHUB_ROLE':'board_uav','CATORA_TOPOLOGY':f"RoboPi: {board_vehicle['name']} UAV node; PC ground container: task client; PC UAV containers: {len(pc_vehicles)}; center: {scenario['center_vehicle']}"}
    (out/'topology.json').write_text(json.dumps({'environment':values,'clock_offset_s':offset,
        'mode':'one-robopi-rest-pc','vehicle_count':count,'scenario':scenario,
        'board_vehicle':board_vehicle,'pc_vehicles':pc_vehicles,'roles':plan['roles'],'containers':containers,
        'native_planner_per_vehicle':True,'live_transport':'DDS UDP Ethernet; PC containers share host NIC','startup_transfer':'one-time HDC calibration and resolved YAML with SHA256'},indent=2)+'\n')
    class Relay(socketserver.BaseRequestHandler):
        def handle(self):
            with socket.create_connection(('127.0.0.1',RPC_PORT),timeout=5) as peer:
                self.request.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);peer.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                while True:
                    ready,_,_=select.select([self.request,peer],[],[],10)
                    if not ready:return
                    for source in ready:
                        data=source.recv(65536)
                        if not data:return
                        (peer if source is self.request else self.request).sendall(data)
    class Server(socketserver.ThreadingTCPServer):allow_reuse_address=True;daemon_threads=True
    relay=Server((os.environ['PC_IP'],BOARD_RPC_RELAY_PORT),Relay);threading.Thread(target=relay.serve_forever,daemon=True).start()
    workers=Workers(out,APP,PREFIX,args.compute_image,remote,board,hdc,pc_vehicles,plan['fleet'],count,
        scenario['name'],scenario['center_vehicle']);relays=[]
    try:
        execute(manager('start',out,profile_path)+['--timeout','90'])
        relays=launch_vehicles(image,APP,out,PREFIX,out/'pc-dds.xml',settings_path=settings_path,vehicles=plan['sim_vehicles'])
        execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'clock','-e','ROS_DOMAIN_ID=91','-e','VEHICLE_NAME='+board_vehicle['airsim_name'],'-e',f'AIRSIM_RPC_PORT={RPC_PORT}',
            '-v',str(out/'pc-dds.xml')+':/opt/fastdds_udp.xml:ro',image,'clock'],stdout=subprocess.DEVNULL)
        for vehicle,container in zip(topology['vehicles'],containers['sensors']):
            name=vehicle['name']
            command=f"source /opt/ros/humble/setup.bash; source /opt/platform_sensors/install/setup.bash; exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args -p use_sim_time:=true -p rpc_port:={RPC_PORT} -p vehicle_name:={vehicle['airsim_name']} -p vehicle_namespace:={name} -p settings_file:=/config/settings.json -p camera_enabled:=false -r /tf_static:=/{name}/tf_static"
            execute(['docker','run','--rm','--detach','--init','--network','host','--name',container,'-e','ROS_DOMAIN_ID=91','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml',
                '-v',str(out/'pc-dds.xml')+':/run/dds.xml:ro','-v',str(settings_path)+':/config/settings.json:ro','--entrypoint','bash',image,'-c',command],stdout=subprocess.DEVNULL)
        if not args.preflight_only:
            execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'audit',
                '-e','ROS_DOMAIN_ID=91','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml',
                '-e','AEROHUB_SWARM_VEHICLES='+','.join(vehicle['name'] for vehicle in topology['vehicles']),
                '-v',str(APP/'audit.py')+':/audit.py:ro','-v',str(out)+':/results',
                '-v',str(out/'pc-dds.xml')+':/run/dds.xml:ro',args.compute_image,'python3','/audit.py'],stdout=subprocess.DEVNULL)
        workers.start()
        exports=' '.join(k+'='+shlex.quote(v) for k,v in values.items())
        board_box={}
        def run_board():
            try:board(f'cd {remote} && . /data/aerohub/lib/swarm-formation/env.sh && export {exports} && python3 mission.py > results/mission.log 2>&1',timeout=700)
            except Exception as exc:board_box['error']=str(exc)
        board_thread=threading.Thread(target=run_board,name='board-uav',daemon=True);board_thread.start()
        proxy_host=os.environ['ROBOPI_IP'] if scenario['center_vehicle']==board_vehicle['name'] else '127.0.0.1'
        ground=[
            'docker','run','--rm','--init','--network','host','--name',PREFIX+'ground',
            '-e','ROS_DOMAIN_ID=91','-e','ROS_LOCALHOST_ONLY=0','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml','-e','AEROHUB_ROLE=ground',
            '-e','CATORA_NODE_NAME=swarm_ground_station','-e','CATORA_APP_ROOT=/opt/swarm_app','-e','CATORA_OUTPUT=/results',
            '-e','AIRSIM_HOST=127.0.0.1','-e',f'AIRSIM_PORT={RPC_PORT}','-e','SWARM_PROXY_HOST='+proxy_host,
            '-e','AEROHUB_SWARM_BOARD_VEHICLE='+board_vehicle['name'],'-e','AEROHUB_SWARM_VEHICLE_COUNT='+str(count),
            '-e','AEROHUB_SWARM_SCENARIO='+scenario['name'],'-e','AEROHUB_SWARM_CENTER='+scenario['center_vehicle'],
            '-e','AEROHUB_SWARM_FAULT='+args.fault_scenario,
            '-e','CATORA_MIXED=1','-e','CATORA_LOCAL_VEHICLE='+board_vehicle['name'],
            '-e','CATORA_PREFLIGHT_ONLY='+('1' if args.preflight_only else '0'),
            '-e','PLANNER_REGISTRY=/opt/swarm_ws/install/uav_planner_framework/share/uav_planner_framework/registry',
            '-e','PLANNER_STARTUP_TIMEOUT=30.0','-e','PLANNER_APP_WALL_TIMEOUT=180',
            '-v',str(paths['task'])+':/opt/swarm_app/task.yaml:ro','-v',str(paths['deployment'])+':/opt/swarm_app/deployment.yaml:ro',
            '-v',str(APP/'planner.yaml')+':/opt/swarm_app/planner.yaml:ro','-v',str(out/'pc-dds.xml')+':/run/dds.xml:ro',
            '-v',str(out)+':/results',args.compute_image,'python3','/opt/swarm_app/mission.py']
        injector_error={}
        def inject_fault():
            if args.fault_scenario=='none':return
            try:
                deadline=time.monotonic()+180
                while not (out/'fault-ready.json').exists():
                    if time.monotonic()>deadline:
                        raise TimeoutError('ground mission did not reach fault-ready phase')
                    time.sleep(.2)
                if args.fault_scenario=='swarm_center_proxy_exit':
                    target=next(v for v in pc_vehicles if v['name']==scenario['center_vehicle'])
                    marker='inject-center-proxy-exit'
                else:
                    target=next(v for v in pc_vehicles if v['name']!=scenario['center_vehicle'])
                    marker='inject-native-planner-exit'
                root=out/'compute'/target['name']
                (root/marker).write_text(args.fault_scenario+'\n')
                injected=root/'fault-injected.json'
                while not injected.exists():
                    if time.monotonic()>deadline:
                        raise TimeoutError('PC worker did not confirm fault injection')
                    time.sleep(.1)
                record=json.loads(injected.read_text())
                record['scenario']=args.fault_scenario
                (out/'fault-injected.json').write_text(json.dumps(record,indent=2)+'\n')
            except BaseException as exc:
                injector_error['error']=f'{type(exc).__name__}: {exc}'
        injector=threading.Thread(
            target=inject_fault,name='swarm-fault-injector',daemon=True)
        injector.start()
        with (out/'ground.log').open('w') as log:
            ground_result=subprocess.run(
                ground,check=False,stdout=log,stderr=subprocess.STDOUT)
        injector.join(timeout=5)
        if injector.is_alive() or injector_error:
            raise RuntimeError('fault injector failed: '+str(injector_error))
        ground_result.check_returncode()
    finally:
        def collect():
            (out/'board-cleanup.json').write_text(board(f'python3 {remote}/stop_board.py {remote}',timeout=20)+'\n')
            board(f'cd {remote} && tar -czf evidence.tar.gz results runtime')
            hdc('file','recv',remote+'/evidence.tar.gz',str(out/'evidence.tar.gz'))
            with tarfile.open(out/'evidence.tar.gz') as archive:archive.extractall(out,filter='data')
            for name in ('mission.log','failure.json','result.json'):
                src=out/'results'/name
                if src.exists():(out/name).write_bytes(src.read_bytes())
        def stop_relay():relay.shutdown();relay.server_close()
        def close_relays():
            for transport in relays:transport.close()
        def stop_containers():
            # PC compute containers log through their supervisor threads into compute/<vehicle>/container.log.
            for name in containers['services']+containers['vehicles']+containers['sensors']:
                with (out/(name+'.log')).open('w') as log:subprocess.run(['docker','logs',name],stdout=log,stderr=subprocess.STDOUT)
                subprocess.run(['docker','stop','--timeout','8',name],capture_output=True)
        cleanup(out,[('board evidence',collect),('board RPC relay',stop_relay),('PC workers',workers.stop),
            ('PX4 startup relays',close_relays),('PC containers',stop_containers),
            ('AirSim',lambda:subprocess.run(manager('stop',out,profile_path),check=False))])
    board_thread.join(timeout=30)
    if workers.failures():raise RuntimeError('PC UAV workers failed or left residue: '+json.dumps(workers.failures()))
    if board_box.get('error') and not args.preflight_only:raise RuntimeError('board UAV failed: '+board_box['error'])
    if args.preflight_only:print((out/'ground-probe.json').read_text());return
    result=json.loads((out/'result.json').read_text())
    expected='SUCCEEDED' if args.fault_scenario=='none' else 'FAILED'
    assert result['planner_status']==expected and result['landed_and_disarmed']
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
