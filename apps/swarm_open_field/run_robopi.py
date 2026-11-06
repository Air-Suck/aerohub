#!/usr/bin/env python3
"""Three Swarm-Formation nodes: drone0 on RoboPi; drone1/2 on PC Docker."""
import argparse,fcntl,hashlib,json,os,select,shlex,socket,socketserver,subprocess,sys,tarfile,threading,time
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[2];APP=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'apps/catora_open_field'))
from sim_startup import launch_vehicles
from orchestrate import Workers
PREFIX='aerohub-swarm-field-'
NAMES=[PREFIX+'clock',PREFIX+'audit']+[PREFIX+f'{kind}{i}' for kind in ('vehicle','sensor','compute') for i in range(3) if kind!='compute' or i>0]

def execute(cmd,**kw):return subprocess.run(list(map(str,cmd)),check=True,**kw)
def manager(action,out):return [sys.executable,ROOT/'scripts/airsim/manager.py',action,'--profile',APP/'profile.json','--run-dir',out/'scene']

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--preflight-only',action='store_true');parser.add_argument('--compute-image',default='uav/swarm-hybrid-compute:local');args=parser.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if any((out/n).exists() for n in ('mission.log','result.json','failure.json')):raise RuntimeError('choose fresh output directory')
    lock=(ROOT/'artifacts/robopi-runtime/cli.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    loaded=subprocess.check_output(['bash','-c','source "$1/scripts/load_env.sh"; env -0','env',str(ROOT)])
    os.environ.update(dict(item.decode().split('=',1) for item in loaded.split(b'\0') if b'=' in item))
    sys.path.insert(0,str(ROOT/'scripts'));from robopi import board,hdc,clock_offset,profile
    image=os.environ['FLIGHT_BOARD_IMAGE'];remote='/data/aerohub/lib/swarm-formation/validation/run-'+str(time.time_ns())
    for name in NAMES:
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
    fleet=yaml.safe_load((APP/'fleet.yaml').read_text());(out/'board-fleet.yaml').write_text(yaml.safe_dump({'vehicles':{'drone0':fleet['vehicles']['drone0']}}))
    with tarfile.open(out/'app.tar.gz','w:gz',dereference=True,format=tarfile.GNU_FORMAT) as archive:
        for name in ('mission.py','task.yaml','deployment.yaml','settings.json','planner.yaml'):archive.add(APP/name,arcname=name)
        for source,target in [('apps/catora_open_field/mission.py','common/flight_mission.py'),('apps/catora_open_field/stop_board.py','stop_board.py'),
            ('apps/catora_open_field/hybrid/metrics.py','metrics.py'),('runtime/as2/run.py','run_as2.py'),('runtime/as2/selected_behaviors.launch.py','selected_behaviors.launch.py'),
            ('runtime/as2/scoped_tf.launch.py','scoped_tf.launch.py'),('runtime/airsim_sensors/uav_as2_sensor_adapter/rpc.py','airsim_rpc.py'),
            ('config/as2/config.yaml','config.yaml'),('config/as2/pid_speed_controller.yaml','pid.yaml')]:archive.add(ROOT/source,arcname=target)
        archive.add(out/'board-fleet.yaml',arcname='fleet.yaml');archive.add(out/'board-dds.xml',arcname='dds.xml');archive.add(vendor/'msgpack',arcname='msgpack')
    hdc('file','send',str(out/'app.tar.gz'),remote+'/app.tar.gz')
    digest=hashlib.sha256((out/'app.tar.gz').read_bytes()).hexdigest()
    board('python3 -c '+shlex.quote(f'import hashlib;assert hashlib.sha256(open({remote+"/app.tar.gz"!r},"rb").read()).hexdigest()=={digest!r}'))
    board(f'tar -xzf {remote}/app.tar.gz -C {remote} && mkdir -p {remote}/results {remote}/runtime')
    values={'ROS_DOMAIN_ID':'91','ROS_LOCALHOST_ONLY':'0','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','FASTRTPS_DEFAULT_PROFILES_FILE':remote+'/dds.xml',
        'CATORA_APP_ROOT':remote,'CATORA_OUTPUT':remote+'/results','AIRSIM_HOST':os.environ['PC_IP'],'AIRSIM_PORT':'42473',
        'AS2_LAUNCHER':remote+'/run_as2.py','AS2_CONFIG_FILE':remote+'/config.yaml','AS2_CONTROLLER_CONFIG_FILE':remote+'/pid.yaml',
        'AS2_FLEET_FILE':remote+'/fleet.yaml','AS2_RUNTIME_DIR':remote+'/runtime','AS2_BEHAVIORS':'takeoff,land','AS2_TF_SCOPE':'per-vehicle',
        'AS2_RESET_OOM_SCORE':'1','AS2_MAX_RSS_MB':'3072','PLANNER_STARTUP_TIMEOUT':'30.0','CATORA_PREFLIGHT_ONLY':'1' if args.preflight_only else '0',
        'PLANNER_APP_WALL_TIMEOUT':'180','PLANNER_STATE_SOURCE':'Swarm input adapter checked registered navigation odometry in the same local frame as AS2 execution',
        'CATORA_MIXED':'1','CATORA_LOCAL_VEHICLE':'drone0','CATORA_COMPUTE_ID':'robopi-drone0',
        'CATORA_METRICS_SCRIPT':remote+'/metrics.py','CATORA_METRICS_FILE':remote+'/results/metrics.jsonl',
        'PLANNER_REGISTRY':'/data/aerohub/lib/swarm-formation/install/uav_planner_framework/share/uav_planner_framework/registry',
        'CATORA_TOPOLOGY':'RoboPi: drone0 native Swarm/map/navigation/adapter/AS2 + global sampler/observer/proxy; PC: two independent native Swarm/map/navigation/adapter/AS2 containers, three PX4/LiDAR and AirSim'}
    (out/'topology.json').write_text(json.dumps({'environment':values,'clock_offset_s':offset,'mode':'two-pc-one-robopi','vehicle_count':3,
        'native_planner_per_vehicle':True,'live_transport':'DDS UDP Ethernet; PC containers share host NIC','startup_transfer':'one-time HDC calibration and resolved YAML with SHA256'},indent=2)+'\n')
    class Relay(socketserver.BaseRequestHandler):
        def handle(self):
            with socket.create_connection(('127.0.0.1',42471),timeout=5) as peer:
                self.request.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);peer.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                while True:
                    ready,_,_=select.select([self.request,peer],[],[],10)
                    if not ready:return
                    for source in ready:
                        data=source.recv(65536)
                        if not data:return
                        (peer if source is self.request else self.request).sendall(data)
    class Server(socketserver.ThreadingTCPServer):allow_reuse_address=True;daemon_threads=True
    relay=Server((os.environ['PC_IP'],42473),Relay);threading.Thread(target=relay.serve_forever,daemon=True).start()
    workers=Workers(out,APP,PREFIX,args.compute_image,remote,board,hdc);relays=[]
    try:
        execute(manager('start',out)+['--timeout','90'])
        relays=launch_vehicles(image,APP,out,PREFIX,out/'pc-dds.xml')
        execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'clock','-e','ROS_DOMAIN_ID=91','-e','VEHICLE_NAME=Drone0','-e','AIRSIM_RPC_PORT=42471',
            '-v',str(out/'pc-dds.xml')+':/opt/fastdds_udp.xml:ro',image,'clock'],stdout=subprocess.DEVNULL)
        for i in range(3):
            command='source /opt/ros/humble/setup.bash; source /opt/platform_sensors/install/setup.bash; exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args -p use_sim_time:=true -p rpc_port:=42471 -p vehicle_name:=Drone'+str(i)+' -p vehicle_namespace:=drone'+str(i)+' -p settings_file:=/config/settings.json -p camera_enabled:=false -r /tf_static:=/drone'+str(i)+'/tf_static'
            execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+f'sensor{i}','-e','ROS_DOMAIN_ID=91','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml',
                '-v',str(out/'pc-dds.xml')+':/run/dds.xml:ro','-v',str(APP/'settings.json')+':/config/settings.json:ro','--entrypoint','bash',image,'-c',command],stdout=subprocess.DEVNULL)
        if not args.preflight_only:
            execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'audit',
                '-e','ROS_DOMAIN_ID=91','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml',
                '-v',str(APP/'audit.py')+':/audit.py:ro','-v',str(out)+':/results',
                '-v',str(out/'pc-dds.xml')+':/run/dds.xml:ro',args.compute_image,'python3','/audit.py'],stdout=subprocess.DEVNULL)
        workers.start()
        exports=' '.join(k+'='+shlex.quote(v) for k,v in values.items())
        board(f'cd {remote} && . /data/aerohub/lib/swarm-formation/env.sh && export {exports} && python3 mission.py > results/mission.log 2>&1',timeout=700)
    finally:
        try:
            (out/'board-cleanup.json').write_text(board(f'python3 {remote}/stop_board.py {remote}',timeout=20)+'\n')
            board(f'cd {remote} && tar -czf evidence.tar.gz results runtime')
            hdc('file','recv',remote+'/evidence.tar.gz',str(out/'evidence.tar.gz'))
            with tarfile.open(out/'evidence.tar.gz') as archive:archive.extractall(out,filter='data')
            for name in ('mission.log','failure.json','result.json'):
                src=out/'results'/name
                if src.exists():(out/name).write_bytes(src.read_bytes())
        finally:
            relay.shutdown();relay.server_close();workers.stop()
            for transport in relays:transport.close()
            for name in NAMES:
                with (out/(name+'.log')).open('w') as log:subprocess.run(['docker','logs',name],stdout=log,stderr=subprocess.STDOUT)
                subprocess.run(['docker','stop','--timeout','8',name],capture_output=True)
            subprocess.run(manager('stop',out),check=False)
    if args.preflight_only:print((out/'results/ground-probe.json').read_text());return
    result=json.loads((out/'result.json').read_text());assert result['planner_status']=='SUCCEEDED' and result['landed_and_disarmed'];print(json.dumps(result,indent=2))

if __name__=='__main__':main()
