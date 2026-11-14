#!/usr/bin/env python3
"""Independent local four-role YOPO validation; shared images stay unchanged."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.request import build_opener, ProxyHandler

ROOT = Path(__file__).resolve().parents[4]
APP = Path(__file__).resolve().parent
ONBOARD = 'onboard_computer:v0.1.0'
GROUND = 'osrf/ros:humble-desktop-full-jammy'
FLIGHT = 'flight_board:v0.1.0'
DOMAIN = 196  # Legacy runs; new runs record an unused domain in domain.json.


def run(args, **kwargs):
    return subprocess.run(list(map(str, args)), check=True, **kwargs)


def capture(args):
    return subprocess.check_output(list(map(str, args)), text=True, stderr=subprocess.STDOUT)


def manager(action, output):
    return [sys.executable, ROOT/'simulator/airsim/manager/manager.py', action,
            '--profile', output/'profile.json', '--run-dir', output/'scene']


def prepare_ground():
    output = ROOT/'log/yopo-single/workstation'
    source = output/'ws/src/as2_msgs'
    source.mkdir(parents=True, exist_ok=True)
    manifest = {}
    # Only canonical interfaces consumed by the task, generated in its own environment.
    for name in ['msg/PlatformInfo.msg', 'msg/PlatformStatus.msg', 'msg/ControlMode.msg',
                 'action/Takeoff.action', 'action/Land.action']:
        text = capture(['docker','run','--rm','--network','none','--entrypoint','cat',ONBOARD,
                        '/root/aerostack2_ws/src/aerostack2/as2_msgs/'+name])
        path = source/name; path.parent.mkdir(exist_ok=True); path.write_text(text)
        manifest[name] = hashlib.sha256(text.encode()).hexdigest()
    (source/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.8)
project(as2_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/PlatformInfo.msg" "msg/PlatformStatus.msg"
  "msg/ControlMode.msg" "action/Takeoff.action" "action/Land.action" DEPENDENCIES std_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
''')
    (source/'package.xml').write_text('''<package format="3"><name>as2_msgs</name><version>1.1.3</version>
<description>Canonical task interface subset from onboard image</description>
<maintainer email="dev@example.com">Aerohub</maintainer><license>BSD-3-Clause</license>
<buildtool_depend>ament_cmake</buildtool_depend><build_depend>rosidl_default_generators</build_depend>
<depend>std_msgs</depend><depend>action_msgs</depend><exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group><export><build_type>ament_cmake</build_type></export></package>''')
    manifest['ground_image'] = capture(['docker','image','inspect',GROUND,'--format','{{.Id}}']).strip()
    manifest['onboard_image'] = capture(['docker','image','inspect',ONBOARD,'--format','{{.Id}}']).strip()
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    run(['docker','run','--rm','--network','none','-v',str(ROOT)+':/repo','-w','/repo/log/yopo-single/workstation/ws',
         '--entrypoint','bash',GROUND,'-lc','source /opt/ros/humble/setup.bash && colcon build --event-handlers console_direct+'])


def domain(output):
    path=output/'domain.json'
    return json.loads(path.read_text())['domain'] if path.exists() else DOMAIN


def common(output):
    return ['--init','--network','host','-e','ROS_DOMAIN_ID='+str(domain(output)),
            '-e','ROS_LOCALHOST_ONLY=1','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml',
            '-e','PYTHONDONTWRITEBYTECODE=1','-v',str(ROOT)+':/repo:ro','-v',str(output)+':/evidence']


def prefix(output):
    return 'yopo-single-'+hashlib.sha256(str(output).encode()).hexdigest()[:8]+'-'


def launch(output, role, image, args, env=(), mounts=(), entrypoint=None):
    command = ['docker','run','-d','--name',prefix(output)+role,
               '--label','aerohub.yopo.output='+str(output),*common(output),*env,*mounts]
    if entrypoint: command += ['--entrypoint',entrypoint]
    command += [image,*args]
    run(command)


def relay(output):
    sys.path.insert(0, str(ROOT/'deployment/airsim/demos/catora_open_field'))
    from sim_startup import StartupRelay
    service = StartupRelay(('127.0.0.1',4790), ('127.0.0.1',4780))
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    try:
        while not stopping: time.sleep(.1)
    finally:
        service.close()


def start(output, scene):
    output.mkdir(parents=True, exist_ok=True)
    if (output/'images.json').exists():
        raise RuntimeError('use a fresh output directory')
    used={json.loads(p.read_text())['domain'] for p in (ROOT/'log/yopo-single').rglob('domain.json')}
    identifiers=capture(['docker','ps','-q']).split()
    if identifiers:
        for container in json.loads(capture(['docker','inspect',*identifiers])):
            for entry in container['Config']['Env']:
                if entry.startswith('ROS_DOMAIN_ID='):
                    used.add(int(entry.split('=',1)[1]))
    available=[value for value in range(110,171) if value not in used]
    if not available:
        raise RuntimeError('no unused validation DDS domain available')
    (output/'domain.json').write_text(json.dumps({'domain':available[0]})+'\n')
    for kind, ports in [(socket.SOCK_STREAM, [45471,4780,4790]), (socket.SOCK_DGRAM,[8961,14840,14880])]:
        for port in ports:
            with socket.socket(socket.AF_INET, kind) as probe:
                probe.bind(('127.0.0.1',port))
    for image in [ONBOARD, GROUND, FLIGHT]:
        run(['docker','image','inspect',image],stdout=subprocess.DEVNULL)
    settings = json.loads((APP/'settings.json').read_text())
    if scene == 'forest':
        forest = json.loads((ROOT/'simulator/airsim/scenarios/super_forest/settings.json').read_text())['Vehicles']['PX4']
        for key in ['X','Y','Z','Yaw']:
            settings['Vehicles']['PX4'][key] = forest.get(key,0)
    (output/'scenario.json').write_text(json.dumps({'scene':scene})+'\n')
    (output/'settings.json').write_text(json.dumps(settings, indent=2)+'\n')
    px4 = json.loads(json.dumps(settings)); px4['Vehicles']['PX4']['TcpPort']=4790
    (output/'px4-settings.json').write_text(json.dumps(px4, indent=2)+'\n')
    (output/'fleet.yaml').write_text('vehicles:\n  drone0:\n    fmu_prefix: /drone0\n    target_system_id: 71\n')
    profile = json.loads((ROOT/'simulator/airsim/scenarios/catora_open_field/profile.json').read_text())
    profile['settings'] = str(output/'settings.json')
    (output/'profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    images = {image:capture(['docker','image','inspect',image,'--format','{{.Id}}']).strip() for image in [ONBOARD,GROUND,FLIGHT]}
    (output/'images.json').write_text(json.dumps(images,indent=2)+'\n')
    source_manifest = {'files': {}, 'as2_checkout': capture(['git','-C',ROOT/'as2/aerostack2','rev-parse','HEAD']).strip(),
                       'as2_gitlink': capture(['git','-C',ROOT,'ls-files','-s','as2/aerostack2']).strip()}
    for folder in [APP, ROOT/'src/tool/navigation/planner/yopo/ros2_adapter', ROOT/'src/basic_function/inference/interfaces',
                   ROOT/'src/basic_function/inference/ros2_adapter', ROOT/'src/behavior/follow_path']:
        for path in folder.rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:
                source_manifest['files'][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    with build_opener(ProxyHandler({})).open('http://127.0.0.1:18081/health/ready', timeout=3) as response:
        source_manifest['model_health'] = json.load(response)
    health = source_manifest['model_health']
    weight_hash = (ROOT/'src/basic_function/inference/models/yopo/weights.sha256').read_text().split()[0]
    if (health.get('model_id') != 'yopo' or health.get('device') != 'cuda' or
            health.get('velocity_m_s') != 2.0 or health.get('preprocessing_version') != 'ros-reference-u8-inpaint-v1' or
            not health.get('model_version','').startswith('sha256:'+weight_hash)):
        raise RuntimeError('Demo requires the frozen CUDA YOPO weights/preprocessing and 2m/s profile: '+str(health))
    (output/'source-manifest.json').write_text(json.dumps(source_manifest,indent=2)+'\n')

    run(manager('start',output)+['--timeout','90'])
    stream = (output/'relay.log').open('w')
    process = subprocess.Popen([sys.executable, __file__, 'relay','--output',str(output)],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    stream.close()
    fields=Path(f'/proc/{process.pid}/stat').read_text().rsplit(')',1)[1].split()
    (output/'relay.json').write_text(json.dumps(dict(pid=process.pid,ticks=fields[19],boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()))+'\n')
    launch(output,'vehicle',FLIGHT,['vehicle'],
           env=['-e','VEHICLE_NAME=PX4','-e','VEHICLE_NAMESPACE=drone0','-e','PX4_INSTANCE=70',
                '-e','PX4_SYSTEM_ID=71','-e','XRCE_AGENT_PORT=8961','-e','AIRSIM_RPC_PORT=45471',
                '-e','FLIGHT_BOARD_SENSORS=0','-e','FLIGHT_BOARD_CAMERAS=0'],
           mounts=['-v',str(output/'px4-settings.json')+':/config/settings.json:ro'])
    launch(output,'clock',FLIGHT,['clock'],env=['-e','VEHICLE_NAME=PX4','-e','AIRSIM_RPC_PORT=45471'])
    deadline=time.monotonic()+60
    while 'Ready for takeoff!' not in capture(['docker','logs',prefix(output)+'vehicle']):
        if time.monotonic()>deadline: raise RuntimeError('PX4 preflight timeout; preserve resources for diagnosis')
        time.sleep(.25)
    launch(output,'sensor',FLIGHT,['-lc',
           'source /opt/ros/humble/setup.bash && source /opt/platform_sensors/install/setup.bash && '
           'exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args '
           '-p settings_file:=/config/settings.json -p rpc_host:=127.0.0.1 -p rpc_port:=45471 '
           '-p vehicle_name:=PX4 -p vehicle_namespace:=drone0 -p camera_rate_hz:=15.0 -p use_sim_time:=true'],
           mounts=['-v',str(output/'settings.json')+':/config/settings.json:ro'],entrypoint='bash')
    launch(output,'compute',ONBOARD,['-lc','exec bash /repo/deployment/airsim/demos/yopo_single/compute.sh'],
           env=['-e','YOPO_FLEET_FILE=/evidence/fleet.yaml'],entrypoint='bash')


def mission(output, args):
    case_path=output/args.case
    if case_path.exists():
        raise RuntimeError('use a fresh case name; evidence must not be overwritten')
    case_path.mkdir(parents=True)
    command=['docker','run','--rm',*common(output),
             '-e','PYTHONPATH=/repo/simulator/airsim/sensors:/repo/deployment/robopi/vendor',
             '--entrypoint','bash',GROUND,'-lc',
             'source /opt/ros/humble/setup.bash && source /repo/log/yopo-single/workstation/ws/install/setup.bash && '
             'exec python3 /repo/deployment/airsim/demos/yopo_single/mission.py --output /evidence/'+args.case+
             (' --require-obstacle' if json.loads((output/'scenario.json').read_text())['scene']=='forest' else '')+(' --ground-only' if args.ground_only else '')+(' --recover-only' if args.recover_only else '')+' --timeout '+str(args.timeout)+' --fault '+args.fault+' --goal '+' '.join(str(v) for v in args.goal)]
    if args.fault not in ('stale', 'worker-exit'):
        run(command)
        return
    request = output/args.case/'fault-request.json'
    if request.exists():
        raise RuntimeError('fault case directory already used')
    process = subprocess.Popen(list(map(str,command)))
    paused = None
    injected = False
    try:
        while process.poll() is None:
            if request.exists() and not injected:
                evidence = {'fault':args.fault,'wall':time.time()}
                if args.fault == 'stale':
                    container = prefix(output)+'sensor'
                    owner = capture(['docker','inspect','--format','{{index .Config.Labels "aerohub.yopo.output"}}',container]).strip()
                    if owner != str(output):
                        raise RuntimeError('sensor container owner mismatch')
                    run(['docker','pause',container])
                    paused = container
                    evidence['sensor_container'] = container
                else:
                    if args.worker_pid <= 0:
                        raise ValueError('--worker-pid must identify this run CUDA worker')
                    cmd = Path('/proc/'+str(args.worker_pid)+'/cmdline').read_bytes().split(b'\0')
                    if not all(token in cmd for token in [b'uvicorn',b'serve:app',b'18081']):
                        raise RuntimeError('worker process identity mismatch')
                    cwd = Path('/proc/'+str(args.worker_pid)+'/cwd').resolve()
                    if cwd not in (ROOT, ROOT/'src/basic_function/inference/models/yopo'):
                        raise RuntimeError('worker workspace mismatch')
                    evidence['worker_pid'] = args.worker_pid
                    evidence['worker_stat'] = Path('/proc/'+str(args.worker_pid)+'/stat').read_text()
                    os.kill(args.worker_pid,signal.SIGINT)
                (output/args.case/'fault-injection.json').write_text(json.dumps(evidence,indent=2)+'\n')
                injected = True
            time.sleep(.1)
        if process.returncode:
            raise subprocess.CalledProcessError(process.returncode,command)
        if not injected:
            raise RuntimeError('task ended before fault injection')
    finally:
        if paused:
            run(['docker','unpause',paused])
            (output/args.case/'sensor-restored.json').write_text(json.dumps({'wall':time.time(),'sensor_container':paused})+'\n')



def stop(output):
    # Preserve flight/clock/scene on unconfirmed recovery; never freeze an armed simulation.
    results=list(output.glob('*/result.json'))
    if not results or not json.loads(max(results,key=lambda p:p.stat().st_mtime).read_text()).get('disarmed'):
        raise RuntimeError('latest task must confirm disarmed before cleanup')
    for role in ['compute','sensor','vehicle','clock']:
        name=prefix(output)+role
        label=capture(['docker','inspect','--format','{{index .Config.Labels "aerohub.yopo.output"}}',name]).strip()
        if label!=str(output): raise RuntimeError('resource owner mismatch')
        (output/(role+'.txt')).write_text(capture(['docker','logs',name]))
        run(['docker','stop','--timeout','15',name]); run(['docker','rm',name])
    state=json.loads((output/'relay.json').read_text())
    try:
        fields=Path(f'/proc/{state["pid"]}/stat').read_text().rsplit(')',1)[1].split()
        command=Path(f'/proc/{state["pid"]}/cmdline').read_bytes()
        if (fields[19]==state['ticks'] and state['boot']==Path('/proc/sys/kernel/random/boot_id').read_text().strip()
                and str(output).encode() in command and b'relay' in command):
            os.killpg(state['pid'],signal.SIGTERM)
    except FileNotFoundError: pass
    run(manager('stop',output))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['prepare-workstation','start','mission','stop','relay','restart-compute'])
    parser.add_argument('--output',type=Path,default=ROOT/'log/yopo-single/sitl-01')
    parser.add_argument('--case',default='ground')
    parser.add_argument('--ground-only',action='store_true')
    parser.add_argument('--recover-only',action='store_true')
    parser.add_argument('--fault',choices=['none','cancel','stale','worker-exit'],default='none')
    parser.add_argument('--worker-pid',type=int,default=0)
    parser.add_argument('--scene',choices=['open','forest'],default='open')
    parser.add_argument('--timeout',type=float,default=90.)
    parser.add_argument('--goal',nargs=3,type=float,default=[0.,15.,2.])
    args=parser.parse_args(); output=args.output.resolve()
    if not output.is_relative_to(ROOT/'log'):
        raise ValueError('output must be inside repository log/')
    if not args.case.replace('-','').replace('_','').isalnum():
        raise ValueError('case must be an identifier')
    if args.action=='prepare-workstation': prepare_ground()
    elif args.action=='start': start(output,args.scene)
    elif args.action=='mission': mission(output,args)
    elif args.action=='restart-compute':
        name=prefix(output)+'compute'
        state=json.loads(capture(['docker','inspect',name]))[0]
        if state['Config']['Labels'].get('aerohub.yopo.output')!=str(output) or state['State']['Running']:
            raise RuntimeError('restart requires this run own stopped compute container')
        (output/('compute-before-restart-'+str(time.time_ns())+'.txt')).write_text(capture(['docker','logs',name]))
        run(['docker','rm',name])
        launch(output,'compute',ONBOARD,['-lc','exec bash /repo/deployment/airsim/demos/yopo_single/compute.sh'],
               env=['-e','YOPO_FLEET_FILE=/evidence/fleet.yaml'],entrypoint='bash')
    elif args.action=='stop': stop(output)
    elif args.action=='relay': relay(output)


if __name__=='__main__':
    main()
