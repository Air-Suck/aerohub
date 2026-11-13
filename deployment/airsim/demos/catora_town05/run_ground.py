#!/usr/bin/env python3
"""Ground-only one-RoboPi/three-PC capability preflight. Never sends arming."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import socket
import shutil
import subprocess
import sys
import tarfile
import threading
import time

ROOT = Path(__file__).resolve().parents[4]
APP = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT), str(ROOT/'tools/robopi'),
               str(ROOT/'deployment/airsim/demos/catora_open_field')]
from robopi import board, hdc, profile
from sim_startup import launch_vehicles
from deployment.swarm.orchestrator import ServiceThread

PREFIX = 'aerohub-catora-town05-ground-'
COMPUTE_IMAGE = 'uav/apps-catora-open-field:migration-main-behavior'
WORKSTATION_IMAGE = 'osrf/ros:humble-desktop-full-jammy'


def run(command, **kwargs):
    return subprocess.run([str(x) for x in command], check=True, **kwargs)


def measure_clock(remote, output):
    code = f"""import socket,time
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind(({os.environ["ROBOPI_IP"]!r},42299));s.settimeout(15)
try:
 for _ in range(30):
  data,peer=s.recvfrom(128)
  if peer[0]!={os.environ["PC_IP"]!r}:continue
  if data==b'stop':break
  s.sendto(str(time.time_ns()).encode(),peer)
finally:s.close()
"""
    probe=output/'clock_probe.py'
    probe.write_text(code)
    hdc('file','send',str(probe),remote+'/clock_probe.py')
    board('nohup python3 '+remote+'/clock_probe.py >'+remote+'/clock.log 2>&1 </dev/null & echo clock_probe')
    time.sleep(.5)
    samples=[]
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.settimeout(2)
        try:
            for _ in range(10):
                start=time.time_ns();sock.sendto(b'time',(os.environ['ROBOPI_IP'],42299))
                data,_=sock.recvfrom(128);end=time.time_ns()
                samples.append({'offset_s':(int(data)-(start+end)/2)/1e9,'rtt_s':(end-start)/1e9})
        finally:sock.sendto(b'stop',(os.environ['ROBOPI_IP'],42299))
    passed=all(abs(s['offset_s'])+s['rtt_s']/2<=.5 for s in samples)
    (output/'clock-check.json').write_text(json.dumps({'samples':samples,'passed':passed,'time':time.time()},indent=2)+'\n')
    if not passed:raise RuntimeError('wall clock gate failed; do not arm')


def main(allow_flight=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--diagnostic', action='store_true', help='passive state-chain probe, no mission')
    parser.add_argument('--probe-vehicle', choices=('drone0','drone1','drone2','drone3'), default='drone0')
    parser.add_argument('--probe-offboard', action='store_true', help='diagnostic only: request Offboard on disarmed drone0, never arm')
    if allow_flight:
        parser.add_argument('--flight', action='store_true')
        parser.add_argument('--ground-evidence', type=Path)
    parser.add_argument('--pc-overlay', type=Path, help='workspace containing the isolated PC install overlay')
    parser.add_argument('--board-framework-root', type=Path, help='explicit pure-Python Planner prefix, separate from AS2 overlay')
    parser.add_argument('--board-as2-root', default='/data/aerohub/validation/super-follow-path/as2')
    parser.add_argument('--workstation-ws', type=Path,
                        default=ROOT/'log/airsim/super-four-part/workstation/ws')
    args = parser.parse_args()
    flight = allow_flight and args.flight
    if args.probe_offboard and not args.diagnostic:
        parser.error('--probe-offboard requires --diagnostic')
    if flight and args.diagnostic:
        parser.error('diagnostic mode never arms; do not combine with --flight')
    if flight:
        if not args.ground_evidence:
            parser.error('--flight requires --ground-evidence from successful mixed ground run')
        prior=json.loads((args.ground_evidence/'report.json').read_text())
        ready=json.loads((args.ground_evidence/'mission/preflight.json').read_text())
        if not prior.get('passed') or not ready.get('passed') or ready.get('actions_ready')!=12:
            parser.error('mixed ground evidence did not pass')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sources = list(APP.glob('*.py')) + list(APP.glob('*.yaml'))
    sources += [ROOT/'deployment/swarm/overhead_view.py', ROOT/'deployment/swarm/view.py',
                ROOT/'simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py']
    sources += [ROOT/'deployment/airsim/as2/run.py',ROOT/'src/behavior/follow_path/launch/motion_behaviors.launch.py']
    sources += list((ROOT/'src/planner/framework/uav_planner_framework').glob('*.py'))
    sources += list((ROOT/'src/planner/cat_ora/ros2_adapter/bridge/uav_cat_ora_adapter').glob('*.py'))
    sources += list((ROOT/'src/planner/framework/registry').rglob('*.yaml'))
    hashes = {}
    for source in sources:
        relative = source.relative_to(ROOT)
        target = output/'source-snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        hashes[str(relative)] = hashlib.sha256(source.read_bytes()).hexdigest()
    (output/'source-hashes.json').write_text(json.dumps(hashes, indent=2)+'\n')
    lock_path = ROOT/'log/airsim/robopi-runtime/cli.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock = lock_path.open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    for name in ('PC_IP', 'ROBOPI_IP', 'HDC_BIN', 'HDC_TARGET'):
        if not os.environ.get(name):
            raise ValueError('source tools/robopi/load_env.sh first: missing ' + name)
    framework_root = args.board_framework_root or Path(args.board_as2_root)/'install/uav_planner_framework'
    remote = '/data/aerohub/lib/catora/validation/run-town05-' + str(time.time_ns())
    names = [PREFIX+f'vehicle{i}' for i in range(4)] + [PREFIX+f'compute{i}' for i in range(1,4)] + [PREFIX+'clock', PREFIX+'workstation']
    existing = subprocess.check_output(['docker','ps','-a','--format','{{.Names}}'],text=True).splitlines()
    if set(existing).intersection(names):
        raise RuntimeError('owned container names already exist; inspect before reuse')
    flight_image = os.environ.get('FLIGHT_BOARD_IMAGE', 'flight_board:v0.1.0')
    identities = {image:subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
                  for image in (flight_image, COMPUTE_IMAGE, WORKSTATION_IMAGE)}
    (output/'images.json').write_text(json.dumps(identities,indent=2)+'\n')
    profile(output/'pc-dds.xml',os.environ['PC_IP'])
    profile(output/'board-dds.xml',os.environ['ROBOPI_IP'])
    with tarfile.open(output/'board.tar.gz','w:gz') as archive:
        archive.add(ROOT/'src/planner/framework/uav_planner_framework',arcname='python/uav_planner_framework',filter=lambda info: None if '__pycache__' in info.name else info)
        archive.add(ROOT/'src/planner/cat_ora/ros2_adapter/bridge/uav_cat_ora_adapter',arcname='python/uav_cat_ora_adapter',filter=lambda info: None if '__pycache__' in info.name else info)
        archive.add(ROOT/'src/planner/framework/registry',arcname='registry')
        for name in ('compute.py','state_probe.py','deployment.yaml','fleet.yaml','task.yaml','tasks'):
            archive.add(APP/name,arcname=name)
        for name in ('run.py','scoped_tf.launch.py','selected_behaviors.launch.py'):
            archive.add(ROOT/'deployment/airsim/as2'/name,arcname='as2/'+name)
        archive.add(ROOT/'src/behavior/follow_path/launch/motion_behaviors.launch.py',arcname='as2/motion_behaviors.launch.py')
        archive.add(ROOT/'deployment/airsim/demos/catora_open_field/stop_board.py',arcname='stop_board.py')
        archive.add(ROOT/'deployment/airsim/config/as2/config.yaml',arcname='config.yaml')
        archive.add(ROOT/'deployment/airsim/config/as2/pid_speed_controller.yaml',arcname='pid.yaml')
        archive.add(output/'board-dds.xml',arcname='dds.xml')
    board('mkdir -p '+remote)
    hdc('file','send',str(output/'board.tar.gz'),remote+'/app.tar.gz')
    digest=hashlib.sha256((output/'board.tar.gz').read_bytes()).hexdigest()
    check=f'import hashlib; assert hashlib.sha256(open({remote+"/app.tar.gz"!r},"rb").read()).hexdigest()=={digest!r}'
    board('python3 -c '+shlex.quote(check)+' && tar -xzf '+remote+'/app.tar.gz -C '+remote)
    (output/'topology.json').write_text(json.dumps({'remote':remote,'board_as2_root':args.board_as2_root,
        'central':'RoboPi/drone0','board_framework_root':str(framework_root),'pc_workers':['drone1','drone2','drone3'],
        'mission_image':WORKSTATION_IMAGE,'pc_overlay':str(args.pc_overlay.resolve()) if args.pc_overlay else None,'ground_only':not flight},indent=2)+'\n')
    manager=[sys.executable,str(ROOT/'simulator/airsim/manager/manager.py')]
    def scene(action):
        return manager+[action,'--profile',str(ROOT/'simulator/airsim/scenarios/catora_town05/profile.json'),
                        '--run-dir',str(output/'scene'),'--timeout','60']
    stop_event=threading.Event();workers=[];relays=[];board_started=False
    report={'passed':False,'ground_only':not flight,'arming_commands_sent':False}
    try:
        measure_clock(remote,output)
        run(scene('start'),stdout=(output/'scene-start.json').open('w'))
        relays=launch_vehicles(flight_image,ROOT/'simulator/airsim/scenarios/catora_town05',output,PREFIX,output/'pc-dds.xml')
        run(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'clock',
            '-e','ROS_DOMAIN_ID=91','-e','VEHICLE_NAME=Drone0','-e','AIRSIM_RPC_PORT=42471',
            '-v',str(output/'pc-dds.xml')+':/opt/fastdds_udp.xml:ro',
            '-v',str(ROOT)+':/repo:ro','--entrypoint','bash',flight_image,'-lc',
            'source /opt/platform_sensors/install/setup.bash; exec python3 /repo/deployment/airsim/demos/catora_town05/clock.py --ros-args -p rpc_port:=42471 -p vehicle_name:=Drone0'])
        common={'ROS_DOMAIN_ID':'91','ROS_LOCALHOST_ONLY':'0','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp',
                'FASTRTPS_DEFAULT_PROFILES_FILE':remote+'/dds.xml','CATORA_APP_ROOT':remote,
                'AS2_CONFIG_FILE':remote+'/config.yaml','AS2_CONTROLLER_CONFIG_FILE':remote+'/pid.yaml',
                'AS2_RESET_OOM_SCORE':'1','PYTHONDONTWRITEBYTECODE':'1'}
        exports=' '.join(k+'='+shlex.quote(v) for k,v in common.items())
        # Candidate owns all interface/AS2 prefixes; append only CAT-ORA native and adapter.
        environment='. '+shlex.quote(args.board_as2_root+'/board_env.sh')+'; '
        environment+='for name in mrs_formation_reshaping uav_cat_ora_adapter; do p=/data/aerohub/lib/catora/install/$name; export AMENT_PREFIX_PATH=$p:$AMENT_PREFIX_PATH LD_LIBRARY_PATH=$p/lib:$LD_LIBRARY_PATH PYTHONPATH=$p/lib/python3.12/site-packages:$PYTHONPATH; done; '
        environment+='export AMENT_PREFIX_PATH='+shlex.quote(str(framework_root))+':$AMENT_PREFIX_PATH PYTHONPATH='+shlex.quote(str(framework_root/'lib/python3.12/site-packages'))+':$PYTHONPATH; '
        environment+='export PYTHONPATH='+shlex.quote(remote+'/python')+':$PYTHONPATH; '
        command=f'nohup env {exports} python3 {remote}/compute.py --app {remote} --output {remote}/compute --vehicle drone0 --as2-launcher {remote}/as2/run.py --central --registry {remote}/registry >{remote}/supervisor.log 2>&1 </dev/null &'
        board(environment+command+' echo compute_requested')
        board_started=True
        if args.diagnostic:
            probe=f'nohup env {exports} python3 {remote}/state_probe.py --output {remote}/probe --vehicles drone0 --duration 30 --warmup 15 >{remote}/probe.log 2>&1 </dev/null &'
            board(environment+probe+' echo passive_probe_requested')
        for i in range(1,4):
            pc_overlay_setup = 'source /work/install/local_setup.bash; ' if args.pc_overlay else ''
            command=['docker','run','--rm','--init','--network','host','--name',PREFIX+f'compute{i}',
                '-e','ROS_DOMAIN_ID=91','-e','ROS_LOCALHOST_ONLY=0',
                '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/demo/pc-dds.xml',
                '-e','AS2_CONFIG_FILE=/repo/deployment/airsim/config/as2/config.yaml',
                '-e','AS2_CONTROLLER_CONFIG_FILE=/repo/deployment/airsim/config/as2/pid_speed_controller.yaml',
                '-e','PYTHONDONTWRITEBYTECODE=1','-v',str(ROOT)+':/repo:ro','-v',str(output)+':/run/demo',
                '--entrypoint','bash',COMPUTE_IMAGE,'-lc',
                f'source /opt/catora_app_ws/install/setup.bash; {pc_overlay_setup}exec python3 /repo/deployment/airsim/demos/catora_town05/compute.py --app /repo/deployment/airsim/demos/catora_town05 --output /run/demo/compute{i} --vehicle drone{i} --as2-launcher /repo/deployment/airsim/as2/run.py']
            if args.pc_overlay:
                entry=command.index('--entrypoint')
                command[entry:entry]=['-v',str(args.pc_overlay.resolve())+':/work:ro']
            worker=ServiceThread(identity=f'drone{i}',placement='pc',command=command,environment=dict(os.environ),
                log_path=output/f'worker{i}.log',stop_event=stop_event)
            workers.append(worker);worker.start()
        vendor=ROOT/'log/airsim/catora-town05-runtime/vendor'
        workstation=['docker','run','--rm','--init','--network','host','--name',PREFIX+'workstation',
            '-e','ROS_DOMAIN_ID=91','-e','ROS_LOCALHOST_ONLY=0','-e','PYTHONPATH=/vendor',
            '-e','MSGPACK_PUREPYTHON=1','-e','PYTHONDONTWRITEBYTECODE=1',
            '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/demo/pc-dds.xml',
            '-v',str(vendor)+':/vendor:ro','-v',str(args.workstation_ws.resolve())+':/work/ws:ro',
            '-v',str(ROOT)+':/repo:ro','-v',str(output)+':/run/demo','--entrypoint','bash',WORKSTATION_IMAGE,'-lc',
            f'source /work/ws/install/setup.bash; exec python3 /repo/deployment/airsim/demos/catora_town05/mission.py --output /run/demo/mission --compute-root {remote} --compute-deployment {remote}/deployment.yaml --local-deployment /repo/deployment/airsim/demos/catora_town05/deployment.yaml ' + ' --preflight-only --calibrate-ground --observe-seconds 60']
        if args.diagnostic:
            workstation[-1]='source /work/ws/install/setup.bash; exec python3 /repo/deployment/airsim/demos/catora_town05/state_probe.py --output /run/demo/diagnostics --duration 35 --warmup 15'+(' --offboard-vehicle '+args.probe_vehicle if args.probe_offboard else '')
        with (output/'mission.log').open('w') as log:
            run(workstation,stdout=log,stderr=subprocess.STDOUT,timeout=200)
        if flight:
            calibrated=output/'mission/deployment.yaml'
            hdc('file','send',str(calibrated),remote+'/calibrated-deployment.yaml')
            command=workstation[-1]
            command=command.replace('--output /run/demo/mission ', '--output /run/demo/flight ')
            command=command.replace('--compute-deployment '+remote+'/deployment.yaml', '--compute-deployment '+remote+'/calibrated-deployment.yaml')
            command=command.replace('--local-deployment /repo/deployment/airsim/demos/catora_town05/deployment.yaml', '--local-deployment /run/demo/mission/deployment.yaml')
            command=command.replace(' --preflight-only --calibrate-ground --observe-seconds 60', '')
            workstation[-1]=command
            measure_clock(remote,output)
            with (output/'flight.log').open('w') as log:
                run(workstation,stdout=log,stderr=subprocess.STDOUT,timeout=650)
        expected=output/'diagnostics/report.json' if args.diagnostic else output/('flight' if flight else 'mission')/('result.json' if flight else 'preflight.json')
        outcome=json.loads(expected.read_text())
        if not outcome.get('passed') or (flight and not outcome.get('landed_and_disarmed')):
            raise RuntimeError('mission did not confirm completion/disarm')
        report['passed']=True
    except BaseException as exc:
        report['error']=str(exc)
        raise
    finally:
        report['arming_commands_sent']=(output/'flight/arming-intent.json').exists()
        if flight and report['arming_commands_sent']:
            def recovered():
                for name in ('result.json','failure-stop.json'):
                    path=output/'flight'/name
                    if path.exists() and json.loads(path.read_text()).get('landed_and_disarmed'):
                        return True
                return False
            if not recovered():
                (output/'recovery-required.json').write_text(json.dumps({'remote':remote,'reason':'disarm not confirmed; capability and clock retained'})+'\n')
                print('Recovery required: retaining capability, clock and simulator; evidence: '+str(output),flush=True)
                while not recovered():time.sleep(1)
        if board_started:
            try:
                report['board_cleanup']=json.loads(board(f'python3 {remote}/stop_board.py {remote}',timeout=30))
                board(f'cd {remote}; tar -czf evidence.tar.gz compute supervisor.log'+(' probe probe.log' if args.diagnostic else ''))
                hdc('file','recv',remote+'/evidence.tar.gz',str(output/'board-evidence.tar.gz'))
            except Exception as exc:
                report['board_cleanup_error']=str(exc);report['passed']=False
        stop_event.set()
        for worker in workers:worker.join(timeout=12)
        (output/'pc-workers.json').write_text(json.dumps([w.record() for w in workers],indent=2)+'\n')
        for name in names:
            logs=subprocess.run(['docker','logs',name],capture_output=True)
            (output/(name+'.log')).write_bytes(logs.stdout+logs.stderr)
            subprocess.run(['docker','stop','--timeout','8',name],capture_output=True)
        for relay in relays:relay.close()
        subprocess.run(scene('stop'),capture_output=True)
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['passed']:raise RuntimeError('ground preflight or cleanup failed')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
