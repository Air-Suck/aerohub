"""Isolated AirSim + flight_board acceptance, reusing fixed images and a local overlay."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parents[4]
APP = Path(__file__).resolve().parent

def call(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--binary', type=Path)
    p.add_argument('--rpc-port',type=int,default=24474)
    p.add_argument('--tcp-port',type=int,default=5483)
    p.add_argument('--relay-port',type=int,default=5583)
    p.add_argument('--short',action='store_true',help='Native short navigation gate before forest')
    p.add_argument('--short-offset',nargs=3,type=float,default=[4.,0.,0.],metavar=('EAST','NORTH','UP'),help='Short gate relative ENU displacement in meters')
    p.add_argument('--forest',action='store_true',help='Run the selected real IPC navigation task; requires --native')
    p.add_argument('--ground-only',action='store_true')
    p.add_argument('--task',type=Path,help='Independent task profile; requires --criteria and free XYZ paths')
    p.add_argument('--criteria',type=Path,help='Independent acceptance criteria for --task')
    p.add_argument('--clock-rate-hz',type=float,default=50.,help='This isolated demo clock RPC/publication rate; does not certify control frequency')
    p.add_argument('--native', action='store_true', help='Use real IPC native source for the control gate')
    p.add_argument('--workspace', type=Path, default=ROOT/'log/ipc/control-overlay')
    p.add_argument('--stop-mode', choices=['loss','cancel'], default='loss', help='Attitude validation stop scenario')
    p.add_argument('--attitude', action='store_true', help='Use attitude/thrust validation source and PX4 attitude loop')
    args = p.parse_args(); out = args.output.resolve(); workspace = args.workspace.resolve()
    if not math.isfinite(args.clock_rate_hz) or not 0 < args.clock_rate_hz <= 1000:
        p.error('--clock-rate-hz must be finite and in (0, 1000]')
    if bool(args.task) != bool(args.criteria): p.error('--task and --criteria must be supplied together')
    if args.task and not args.forest: p.error('--task requires --forest')
    if args.attitude and (args.native or args.ground_only or args.forest or args.short):
        p.error('--attitude is an independent control validation; cannot combine with IPC task modes')
    out.mkdir(parents=True, exist_ok=True)
    if (out/'images.json').exists(): raise RuntimeError('Choose a fresh output directory')
    if not (workspace/'install/setup.bash').exists(): raise RuntimeError('Build control overlay first')
    if args.native and not (ROOT/'log/ipc/native-deps/ros-install/lib/ipc_native_core/ipc_node').is_file():
        raise RuntimeError('Build IPC native node before starting simulation')
    image = os.environ.get('IPC_CONTROL_IMAGE', 'uav/apps-super-forest:repo-native-20261109')
    board = 'flight_board:v0.1.0'
    images = {key: subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',value],text=True).strip()
              for key,value in [('compute',image),('flight_board',board)]}
    with (out/'domain-probe.json').open('w') as log:
        call(['docker','run','--rm','--network','host','--entrypoint','/bin/bash',
            '-e','ROS_DOMAIN_ID=182','-v',str(ROOT)+':/repo:ro',images['compute'],'-lc',
            'source /root/aerostack2_ws/install/setup.bash && python3 /repo/deployment/airsim/demos/ipc_control/probe_domain.py'],stdout=log)
    (out/'images.json').write_text(json.dumps(images, indent=2)+'\n')
    prefix = 'aerohub-ipc-control-'
    names = [prefix+k for k in ('vehicle','clock','app')]
    for name in names:
        if subprocess.run(['docker','container','inspect',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError('Owned test name already exists: '+name)
    for kind, ports in [(socket.SOCK_STREAM, (args.rpc_port,args.tcp_port,args.relay_port)),(socket.SOCK_DGRAM,(8963,15743,15783))]:
        for port in ports:
            with socket.socket(socket.AF_INET, kind) as sock:
                if kind == socket.SOCK_STREAM: sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                try: sock.bind(('0.0.0.0',port))
                except OSError as exc: raise RuntimeError(f'Port {port}, type {kind} unavailable') from exc
    settings = json.loads((ROOT/'simulator/airsim/scenarios/super_forest/settings.json').read_text())
    if args.forest or args.short:
        if not args.native: raise RuntimeError('--forest requires --native')
        task=json.loads((args.task or ROOT/'deployment/airsim/demos/ipc_navigation/forest-task.json').read_text())
        if args.task and task.get('planar_path') is not False: raise RuntimeError('New task profiles must enable free XYZ paths')
        if args.short: task.update(name='IPC native short navigation gate',target_offset_enu_m=args.short_offset,require_detour=False,timeout_sim_s=60.,planar_path=False)
        else: settings['Vehicles']['PX4'].update(dict(zip(('X','Y','Z'),task['spawn_ned_m'])))
        if task.get('planar_path') is not False:
            raise RuntimeError('All new IPC flight validation requires free XYZ paths; old planar profiles are historical evidence only')
        (out/'forest-task.json').write_text(json.dumps(task,indent=2)+'\n')
        if args.forest and not args.short:
            criteria=args.criteria or ROOT/'deployment/airsim/demos/ipc_navigation/acceptance-criteria.json'
            call([sys.executable,ROOT/'deployment/airsim/demos/ipc_navigation/check_criteria.py',out/'forest-task.json',criteria])
            (out/'acceptance-criteria.json').write_text(criteria.read_text())
    settings['ApiServerPort'] = args.rpc_port
    settings['Vehicles']['PX4'].update(TcpPort=args.tcp_port,ControlPortLocal=15743,ControlPortRemote=15783)
    (out/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    settings['Vehicles']['PX4']['TcpPort'] = args.relay_port
    (out/'px4-settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    profile = json.loads((ROOT/'simulator/airsim/scenarios/catora_open_field/profile.json').read_text())
    profile['settings'] = str(out/'settings.json')
    (out/'profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    config = yaml.safe_load((ROOT/'deployment/airsim/config/as2/config.yaml').read_text())
    config['/**/controller_manager']['ros__parameters'].update(cmd_freq=100.,info_freq=50.)
    (out/'config.yaml').write_text(yaml.safe_dump(config))
    (out/'platform.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{
        'use_sim_time':True,'cmd_freq':100.,'info_freq':50.,'external_odom':False,
        'fmu_prefix':'/drone0','target_system_id':73,'max_thrust':15.,'min_thrust':.15}}}))
    hashes = {str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest()
        for directory in [APP,ROOT/'src/behavior/follow_reference'] + ([ROOT/'deployment/airsim/demos/attitude_control'] if args.attitude else []) for f in directory.rglob('*')
        if f.is_file() and '__pycache__' not in f.parts}
    (out/'manifest.json').write_text(json.dumps({'source_sha256':hashes,'domain':182,
        'clock_rate_hz_requested':args.clock_rate_hz,
        'control_mode':('attitude' if args.attitude else 'body_rates'),
        'compute_role':('real IPC native + AS2 on PC' if args.native else 'validation controller + AS2 on PC; not IPC or RoboPi'),
        'thresholds':(task if args.forest or args.short else {'max_hold_error_m':.75,'max_tilt_rad':.35,'max_stop_speed_mps':.5,
                      'hold_wall_seconds':5}),'motion_api':'AS2/PX4 only; AirSim RPC for scoring'},indent=2)+'\n')
    if args.native:
        native_sources = {str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest()
            for directory in [ROOT/'src/planner/ipc/ros2_adapter/native',ROOT/'deployment/airsim/demos/ipc_navigation']
            for f in directory.rglob('*') if f.is_file() and '__pycache__' not in f.parts}
        runtime_files=[ROOT/'log/ipc/native-deps/ros-install/lib/ipc_native_core/ipc_node',
            workspace/'install/lib/libfollow_reference_acro_plugin.so',
            workspace/'install/lib/libas2_motion_controller.so',
            workspace/'install/lib/as2_motion_controller/as2_motion_controller_node',
            workspace/'install/lib/as2_platform_pixhawk/as2_platform_pixhawk_node',
            workspace/'install/lib/uav_follow_reference_acro/follow_reference_acro_node',
            ROOT/'log/ipc/native-deps/install/lib/libOsqpEigen.so',ROOT/'log/ipc/native-deps/install/lib/libosqp.so']
        runtime_hashes={(str(f.relative_to(ROOT)) if f.is_relative_to(ROOT) else str(f)):hashlib.sha256(f.read_bytes()).hexdigest() for f in runtime_files}
        (out/'native-manifest.json').write_text(json.dumps({'source_sha256':native_sources,'runtime_sha256':runtime_hashes,
            'patch_sha256':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (ROOT/'deployment/platform/patches').glob('*body-rates*') if f.is_file()},
            'compute_role':'real IPC native + AS2 on PC','task':'forest' if args.forest else ('disarmed shadow' if args.ground_only else ('short navigation gate' if args.short else 'control gate')),
            'runtime_install':'log/ipc/native-deps/ros-install','dependencies_install':'log/ipc/native-deps/install',
            'not_accepted':['RoboPi','real aircraft','100Hz']},indent=2)+'\n')
    manager = [sys.executable,ROOT/'simulator/airsim/manager/manager.py']
    def scene(action):
        cmd = manager+[action,'--profile',out/'profile.json','--run-dir',out/'scene']
        if args.binary: cmd += ['--binary',args.binary]
        return cmd
    started = []; relay = None; app = None; scene_started = False
    try:
        call(scene('start')+['--timeout','90']); scene_started = True
        relaylog = (out/'relay.log').open('w')
        relay = subprocess.Popen([sys.executable,APP/'relay.py','--output',out,
            '--listen-port',str(args.relay_port),'--upstream-port',str(args.tcp_port)],
            stdout=relaylog,stderr=subprocess.STDOUT,start_new_session=True)
        deadline = time.monotonic()+5
        while not (out/'relay.json').exists():
            if relay.poll() is not None or time.monotonic()>deadline: raise RuntimeError('relay startup failed')
            time.sleep(.05)
        common = ['docker','run','--rm','--detach','--init','--network','host',
            '--label','aerohub.ipc.output='+str(out),'-e','ROS_DOMAIN_ID=182',
            '-e','AIRSIM_RPC_PORT='+str(args.rpc_port),'-e','VEHICLE_NAME=PX4']
        if args.native or args.attitude: common += ['-e','FLIGHT_BOARD_CAMERAS=0']
        call(common+['--name',names[0],'-e','PX4_INSTANCE=72','-e','PX4_SYSTEM_ID=73',
            '-e','XRCE_AGENT_PORT=8963','-e','VEHICLE_NAMESPACE=drone0',
            '-v',str(out/'px4-settings.json')+':/config/settings.json:ro',images['flight_board'],'vehicle'])
        started.append(names[0])
        if args.clock_rate_hz == 50.:
            clock_command = common+['--name',names[1],images['flight_board'],'clock']
        else:
            # Configure only this owned clock container, preserving the fixed
            # image and other demos' supervisor/default clock configuration.
            clock_command = common+['--name',names[1],'--entrypoint','/bin/bash',images['flight_board'],'-lc',
                'source /opt/ros/humble/setup.bash && source /opt/platform_sensors/install/setup.bash && '
                'exec ros2 run uav_as2_sensor_adapter airsim_clock_node --ros-args '
                f'-p rpc_port:={args.rpc_port} -p vehicle_name:=PX4 -p rate_hz:={args.clock_rate_hz}']
        call(clock_command); started.append(names[1])
        deadline = time.monotonic()+70
        while True:
            text = subprocess.check_output(['docker','logs',names[0]],stderr=subprocess.STDOUT,text=True)
            if 'Ready for takeoff!' in text: break
            if time.monotonic()>deadline: raise RuntimeError('PX4 not ready')
            time.sleep(.25)
        status = subprocess.check_output(['docker','exec',names[0],'/opt/px4-gazebo/bin/px4-listener',
            '--instance','72','vehicle_status','-n','1'],text=True,timeout=10)
        (out/'preflight-uorb.txt').write_text(status)
        passed = 'pre_flight_checks_pass: True' in status
        (out/'preflight.json').write_text(json.dumps({'checks_pass':passed,'source':'target firmware uORB vehicle_status'})+'\n')
        if not passed: raise RuntimeError('PX4 full preflight checks not passed')
        app_log = (out/'application.log').open('w')
        command = ['docker','run','--rm','--init','--network','host','--name',names[2],
            '--label','aerohub.ipc.output='+str(out),'-e','ROS_DOMAIN_ID=182',
            '-e','IPC_TEST_RPC_PORT='+str(args.rpc_port),
            '-e','AS2_BEHAVIORS=takeoff,land','-v',str(ROOT)+':/repo:ro',
            '-v',str(workspace)+':/overlay:ro','-v',str(out)+':/evidence',
            '--entrypoint','/bin/bash',images['compute'],'-lc',
            'source /root/aerostack2_ws/install/setup.bash && source /overlay/install/setup.bash && exec python3 /repo/deployment/airsim/demos/ipc_control/session.py']
        if args.attitude:
            command[command.index('--entrypoint'):command.index('--entrypoint')] = ['-e','EXTERNAL_ATTITUDE_TEST=1','-e','ATTITUDE_STOP_MODE='+args.stop_mode,'-e','OPENBLAS_NUM_THREADS=1','-e','OMP_NUM_THREADS=1','-e','MKL_NUM_THREADS=1']
        if args.native:
            command[command.index('--entrypoint'):command.index('--entrypoint')] = ['-e','IPC_NATIVE=1','-e','OPENBLAS_NUM_THREADS=1','-e','OMP_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-v',str(ROOT/'log/ipc/native-deps')+':/native:ro']
            command[-1] = command[-1].replace(' && exec python3', ' && source /native/ros-install/local_setup.bash && export LD_LIBRARY_PATH=/native/install/lib:$LD_LIBRARY_PATH && exec python3')
        if args.ground_only: command[command.index('--entrypoint'):command.index('--entrypoint')] = ['-e','IPC_GROUND_ONLY=1']
        if args.forest or args.short: command[command.index('--entrypoint'):command.index('--entrypoint')] = ['-e','IPC_FOREST=1']
        app = subprocess.Popen(command,stdout=app_log,stderr=subprocess.STDOUT); started.append(names[2])
        while app.poll() is None:
            if (out/'need-hover').exists() and not (out/'hover.json').exists():
                sample = subprocess.check_output(['docker','exec',names[0],'/opt/px4-gazebo/bin/px4-listener',
                    '--instance','72','vehicle_attitude_setpoint','-n','1'],text=True,timeout=10)
                (out/'hover-uorb.txt').write_text(sample)
                match = re.search(r'thrust_body:\s*\[([^\]]+)\]', sample)
                if not match: raise RuntimeError('PX4 hover thrust unavailable')
                thrust = -float(match.group(1).split(',')[2])
                if not .1 < thrust < .85: raise RuntimeError('invalid measured hover thrust')
                temporary = out/'hover.tmp'
                temporary.write_text(json.dumps({'normalized_thrust':thrust,'source':'PX4 vehicle_attitude_setpoint before takeover'})+'\n')
                temporary.replace(out/'hover.json')
            if (out/'acro-established').exists() and not (out/'px4-rates-uorb.txt').exists():
                sample = subprocess.check_output(['docker','exec',names[0],'/opt/px4-gazebo/bin/px4-listener',
                    '--instance','72','vehicle_rates_setpoint','-n','3'],text=True,timeout=10)
                (out/'px4-rates-uorb.txt').write_text(sample)
                mode = subprocess.check_output(['docker','exec',names[0],'/opt/px4-gazebo/bin/px4-listener',
                    '--instance','72','vehicle_control_mode','-n','1'],text=True,timeout=10)
                (out/'px4-control-mode-uorb.txt').write_text(mode)
            time.sleep(.1)
        if app.returncode: raise RuntimeError('Control test failed; inspect result.json/application.log')
        if args.native and not args.ground_only:
            call([sys.executable,APP/'analyze.py',out])
    finally:
        # Preserve control/clock/simulation when landing has not been confirmed.
        result = json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {}
        safe = app is None or result.get('landed_and_disarmed',False)
        if not safe:
            print('Recovery unconfirmed: retaining owned resources at '+str(out),file=sys.stderr)
        else:
            for name in started:
                if subprocess.run(['docker','inspect',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode == 0:
                    with (out/(name+'.log')).open('w') as stream:
                        subprocess.run(['docker','logs',name],stdout=stream,stderr=subprocess.STDOUT)
                    subprocess.run(['docker','stop','--time','10',name],stdout=subprocess.DEVNULL)
            if scene_started: call(scene('stop'))
            if relay is not None and relay.poll() is None:
                os.killpg(relay.pid,signal.SIGTERM); relay.wait(timeout=5)
            (out/'cleanup.json').write_text(json.dumps({'confirmed_disarmed':result.get('landed_and_disarmed'),
                'owned_resources_stopped':True})+'\n')
            if args.native and not args.ground_only:
                mapping=json.loads((out/'analysis.json').read_text()) if (out/'analysis.json').exists() else {}
                (out/'acceptance.json').write_text(json.dumps({'success':bool(result.get('success') and mapping.get('success')),
                    'navigation_passed':bool(result.get('success')),'control_mapping_passed':bool(mapping.get('success')),
                    'landed_and_disarmed':bool(result.get('landed_and_disarmed'))},indent=2)+'\n')

if __name__ == '__main__': main()
