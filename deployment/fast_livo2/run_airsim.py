#!/usr/bin/env python3
"""Own the visible AirSim/PX4/FAST-LIVO2/AS2 acceptance deployment."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'deployment/airsim/demos/catora_open_field'))
from sim_startup import StartupRelay

PREFIX = 'aerohub-fast-livo2-'
NAMES = [PREFIX+name for name in ('vehicle','clock','algorithm','app')]
BOARD = 'flight_board:v0.1.0'
ALGORITHM = os.environ.get('FAST_LIVO_ALGORITHM_IMAGE', 'uav/fast-livo2:airsim')
APPLICATION = os.environ.get('FAST_LIVO_AS2_IMAGE', 'uav/fast-livo2-as2:airsim')
ODOMETRY_TOPIC = os.environ.get('FAST_LIVO_ODOM_TOPIC', '/LIVO2/imu_propagate')
if ODOMETRY_TOPIC not in ('/LIVO2/imu_propagate', '/aft_mapped_to_init'):
    raise ValueError('FAST_LIVO_ODOM_TOPIC must select an actual FAST-LIVO2 odometry output')
WARMUP_SECONDS = float(os.environ.get('FAST_LIVO_WARMUP_SECONDS', '0'))
if not 0 <= WARMUP_SECONDS <= 30:
    raise ValueError('FAST_LIVO_WARMUP_SECONDS must be between 0 and 30')


def execute(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def manager(action, output):
    return [sys.executable, ROOT/'simulator/manager/manager.py', action,
            '--profile', ROOT/'simulator/airsim/scenarios/fast_livo2/profile.json',
            '--run-dir', output/'scene']


def cleanup(output):
    for name in NAMES:
        if subprocess.run(['docker','container','inspect',name], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode:
            continue
        with (output/(name+'.log')).open('w') as stream:
            subprocess.run(['docker','logs',name], stdout=stream, stderr=subprocess.STDOUT)
        subprocess.run(['docker','stop','--timeout','10',name], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    subprocess.run(manager('stop', output), check=False)


def wait_px4(output):
    started = time.monotonic(); deadline = started + float(os.environ.get('PX4_STARTUP_TIMEOUT','60'))
    while True:
        log = subprocess.check_output(['docker','logs',PREFIX+'vehicle'], stderr=subprocess.STDOUT, text=True)
        if 'Ready for takeoff!' in log:
            (output/'px4-startup.json').write_text(json.dumps({
                'ready': True, 'initial_tcp_buffer_seconds': 0.1, 'tcp_nodelay': True,
                'startup_wall_seconds': time.monotonic()-started}, indent=2)+'\n')
            return
        if time.monotonic() > deadline:
            raise RuntimeError('PX4 preflight did not become ready')
        time.sleep(0.25)


def source_manifest(output):
    paths = [ROOT/'deployment/fast_livo2', ROOT/'src/state_estimator/fast_livo2/plugin',
             ROOT/'src/state_estimator/fast_livo2/ros1_proxy', ROOT/'src/perception/fast_livo2',
             ROOT/'simulator/airsim/scenarios/fast_livo2', ROOT/'deployment/airsim/as2/run.py']
    files = []
    for path in paths:
        files.extend(path.rglob('*') if path.is_dir() else [path])
    hashes = {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
              for path in files if path.is_file() and '__pycache__' not in path.parts}
    manifest = {
        'commit': subprocess.check_output(['git','-C',ROOT,'rev-parse','HEAD'], text=True).strip(),
        'fast_livo2_commit': subprocess.check_output(
            ['git','-C',ROOT/'src/state_estimator/fast_livo2/upstream','rev-parse','HEAD'], text=True).strip(),
        'dirty': bool(subprocess.check_output(['git','-C',ROOT,'status','--porcelain'])),
        'ros_domain': 96, 'ros1_master': 11331,
        'fast_livo_odom_topic': ODOMETRY_TOPIC,
        'fast_livo_stationary_warmup_seconds': WARMUP_SECONDS,
        'ports': {'airsim_rpc':44471,'airsim_tcp':4980,'px4_relay':5080,'agent':8961},
        'source_hashes': hashes,
    }
    (output/'source-manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['run','stop','status'])
    parser.add_argument('--scenario', choices=['shadow','authority'], default='authority')
    parser.add_argument('--output', type=Path, default=ROOT/'log/airsim/fast-livo2')
    args = parser.parse_args(); output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    if args.action == 'stop': cleanup(output); return
    if args.action == 'status':
        subprocess.run(manager('status', output), check=False)
        execute(['docker','ps','--filter','name='+PREFIX]); return
    if any((output/name).exists() for name in ('result.json','failure.json','images.json')):
        raise RuntimeError('choose a fresh output directory')
    for name in NAMES:
        if subprocess.run(['docker','container','inspect',name], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError('reserved container exists: '+name)
    for image in (BOARD,ALGORITHM,APPLICATION):
        execute(['docker','image','inspect',image], stdout=subprocess.DEVNULL)
    source_manifest(output)
    algorithm_debug = output/'algorithm-debug'
    (algorithm_debug/'result').mkdir(parents=True)
    settings = json.loads((ROOT/'simulator/airsim/scenarios/fast_livo2/settings.json').read_text())
    px4_settings = json.loads(json.dumps(settings)); px4_settings['Vehicles']['PX4']['TcpPort'] = 5080
    (output/'px4-settings.json').write_text(json.dumps(px4_settings, indent=2)+'\n')
    (output/'fleet.yaml').write_text(yaml.safe_dump({'vehicles':{
        'drone0':{'fmu_prefix':'/drone0','target_system_id':71}}}))
    relay = None
    try:
        execute(manager('start',output)+['--timeout','90'])
        state = json.loads((output/'scene/state.json').read_text())
        pid = state['pid']
        environment = Path(f'/proc/{pid}/environ').read_bytes().replace(b'\0',b'\n').decode(errors='replace')
        window = subprocess.check_output(
            ['bash','-lc','DISPLAY=${DISPLAY:?} xwininfo -root -tree | grep -i AirSimNH'], text=True)
        (output/'window-evidence.txt').write_text(
            'command='+json.dumps(state['command'])+'\n'+environment+window)
        relay = StartupRelay(('127.0.0.1',5080),('127.0.0.1',4980))
        execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'vehicle',
                 '-e','ROS_DOMAIN_ID=96','-e','VEHICLE_NAME=PX4','-e','VEHICLE_NAMESPACE=drone0',
                 '-e','PX4_INSTANCE=70','-e','PX4_SYSTEM_ID=71','-e','XRCE_AGENT_PORT=8961',
                 '-e','AIRSIM_RPC_PORT=44471','-e','FLIGHT_BOARD_SENSORS=0',
                 '-v',str(output/'px4-settings.json')+':/config/settings.json:ro',BOARD,'vehicle'],
                stdout=subprocess.DEVNULL)
        execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'clock',
                 '-e','ROS_DOMAIN_ID=96','-e','VEHICLE_NAME=PX4','-e','AIRSIM_RPC_PORT=44471',BOARD,'clock'],
                stdout=subprocess.DEVNULL)
        wait_px4(output)
        execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'algorithm',
                 '-e','ROS_MASTER_URI=http://127.0.0.1:11331','-e','ROS_IP=127.0.0.1',
                 '-e','AIRSIM_RPC_PORT=44471','-e','AIRSIM_VEHICLE=PX4','-e','FAST_LIVO_RESULTS=/results',
                 '-v',str(algorithm_debug)+':/opt/fast_livo_ws/src/fast_livo/Log',
                 '-v',str(output)+':/results',ALGORITHM,'bash','-lc',
                 'source /opt/fast_livo_ws/devel/setup.bash && exec roslaunch -p 11331 fast_livo as2_mapping.launch '
                 'config:=/opt/fast_livo_ws/src/fast_livo/config/airsim.yaml '
                 'camera:=/opt/fast_livo_ws/src/fast_livo/config/camera_airsim.yaml '
                 f'odom_topic:={ODOMETRY_TOPIC} use_sim_time:=true airsim:=true'],
                stdout=subprocess.DEVNULL)
        # Ensure ROS 1 output streams exist before the ROS 2 bridge begins reconnecting.
        deadline=time.monotonic()+30
        while True:
            probe=subprocess.run(['bash','-lc','exec 3<>/dev/tcp/127.0.0.1/18761'],
                                 stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            if probe.returncode==0:break
            if time.monotonic()>deadline:raise RuntimeError('FAST-LIVO2 output bridge did not listen')
            time.sleep(.25)
        identities={name:subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
                    for name,image in (('flight_board',BOARD),('algorithm',ALGORITHM),('application',APPLICATION))}
        (output/'images.json').write_text(json.dumps(identities,indent=2)+'\n')
        app_environment = [
            '-e','ROS_DOMAIN_ID=96','-e','FAST_LIVO_RESULTS=/results','-e','AIRSIM_RPC_PORT=44471',
            '-e','AS2_FLEET_FILE=/results/fleet.yaml','-e','FAST_LIVO_SCENARIO='+args.scenario,
            '-e',f'FAST_LIVO_WARMUP_SECONDS={WARMUP_SECONDS:g}',
            '-e','AS2_TF_SCOPE=per-vehicle']
        if args.scenario == 'authority':
            app_environment += [
                '-e','AS2_STATE_ESTIMATOR_PLUGIN=fast_livo2',
                '-e','AS2_STATE_ESTIMATOR_CONFIG_FILE=/opt/fast_livo2_as2_ws/install/share/uav_fast_livo2/config/estimator.yaml',
                '-e','AS2_STATE_ESTIMATOR_LAUNCH_PACKAGE=uav_fast_livo2',
                '-e','AS2_STATE_ESTIMATOR_LAUNCH_FILE=external_state_estimator.launch.py']
        execute(['docker','run','--rm','--init','--network','host','--name',PREFIX+'app'] +
                app_environment + ['-v',str(output)+':/results',APPLICATION,
                 '/opt/fast_livo2_as2_ws/airsim_entrypoint.sh'])
        result=json.loads((output/'result.json').read_text())
        if not result.get('success'):raise RuntimeError('FAST-LIVO2 mission did not pass')
    finally:
        cleanup(output)
        if relay: relay.close()


if __name__ == '__main__':
    main()
