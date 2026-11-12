"""Isolated GCOPTER PC simulation; uses existing images and external overlays."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import hashlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'deployment/airsim/demos/catora_open_field'))
from sim_startup import StartupRelay


def run(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=('short', 'cancel', 'map-reset', 'local', 'forest'), default='local')
    args = parser.parse_args()
    occupied_domains = set()
    container_ids = subprocess.check_output(['docker', 'ps', '-q'], text=True).split()
    if container_ids:
        containers = json.loads(subprocess.check_output(['docker', 'inspect', *container_ids], text=True))
        for container in containers:
            for value in container['Config'].get('Env', []):
                if value.startswith('ROS_DOMAIN_ID='):
                    occupied_domains.add(int(value.split('=', 1)[1]))
    domain = next(value for value in range(190, 220) if value not in occupied_domains)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    settings = json.loads((ROOT/'simulator/airsim/scenarios/super_forest/settings.json').read_text())
    settings['ApiServerPort'] = 43482
    settings['Vehicles']['PX4'].update(TcpPort=4891, ControlPortLocal=15741, ControlPortRemote=15781)
    (out/'settings.json').write_text(json.dumps(settings, indent=2)+'\n')
    px4_settings = json.loads(json.dumps(settings))
    px4_settings['Vehicles']['PX4']['TcpPort'] = 4991
    (out/'px4-settings.json').write_text(json.dumps(px4_settings)+'\n')
    profile = {'binary': str(args.binary.resolve()), 'settings': str(out/'settings.json'),
        'overhead_pose_source': 'object',
        'project': 'AirSimNH', 'args': ['-vulkan', '-nosound', '-windowed', '-ResX=960', '-ResY=540']}
    (out/'profile.json').write_text(json.dumps(profile)+'\n')
    (out/'fleet.yaml').write_text('vehicles:\n  drone0:\n    fmu_prefix: /drone0\n    target_system_id: 71\n')
    manager = [sys.executable, ROOT/'simulator/airsim/manager/manager.py']
    names = ['aerohub-gcopter-vehicle', 'aerohub-gcopter-clock', 'aerohub-gcopter-compute', 'aerohub-gcopter-workstation']
    owned = []
    relay = None
    scene_started = False
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for directory in (ROOT/'src/planner/gcopter/ros2_adapter', ROOT/'deployment/gcopter')
        for path in directory.rglob('*') if path.is_file() and '__pycache__' not in path.parts}
    (out/'source-manifest.json').write_text(json.dumps({'source_hashes': hashes,
        'upstream': subprocess.check_output(['git', '-C', str(ROOT/'src/planner/gcopter/upstream'), 'rev-parse', 'HEAD'], text=True).strip(),
        'image_ids': {name: subprocess.check_output(['docker', 'image', 'inspect', '--format', '{{.Id}}', name], text=True).strip()
                      for name in ('flight_board:v0.1.0', 'uav/apps-super-forest:onboard-v0.1.1', 'osrf/ros:humble-desktop-full-jammy')},
        'overlay': 'log/gcopter/install',
        'artifact_hashes': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (ROOT/'log/gcopter/install/uav_gcopter_core/lib/uav_gcopter_core/gcopter_function',
                         ROOT/'log/gcopter/install/uav_gcopter_core/lib/uav_gcopter_core/gcopter_local_function',
                         ROOT/'log/gcopter/install/uav_gcopter_core/lib/libuav_gcopter_core.a',
                         ROOT/'log/gcopter/install/uav_gcopter_adapter/lib/python3.10/site-packages/uav_gcopter_adapter/node.py')},
        'ros_domain': domain, 'case': args.case, 'search_seed': 42 if args.case in ('local', 'forest') else 0,
        'rrt_timeout_s': .05 if args.case in ('local', 'forest') else .02,
        'criteria': {'arrival_m': 1.5 if args.case in ('local', 'forest') else .25, 'hold_s': 2,
        'speed_m_s': 4.04,
        'navigation_timeout_s': 300 if args.case == 'forest' else 90,
        'map_source': 'live-lidar-rog-map' if args.case in ('local', 'forest') else 'registered_lidar_snapshot',
        'collision_count': 0, 'stop_confirmation': True, 'land_disarmed': True}}, indent=2)+'\n')
    for name in names:
        if subprocess.run(['docker', 'inspect', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError('reserved container already exists: '+name)
    try:
        run(manager+['start', '--profile', out/'profile.json', '--run-dir', out/'scene', '--timeout', '90'],
            env=dict(os.environ, AIRSIM_RENDER_MODE='windowed'))
        scene_started = True
        relay = StartupRelay(('127.0.0.1', 4991), ('127.0.0.1', 4891))
        board = 'flight_board:v0.1.0'
        image = 'uav/apps-super-forest:onboard-v0.1.1'
        for name, mode in zip(names, ('vehicle', 'clock')):
            command = ['docker', 'run', '--rm', '--detach', '--init', '--network', 'host', '--name', name,
                '-e', 'ROS_DOMAIN_ID='+str(domain), '-e', 'VEHICLE_NAME=PX4', '-e', 'VEHICLE_NAMESPACE=drone0',
                '-e', 'PX4_INSTANCE=70', '-e', 'PX4_SYSTEM_ID=71', '-e', 'XRCE_AGENT_PORT=8961',
                '-e', 'AIRSIM_RPC_PORT=43482', '-e', 'FLIGHT_BOARD_SENSORS=1', '-e', 'FLIGHT_BOARD_CAMERAS=0',
                '-v', str(out/'px4-settings.json')+':/config/settings.json:ro', board, mode]
            run(command, stdout=subprocess.DEVNULL)
            owned.append(name)
        end = time.monotonic()+60
        while 'Ready for takeoff!' not in subprocess.check_output(['docker', 'logs', names[0]], stderr=subprocess.STDOUT, text=True):
            if time.monotonic() > end:
                raise TimeoutError('PX4 preflight')
            time.sleep(.25)
        command = ['docker', 'run', '--rm', '--detach', '--init', '--network', 'host', '--name', names[2],
            '-e', 'GCOPTER_CASE='+args.case,
            '-e', 'ROS_DOMAIN_ID='+str(domain), '-e', 'AS2_EXECUTION_BACKEND=follow_path',
            '-e', 'AS2_FLEET_FILE=/evidence/fleet.yaml', '-e', 'AIRSIM_RPC_PORT=43482',
            '-v', str(ROOT)+':/workspace', '-v', str(out)+':/evidence', '--entrypoint', 'bash', image, '-lc',
            'set -eo pipefail; source /root/aerostack2_ws/install/setup.bash; source /opt/super_app_ws/install/setup.bash; '
            'source /opt/uav_execution_ws/install/setup.bash; source /workspace/log/gcopter/install/setup.bash; '
            'for package in /workspace/log/gcopter/deps/*.deb; do dpkg-deb -x "$package" /; done; '
            'python3 /workspace/deployment/gcopter/compute_runtime.py']
        owned.append(names[2])
        run(command, stdout=subprocess.DEVNULL)
        command = ['docker', 'run', '--rm', '--init', '--network', 'host', '--name', names[3],
            '-e', 'GCOPTER_CASE='+args.case, '-e', 'ROS_DOMAIN_ID='+str(domain),
            '-v', str(ROOT)+':/workspace', '-v', str(out)+':/evidence', '--entrypoint', 'bash',
            'osrf/ros:humble-desktop-full-jammy', '-lc',
            'set -eo pipefail; source /opt/ros/humble/setup.bash; source /workspace/log/gcopter/install/setup.bash; '
            'for package in /workspace/log/gcopter/deps/*.deb; do dpkg-deb -x "$package" /; done; '
            'python3 /workspace/deployment/gcopter/mission.py']
        owned.append(names[3])
        with (out/'mission.log').open('w') as stream:
            run(command, stdout=stream, stderr=subprocess.STDOUT)
    finally:
        for name in reversed(owned):
            with (out/(name+'.log')).open('w') as stream:
                subprocess.run(['docker', 'logs', name], stdout=stream, stderr=subprocess.STDOUT)
            subprocess.run(['docker', 'stop', '--time', '10', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if relay:
            relay.close()
        if scene_started:
            run(manager+['stop', '--run-dir', out/'scene'])


if __name__ == '__main__':
    main()
