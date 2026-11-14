#!/usr/bin/env python3
"""Own the visible fixed-image AirSim/PX4/FAST-LIVO2/AS2 acceptance deployment."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'deployment/airsim/demos/catora_open_field'))
from sim_startup import StartupRelay

PREFIX = 'aerohub-fast-livo2-'
NAMES = [PREFIX + name for name in (
    'vehicle', 'clock', 'sensors', 'algorithm', 'as2', 'mission', 'preflight',
    'python-source')]
BOARD = 'flight_board:v0.1.0'
ONBOARD = 'onboard_computer:v0.1.0'
WARMUP_SECONDS = float(os.environ.get('FAST_LIVO_WARMUP_SECONDS', '0'))
if not 0 <= WARMUP_SECONDS <= 30:
    raise ValueError('FAST_LIVO_WARMUP_SECONDS must be between 0 and 30')
STARTUP_RELAY_DELAY = float(os.environ.get('PX4_STARTUP_RELAY_DELAY', '0.1'))
if not 0.1 <= STARTUP_RELAY_DELAY <= 2.0:
    raise ValueError('PX4_STARTUP_RELAY_DELAY must be between 0.1 and 2.0 seconds')


def host_path(variable, default):
    path = Path(os.environ.get(variable, default))
    return (path if path.is_absolute() else ROOT / path).resolve()


CONFIG_HOST = host_path('FAST_LIVO_CONFIG_HOST', 'deployment/fast_livo2/airsim.yaml')
CAMERA_HOST = host_path('FAST_LIVO_CAMERA_HOST', 'deployment/fast_livo2/camera_airsim.yaml')
MISSION_HOST = host_path('FAST_LIVO_MISSION_HOST', 'deployment/fast_livo2/airsim_mission.py')
AS2_RUN_HOST = host_path('FAST_LIVO_AS2_RUN_HOST', 'deployment/airsim/as2/run.py')
SCOPED_TF_HOST = host_path(
    'FAST_LIVO_SCOPED_TF_HOST', 'deployment/airsim/as2/scoped_tf.launch.py')
ESTIMATOR_HOST = host_path(
    'FAST_LIVO_ESTIMATOR_HOST', 'deployment/fast_livo2/plugin.params.yaml')
RPC_HOST = host_path(
    'FAST_LIVO_RPC_HOST', 'simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py')
RUN_ALGORITHM_HOST = host_path(
    'FAST_LIVO_RUN_ALGORITHM_HOST', 'deployment/fast_livo2/run_algorithm.sh')
FAST_WORKSPACE = host_path(
    'FAST_LIVO2_WORKSPACE', 'log/workspaces/fast_livo2-script-validation-01')
SENSOR_WORKSPACE = host_path(
    'AIRSIM_SENSOR_WORKSPACE', 'log/workspaces/airsim-sensors-fast-livo2-validation-01')
PX4_WORKSPACE = host_path(
    'PX4_MSGS_OVERLAY_WORKSPACE', 'log/workspaces/px4_msgs-61907962-script-validation-02')
AS2_WORKSPACE = host_path(
    'AS2_EXECUTION_WORKSPACE', 'log/workspaces/as2-execution-fast-livo2-03')
WORKSPACES = {
    'fast_livo2': FAST_WORKSPACE,
    'airsim_sensors': SENSOR_WORKSPACE,
    'px4_msgs': PX4_WORKSPACE,
    'as2_execution': AS2_WORKSPACE,
}


def workspace_state_file(name):
    filename = 'source-state.env' if name == 'px4_msgs' else 'source-state.txt'
    return WORKSPACES[name] / filename


def execute(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def manager(action, output):
    return [sys.executable, ROOT / 'simulator/airsim/manager/manager.py', action,
            '--profile', ROOT / 'simulator/airsim/scenarios/fast_livo2/profile.json',
            '--run-dir', output / 'scene']


def container_exists(name):
    return subprocess.run(
        ['docker', 'container', 'inspect', name], stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL).returncode == 0


def cleanup(output):
    for name in reversed(NAMES):
        if not container_exists(name):
            continue
        subprocess.run(
            ['docker', 'stop', '--timeout', '10', name], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        with (output / (name + '.log')).open('w') as stream:
            subprocess.run(['docker', 'logs', name], stdout=stream, stderr=subprocess.STDOUT)
        with (output / (name + '.inspect.json')).open('w') as stream:
            subprocess.run(['docker', 'inspect', name], stdout=stream, stderr=subprocess.STDOUT)
        subprocess.run(
            ['docker', 'rm', '--force', name], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
    subprocess.run(manager('stop', output), check=False)


def wait_px4(output, attempt):
    started = time.monotonic()
    deadline = started + float(os.environ.get('PX4_STARTUP_TIMEOUT', '60'))
    while True:
        log = subprocess.check_output(
            ['docker', 'logs', PREFIX + 'vehicle'], stderr=subprocess.STDOUT, text=True)
        startup_complete = 'Startup script returned successfully' in log
        ready = 'Ready for takeoff!' in log
        if ready:
            record = {
                'ready': ready,
                'startup_complete': startup_complete,
                'external_vision_required_for_preflight': False,
                'initial_tcp_buffer_seconds': STARTUP_RELAY_DELAY,
                'tcp_nodelay': True,
                'startup_wall_seconds': time.monotonic() - started,
                'attempt': attempt,
            }
            (output / f'px4-startup-attempt-{attempt}.json').write_text(
                json.dumps(record, indent=2) + '\n')
            return record
        if time.monotonic() > deadline:
            raise RuntimeError('PX4 preflight did not become ready')
        time.sleep(0.25)


def probe_px4_preflight(output, attempt):
    name = PREFIX + 'preflight'
    command = [
        'docker', 'run', '--rm', '--init', '--network', 'host', '--name', name,
        '--env', 'ROS_DOMAIN_ID=96',
        '--volume', f'{PX4_WORKSPACE}:/px4-overlay:ro',
        '--volume',
        f'{ROOT / "deployment/fast_livo2/preflight_probe.py"}:/probe.py:ro',
        '--entrypoint', 'bash', ONBOARD, '-lc',
        'source /opt/ros/humble/setup.bash && '
        'source /opt/px4_ws/install/setup.bash && '
        'source /px4-overlay/install/setup.bash && '
        'exec python3 /probe.py --timeout 15 --stable-seconds 2',
    ]
    completed = subprocess.run(command, text=True, capture_output=True)
    evidence = {
        'attempt': attempt,
        'returncode': completed.returncode,
        'stdout': completed.stdout,
        'stderr': completed.stderr,
    }
    (output / f'px4-preflight-attempt-{attempt}.json').write_text(
        json.dumps(evidence, indent=2) + '\n')
    return completed.returncode == 0


def validate_workspaces():
    required = {
        'fast_livo2': FAST_WORKSPACE / 'install/fast_livo/lib/fast_livo/fastlivo_mapping',
        'airsim_sensors': SENSOR_WORKSPACE / (
            'install/lib/uav_as2_sensor_adapter/airsim_sensor_node'),
        'px4_msgs': PX4_WORKSPACE / 'install/setup.bash',
        'as2_execution': AS2_WORKSPACE / (
            'install/lib/as2_platform_pixhawk/as2_platform_pixhawk_node'),
    }
    for name, artifact in required.items():
        if not artifact.exists():
            raise RuntimeError(f'{name} overlay artifact is missing: {artifact}')
        if not workspace_state_file(name).is_file():
            raise RuntimeError(f'{name} overlay source manifest is missing')


def source_manifest(output):
    paths = [
        ROOT / 'deployment/fast_livo2',
        ROOT / 'deployment/flight_board/prepare_sensor_overlay.sh',
        ROOT / 'deployment/onboard_computer/prepare_execution_overlay.sh',
        ROOT / 'deployment/airsim/as2/run.py',
        ROOT / 'deployment/airsim/as2/scoped_tf.launch.py',
        ROOT / 'src/basic_function/state_estimator/fast_livo2/plugin',
        ROOT / 'src/basic_function/state_estimator/fast_livo2/ros2_adapter',
        ROOT / 'src/basic_function/perception/fast_livo2',
        ROOT / 'simulator/airsim/scenarios/fast_livo2',
        ROOT / 'simulator/airsim/sensors',
    ]
    files = []
    for path in paths:
        files.extend(path.rglob('*') if path.is_dir() else [path])
    hashes = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files
        if path.is_file() and '__pycache__' not in path.parts
    }
    workspace_state = {}
    for name, path in WORKSPACES.items():
        state = workspace_state_file(name)
        workspace_state[name] = {
            'path': str(path),
            'source_state_sha256': hashlib.sha256(state.read_bytes()).hexdigest(),
            'source_state': state.read_text().splitlines(),
        }
    manifest = {
        'commit': subprocess.check_output(
            ['git', '-C', ROOT, 'rev-parse', 'HEAD'], text=True).strip(),
        'fast_livo2_commit': subprocess.check_output(
            ['git', '-C', ROOT / 'src/basic_function/state_estimator/fast_livo2/ros2_upstream',
             'rev-parse', 'HEAD'], text=True).strip(),
        'dirty': bool(subprocess.check_output(['git', '-C', ROOT, 'status', '--porcelain'])),
        'ros_domain': 96,
        'render': {'display': os.environ.get('DISPLAY'),
                   'mode': os.environ.get('AIRSIM_RENDER_MODE')},
        'fast_livo_stationary_warmup_seconds': WARMUP_SECONDS,
        'fast_livo_config': {
            'path': str(CONFIG_HOST),
            'sha256': hashlib.sha256(CONFIG_HOST.read_bytes()).hexdigest(),
        },
        'fast_livo_camera': {
            'path': str(CAMERA_HOST),
            'sha256': hashlib.sha256(CAMERA_HOST.read_bytes()).hexdigest(),
        },
        'fast_livo_mission': {
            'path': str(MISSION_HOST),
            'sha256': hashlib.sha256(MISSION_HOST.read_bytes()).hexdigest(),
        },
        'as2_run': {
            'path': str(AS2_RUN_HOST),
            'sha256': hashlib.sha256(AS2_RUN_HOST.read_bytes()).hexdigest(),
        },
        'ports': {'airsim_rpc': 44471, 'airsim_tcp': 4980,
                  'px4_relay': 5080, 'agent': 8961},
        'workspaces': workspace_state,
        'source_hashes': hashes,
    }
    (output / 'source-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


def extract_msgpack(output):
    target = output / 'python-overlay'
    target.mkdir()
    name = PREFIX + 'python-source'
    execute(['docker', 'create', '--name', name, BOARD], stdout=subprocess.DEVNULL)
    try:
        execute(['docker', 'cp', name + ':/usr/lib/python3/dist-packages/msgpack', target])
    finally:
        subprocess.run(['docker', 'rm', '--force', name], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    if not (target / 'msgpack/__init__.py').is_file():
        raise RuntimeError('fixed flight-board image did not provide the msgpack runtime')
    return target


def run_detached(name, image, environment, volumes, command):
    args = ['docker', 'run', '--detach', '--init', '--network', 'host', '--name', name]
    for key, value in environment.items():
        args += ['--env', f'{key}={value}']
    for source, target, mode in volumes:
        args += ['--volume', f'{source}:{target}:{mode}']
    args += ['--entrypoint', 'bash', image, '-lc', command]
    execute(args, stdout=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['run', 'stop', 'status'])
    parser.add_argument(
        '--scenario',
        choices=['shadow', 'authority', 'sensor_fault', 'algorithm_fault'],
        default='authority')
    parser.add_argument('--output', type=Path, default=ROOT / 'log/airsim/fast-livo2')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.action == 'stop':
        cleanup(output)
        return
    if args.action == 'status':
        subprocess.run(manager('status', output), check=False)
        execute(['docker', 'ps', '--filter', 'name=' + PREFIX])
        return
    if os.environ.get('DISPLAY') != ':3' or os.environ.get('AIRSIM_RENDER_MODE') != 'windowed':
        raise RuntimeError('acceptance requires DISPLAY=:3 and AIRSIM_RENDER_MODE=windowed')
    if any((output / name).exists() for name in ('result.json', 'failure.json', 'images.json')):
        raise RuntimeError('choose a fresh output directory')
    for name in NAMES:
        if container_exists(name):
            raise RuntimeError('reserved container exists: ' + name)
    for image in (BOARD, ONBOARD):
        execute(['docker', 'image', 'inspect', image], stdout=subprocess.DEVNULL)
    validate_workspaces()
    source_manifest(output)
    msgpack_overlay = extract_msgpack(output)
    algorithm_debug = output / 'algorithm-debug'
    (algorithm_debug / 'result').mkdir(parents=True)
    settings = json.loads(
        (ROOT / 'simulator/airsim/scenarios/fast_livo2/settings.json').read_text())
    px4_settings = json.loads(json.dumps(settings))
    px4_settings['Vehicles']['PX4']['TcpPort'] = 5080
    (output / 'px4-settings.json').write_text(json.dumps(px4_settings, indent=2) + '\n')
    (output / 'fleet.yaml').write_text(yaml.safe_dump({'vehicles': {
        'drone0': {'fmu_prefix': '/drone0', 'target_system_id': 71},
    }}))
    identities = {
        name: subprocess.check_output(
            ['docker', 'image', 'inspect', '--format', '{{.Id}}', image], text=True).strip()
        for name, image in (('flight_board', BOARD), ('onboard_computer', ONBOARD))
    }
    (output / 'images.json').write_text(json.dumps(identities, indent=2) + '\n')
    relay = None
    try:
        execute(manager('start', output) + ['--timeout', '90'])
        state = json.loads((output / 'scene/state.json').read_text())
        pid = state['pid']
        environment = Path(f'/proc/{pid}/environ').read_bytes().replace(
            b'\0', b'\n').decode(errors='replace')
        window = subprocess.check_output(
            ['bash', '-lc', 'DISPLAY=${DISPLAY:?} xwininfo -root -tree | grep -i AirSimNH'],
            text=True)
        (output / 'window-evidence.txt').write_text(
            'command=' + json.dumps(state['command']) + '\n' + environment + window)
        relay = StartupRelay(
            ('127.0.0.1', 5080), ('127.0.0.1', 4980), delay=STARTUP_RELAY_DELAY)

        vehicle_environment = {
            'ROS_DOMAIN_ID': '96', 'VEHICLE_NAME': 'PX4',
            'VEHICLE_NAMESPACE': 'drone0', 'PX4_INSTANCE': '70',
            'PX4_SYSTEM_ID': '71', 'XRCE_AGENT_PORT': '8961',
            'AIRSIM_RPC_PORT': '44471', 'FLIGHT_BOARD_SENSORS': '0',
            'PX4_PARAM_COM_ARM_WO_GPS': '1', 'PX4_PARAM_CBRK_SUPPLY_CHK': '894281',
        }
        vehicle_args = [
            'docker', 'run', '--detach', '--init', '--network', 'host',
            '--name', PREFIX + 'vehicle',
        ]
        for key, value in vehicle_environment.items():
            vehicle_args += ['--env', f'{key}={value}']
        vehicle_args += [
            '--volume', str(output / 'px4-settings.json') + ':/config/settings.json:ro',
            BOARD, 'vehicle',
        ]
        execute([
            'docker', 'run', '--detach', '--init', '--network', 'host',
            '--name', PREFIX + 'clock', '--env', 'ROS_DOMAIN_ID=96',
            '--env', 'VEHICLE_NAME=PX4', '--env', 'AIRSIM_RPC_PORT=44471',
            BOARD, 'clock',
        ], stdout=subprocess.DEVNULL)
        execute(vehicle_args, stdout=subprocess.DEVNULL)
        startup = wait_px4(output, 1)
        preflight_ready = probe_px4_preflight(output, 1)
        startup['preflight_ready'] = preflight_ready
        if not preflight_ready:
            # AirSim 1.8.1 lockstep exits when this PX4 TCP peer disconnects;
            # a valid retry must restart the complete AirSim/PX4 run.
            raise RuntimeError('PX4 preflight did not stabilize in this complete run')
        (output / 'px4-startup.json').write_text(json.dumps({
            'ready': True,
            'preflight_ready': True,
            'successful_attempt': startup['attempt'],
            'attempts': [startup],
            'external_vision_required_for_preflight': False,
            'initial_tcp_buffer_seconds': STARTUP_RELAY_DELAY,
            'tcp_nodelay': True,
        }, indent=2) + '\n')

        sensor_command = (
            'source /opt/ros/humble/setup.bash && '
            'source /opt/platform_sensors/install/setup.bash && '
            'source /sensor-overlay/install/setup.bash && '
            'exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args '
            '-p use_sim_time:=true -p settings_file:=/config/settings.json '
            '-p vehicle_name:=PX4 -p vehicle_namespace:=drone0 '
            '-p rpc_host:=127.0.0.1 -p rpc_port:=44471 '
            '-p camera_rate_hz:=5.0 -p imu_rate_hz:=100.0 -p lidar_rate_hz:=10.0')
        run_detached(
            PREFIX + 'sensors', BOARD, {'ROS_DOMAIN_ID': '96'},
            [(SENSOR_WORKSPACE, '/sensor-overlay', 'ro'),
             (output / 'px4-settings.json', '/config/settings.json', 'ro')],
            sensor_command)

        run_detached(
            PREFIX + 'algorithm', ONBOARD, {'ROS_DOMAIN_ID': '96'},
            [(FAST_WORKSPACE, '/workspace', 'ro'),
             (algorithm_debug, '/workspace/src/fast_livo/Log', 'rw'),
             (CONFIG_HOST, '/config/airsim.yaml', 'ro'),
             (CAMERA_HOST, '/config/camera.yaml', 'ro'),
             (RUN_ALGORITHM_HOST, '/opt/aerohub/run_fast_livo2.sh', 'ro')],
            'exec /opt/aerohub/run_fast_livo2.sh')

        as2_environment = {
            'ROS_DOMAIN_ID': '96', 'AS2_FLEET_FILE': '/results/fleet.yaml',
            'AS2_TF_SCOPE': 'per-vehicle', 'AS2_EXTERNAL_ODOM': '0',
        }
        if args.scenario != 'shadow':
            as2_environment.update({
                'AS2_STATE_ESTIMATOR_PLUGIN': 'fast_livo2',
                'AS2_STATE_ESTIMATOR_CONFIG_FILE': '/config/plugin.yaml',
                'AS2_STATE_ESTIMATOR_LAUNCH_PACKAGE': 'uav_fast_livo2',
                'AS2_STATE_ESTIMATOR_LAUNCH_FILE': 'external_state_estimator.launch.py',
            })
        as2_command = (
            'source /opt/ros/humble/setup.bash && '
            'source /opt/px4_ws/install/setup.bash && '
            'source /root/aerostack2_ws/install/setup.bash && '
            'source /fast-livo-overlay/install/setup.bash && '
            'source /px4-overlay/install/setup.bash && '
            'source /as2-overlay/install/setup.bash && '
            'exec python3 /opt/aerohub/run_as2.py')
        run_detached(
            PREFIX + 'as2', ONBOARD, as2_environment,
            [(output, '/results', 'rw'), (PX4_WORKSPACE, '/px4-overlay', 'ro'),
             (AS2_WORKSPACE, '/as2-overlay', 'ro'),
             (FAST_WORKSPACE, '/fast-livo-overlay', 'ro'),
             (AS2_RUN_HOST, '/opt/aerohub/run_as2.py', 'ro'),
             (SCOPED_TF_HOST, '/opt/aerohub/scoped_tf.launch.py', 'ro'),
             (ESTIMATOR_HOST, '/config/plugin.yaml', 'ro')],
            as2_command)

        mission_environment = {
            'ROS_DOMAIN_ID': '96', 'FAST_LIVO_RESULTS': '/results',
            'AIRSIM_RPC_PORT': '44471', 'FAST_LIVO_SCENARIO': args.scenario,
            'FAST_LIVO_WARMUP_SECONDS': f'{WARMUP_SECONDS:g}',
        }
        mission_command = (
            'source /opt/ros/humble/setup.bash && '
            'source /opt/px4_ws/install/setup.bash && '
            'source /root/aerostack2_ws/install/setup.bash && '
            'source /px4-overlay/install/setup.bash && '
            'source /as2-overlay/install/setup.bash && '
            'PYTHONPATH=/tmp/aerohub:/opt/flight-board-python:${PYTHONPATH:-} '
            'exec python3 /tmp/aerohub/airsim_mission.py')
        mission_args = [
            'docker', 'run', '--init', '--network', 'host', '--name', PREFIX + 'mission',
        ]
        for key, value in mission_environment.items():
            mission_args += ['--env', f'{key}={value}']
        for source, target, mode in (
                (output, '/results', 'rw'), (PX4_WORKSPACE, '/px4-overlay', 'ro'),
                (AS2_WORKSPACE, '/as2-overlay', 'ro'),
                (msgpack_overlay, '/opt/flight-board-python', 'ro'),
                (MISSION_HOST, '/tmp/aerohub/airsim_mission.py', 'ro'),
                (RPC_HOST, '/tmp/aerohub/airsim_rpc.py', 'ro')):
            mission_args += ['--volume', f'{source}:{target}:{mode}']
        mission_args += ['--entrypoint', 'bash', ONBOARD, '-lc', mission_command]
        injector = None
        injector_errors = []
        if args.scenario in ('sensor_fault', 'algorithm_fault'):
            target = PREFIX + ('sensors' if args.scenario == 'sensor_fault' else 'algorithm')

            def inject_fault():
                marker = output / 'fault-ready.json'
                deadline = time.monotonic() + 180.0
                while not marker.is_file():
                    if time.monotonic() >= deadline:
                        injector_errors.append('timed out waiting for fault-ready.json')
                        return
                    time.sleep(0.1)
                command = ['docker', 'stop', '--timeout', '1', target]
                started = time.time()
                completed = subprocess.run(command, text=True, capture_output=True)
                evidence = {
                    'scenario': args.scenario,
                    'target_container': target,
                    'command': command,
                    'started_wall_time': started,
                    'completed_wall_time': time.time(),
                    'returncode': completed.returncode,
                    'stdout': completed.stdout,
                    'stderr': completed.stderr,
                }
                (output / 'fault-injection.json').write_text(
                    json.dumps(evidence, indent=2) + '\n')
                if completed.returncode:
                    injector_errors.append(
                        f'fault injection failed for {target}: {completed.stderr.strip()}')

            injector = threading.Thread(target=inject_fault, daemon=True)
            injector.start()
        with (output / 'mission-console.log').open('w') as stream:
            execute(mission_args, stdout=stream, stderr=subprocess.STDOUT)
        if injector:
            injector.join(timeout=5.0)
            if injector.is_alive():
                raise RuntimeError('fault injector did not finish')
            if injector_errors:
                raise RuntimeError('; '.join(injector_errors))
        result = json.loads((output / 'result.json').read_text())
        if not result.get('success'):
            raise RuntimeError('FAST-LIVO2 mission did not pass')
    finally:
        cleanup(output)
        if relay:
            relay.close()


if __name__ == '__main__':
    main()
