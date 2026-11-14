"""AS2-only multi-aircraft launcher; Agent and simulator live in flight_board."""
import os
import json
from pathlib import Path
import re
import signal
import subprocess
import time
import yaml


def main():
    # HDC children may inherit -1000. Keep this private runtime killable on OOM.
    if os.environ.get('AS2_RESET_OOM_SCORE') == '1':
        Path('/proc/self/oom_score_adj').write_text('0')
    memory_limit = int(os.environ.get('AS2_MAX_RSS_MB', '0')) * 1024
    config = os.environ.get('AS2_CONFIG_FILE', '/opt/onboard_computer/config.yaml')
    controller = os.environ.get('AS2_CONTROLLER_CONFIG_FILE', '/opt/onboard_computer/pid_speed_controller.yaml')
    estimator_plugin = os.environ.get('AS2_STATE_ESTIMATOR_PLUGIN', 'raw_odometry')
    estimator_config = os.environ.get('AS2_STATE_ESTIMATOR_CONFIG_FILE', config)
    estimator_launch_package = os.environ.get(
        'AS2_STATE_ESTIMATOR_LAUNCH_PACKAGE', 'as2_state_estimator')
    estimator_launch_file = os.environ.get(
        'AS2_STATE_ESTIMATOR_LAUNCH_FILE', 'state_estimator_launch.py')
    external_odom_value = os.environ.get('AS2_EXTERNAL_ODOM', '0')
    if external_odom_value not in ('0', '1'):
        raise ValueError('AS2_EXTERNAL_ODOM must be 0 or 1')
    external_odom = external_odom_value == '1'
    fleet = yaml.safe_load(Path(os.environ.get('AS2_FLEET_FILE', '/opt/onboard_computer/fleet.yaml')).read_text())
    vehicles = fleet['vehicles']
    if not vehicles:
        raise ValueError('fleet must contain at least one aircraft')
    commands = []
    runtime = Path(os.environ.get('AS2_RUNTIME_DIR', '/tmp/onboard_computer'))
    runtime.mkdir(exist_ok=True)
    for namespace, vehicle in vehicles.items():
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', namespace):
            raise ValueError('invalid vehicle namespace')
        system_id = int(vehicle['target_system_id'])
        if not 1 <= system_id <= 255:
            raise ValueError('invalid target_system_id')
        platform_config = runtime / (namespace + '.yaml')
        platform_config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'use_sim_time': True, 'cmd_freq': 50.0, 'info_freq': 10.0,
            'external_odom': external_odom, 'fmu_prefix': vehicle['fmu_prefix'],
            'target_system_id': system_id}}}))
        estimator_command = [
            'ros2', 'launch', estimator_launch_package, estimator_launch_file,
            f'namespace:={namespace}']
        estimator_command.extend([
            f'config_file:={config}', f'plugin_name:={estimator_plugin}',
            f'plugin_config_file:={estimator_config}', 'use_sim_time:=true'])
        commands.extend([
            ['ros2', 'launch', 'as2_platform_pixhawk', 'pixhawk_launch.py',
             f'namespace:={namespace}', f'platform_config_file:={platform_config}'],
            estimator_command,
            ['ros2', 'launch', 'as2_motion_controller', 'controller_launch.py',
             f'namespace:={namespace}', f'config_file:={config}', 'plugin_name:=pid_speed_controller',
             f'plugin_config_file:={controller}', 'use_sim_time:=true'],
            ['ros2', 'launch', 'as2_behaviors_motion', 'motion_behaviors_launch.py',
             f'namespace:={namespace}', f'config_file:={config}', 'use_sim_time:=true'],
        ])
    if os.environ.get('AS2_BEHAVIORS'):
        subset=str(Path(__file__).with_name('selected_behaviors.launch.py'))
        for command in commands:
            if command[1:4]==['launch','as2_behaviors_motion','motion_behaviors_launch.py']:
                command[2:4]=[subset]
    backend = os.environ.get('AS2_EXECUTION_BACKEND', 'legacy_setpoint')
    if backend not in ('follow_path', 'legacy_setpoint'):
        raise ValueError('invalid AS2_EXECUTION_BACKEND')
    if backend == 'follow_path':
        for command in commands:
            if command[1:4] == ['launch', 'as2_behaviors_motion', 'motion_behaviors_launch.py']:
                command[2:4] = ['uav_follow_path_execution', 'motion_behaviors.launch.py']
                tolerance = os.environ.get('AS2_TIMED_CONTINUITY_TOLERANCE')
                if tolerance:
                    value = float(tolerance)
                    if not 0.0 < value <= 1.0:
                        raise ValueError('AS2_TIMED_CONTINUITY_TOLERANCE must be in (0, 1]')
                    command.append(f'timed_continuity_tolerance:={value}')
                acceleration = os.environ.get('AS2_TIMED_ACCELERATION_CONTINUITY_TOLERANCE')
                if acceleration:
                    value = float(acceleration)
                    if not 0.0 < value <= 20.0:
                        raise ValueError('AS2_TIMED_ACCELERATION_CONTINUITY_TOLERANCE must be in (0, 20]')
                    command.append(f'timed_acceleration_continuity_tolerance:={value}')
            elif len(command) > 2 and command[2].endswith('selected_behaviors.launch.py'):
                raise ValueError('AS2_BEHAVIORS subset is incompatible with follow_path backend')
    if os.environ.get('AS2_TF_SCOPE')=='per-vehicle':
        scoped=str(Path(__file__).with_name('scoped_tf.launch.py'))
        wrapped=[]
        for command in commands:
            standalone=command[2].startswith('/')
            package,name,args=('',command[2],command[3:]) if standalone else (command[2],command[3],command[4:])
            arguments=dict(item.split(':=',1) for item in args)
            replacement=['ros2','launch',scoped,'target_launch:='+name,
                'target_arguments:='+json.dumps(arguments,separators=(',',':'))]
            if package:replacement.append('target_package:='+package)
            wrapped.append(replacement)
        commands=wrapped
    children = []
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    def terminate(child, sig):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    result = 0
    next_memory_check = 0.0
    try:
        for command in commands:
            print('Starting:', ' '.join(command), flush=True)
            children.append(subprocess.Popen(command, start_new_session=True))
        while not stopping:
            for child in children:
                if child.poll() is not None:
                    print(f'AS2 child exited: {child.returncode}', flush=True)
                    result = child.returncode or 1
                    stopping = True
            if memory_limit and time.monotonic() >= next_memory_check:
                processes = {}
                for status in Path('/proc').glob('[0-9]*/status'):
                    try:
                        fields = dict(line.split(':', 1) for line in status.read_text().splitlines()
                                      if ':' in line)
                        processes[int(status.parent.name)] = (
                            int(fields['PPid']), int(fields.get('VmRSS', '0 kB').split()[0]))
                    except (OSError, ValueError, KeyError):
                        continue
                owned = {os.getpid()}
                while True:
                    children_pids = {pid for pid, (parent, _) in processes.items() if parent in owned}
                    if children_pids <= owned:
                        break
                    owned.update(children_pids)
                rss = sum(processes[pid][1] for pid in owned if pid in processes)
                print(f'AS2 process tree RSS: {rss // 1024} MiB', flush=True)
                if rss > memory_limit:
                    print('AS2 memory limit exceeded; stopping owned processes', flush=True)
                    result = 1
                    stopping = True
                next_memory_check = time.monotonic() + 1
            time.sleep(0.2)
    finally:
        for child in children:
            if child.poll() is None:
                terminate(child, signal.SIGTERM)
        deadline = time.monotonic() + 5
        for child in children:
            try:
                child.wait(timeout=max(0.01, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                terminate(child, signal.SIGKILL)
                child.wait()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
