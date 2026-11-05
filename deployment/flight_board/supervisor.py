"""Run one aircraft's PX4/Agent/sensors, or the one global clock bridge."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import threading


def number(name, default, low=1, high=65535):
    value = int(os.environ.get(name, default))
    if not low <= value <= high:
        raise ValueError(f'{name} must be in [{low}, {high}]')
    return value


def commands(mode, env):
    host = env.get('AIRSIM_HOST', '127.0.0.1')
    rpc = str(number('AIRSIM_RPC_PORT', 41471))
    vehicle = env.get('VEHICLE_NAME', 'PX4')
    common = ['--ros-args', '-p', f'rpc_host:={host}', '-p', f'rpc_port:={rpc}',
              '-p', f'vehicle_name:={vehicle}']
    if mode == 'clock':
        return [(['ros2', 'run', 'uav_as2_sensor_adapter', 'airsim_clock_node'] + common,
                 env, None)]
    if mode != 'vehicle':
        raise ValueError('usage: flight-board [vehicle|clock|shell]')
    config_path = env.get('AIRSIM_SETTINGS', '/config/settings.json')
    settings = json.loads(Path(config_path).read_text())
    config = settings['Vehicles'][vehicle]
    if config.get('VehicleType') != 'PX4Multirotor' or not config.get('UseTcp'):
        raise ValueError('vehicle requires PX4Multirotor and UseTcp=true')
    if not config.get('LockStep', False):
        raise ValueError('vehicle requires LockStep=true')
    tcp_port = int(config.get('TcpPort', 4560))
    if not 1 <= tcp_port <= 65535:
        raise ValueError('invalid TcpPort')
    instance = number('PX4_INSTANCE', 0, 0, 200)
    system = number('PX4_SYSTEM_ID', instance + 1, 1, 255)
    agent_port = number('XRCE_AGENT_PORT', 8888)
    namespace = env.get('VEHICLE_NAMESPACE', 'drone0')
    # Writable per-container copy; never patch the image or a shared PX4 tree.
    runtime = Path('/var/lib/flight_board')
    etc = runtime / 'etc'
    shutil.copytree('/opt/px4-gazebo/etc', etc, dirs_exist_ok=True)
    sim = etc / 'init.d-posix/px4-rc.mavlinksim'
    sim.write_text(sim.read_text().replace('simulator_tcp_port=$((4560+px4_instance))',
                                          f'simulator_tcp_port={tcp_port}'))
    mav = etc / 'init.d-posix/px4-rc.mavlink'
    text = mav.read_text()
    for variable, value in [('udp_offboard_port_local', config.get('ControlPortRemote', 14580+instance)),
                            ('udp_offboard_port_remote', config.get('ControlPortLocal', 14540+instance))]:
        value = int(value)
        if not 1 <= value <= 65535:
            raise ValueError('invalid control port')
        lines = text.splitlines()
        text = '\n'.join(f'{variable}={value}' if line.startswith(variable+'=') else line
                         for line in lines) + '\n'
    text = '\n'.join(line for line in text.splitlines()
                     if 'udp_offboard_port_remote=14549' not in line) + '\n'
    mav.write_text(text)
    px4_env = dict(env, PX4_SIM_MODEL='none_iris', PX4_SYS_AUTOSTART='10016',
                   PX4_SIMULATOR='mavlink', PX4_SIM_HOST_ADDR=host,
                   PX4_UXRCE_DDS_PORT=str(agent_port), PX4_UXRCE_DDS_NS=namespace,
                   PX4_PARAM_MAV_SYS_ID=str(system), PX4_PARAM_UXRCE_DDS_KEY=str(system),
                   PX4_PARAM_SDLOG_MODE='-1')
    px4_env.pop('PX4_SIM_HOSTNAME', None)
    agent_env = dict(env)
    # PX4/Agent use private ELF loader/RPATH; never inherit ROS library paths.
    agent_env.pop('LD_LIBRARY_PATH', None)
    px4_env.pop('LD_LIBRARY_PATH', None)
    result = [
        (['/opt/xrce/bin/MicroXRCEAgent', 'udp4', '-p', str(agent_port)], agent_env, None),
        (['/opt/px4-gazebo/bin/px4', '-i', str(instance), '-d', str(etc)], px4_env, str(runtime)),
    ]
    if env.get('FLIGHT_BOARD_CAMERAS', '1') not in ('0', '1'):
        raise ValueError('FLIGHT_BOARD_CAMERAS must be 0 or 1')
    if env.get('FLIGHT_BOARD_SENSORS', '1') not in ('0', '1'):
        raise ValueError('FLIGHT_BOARD_SENSORS must be 0 or 1')
    if env.get('FLIGHT_BOARD_SENSORS', '1') == '1':
        result.append(
        (['ros2', 'run', 'uav_as2_sensor_adapter', 'airsim_sensor_node'] + common +
         ['-p', f'settings_file:={config_path}', '-p', f'vehicle_namespace:={namespace}',
          '-p', 'use_sim_time:=true', '-p',
          'camera_enabled:=' + ('true' if env.get('FLIGHT_BOARD_CAMERAS', '1') == '1' else 'false')], env, None))
    return result


STATE = Path('/run/flight_board/state.json')


def ticks(pid):
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    if fields[0] == 'Z':
        raise ProcessLookupError(pid)
    return fields[19]


def status():
    try:
        state = json.loads(STATE.read_text())
    except (OSError, ValueError):
        return {'healthy': False, 'processes': {}}
    for record in state['processes'].values():
        try:
            record['alive'] = ticks(record['pid']) == record['ticks']
        except (OSError, ProcessLookupError):
            record['alive'] = False
    expected = {'clock'} if state['mode'] == 'clock' else {'px4', 'agent', 'sensors'}
    if state['mode'] == 'vehicle' and os.environ.get('FLIGHT_BOARD_SENSORS', '1') == '0':
        expected.discard('sensors')
    state['healthy'] = set(state['processes']) == expected and all(
        p['alive'] for p in state['processes'].values())
    return state


def save(state):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    temporary = STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(state))
    temporary.replace(STATE)


def relay(name, stream):
    for line in stream:
        print(f'[{name}] {line.rstrip()}', flush=True)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'vehicle'
    if mode in ('status', 'healthcheck'):
        report = status()
        if mode == 'status':
            print(json.dumps(report))
        return 0 if report['healthy'] else 1
    if mode == 'shell':
        os.execvp('bash', ['bash'] + sys.argv[2:])
    children = []
    threads = []
    state = {'mode': mode, 'processes': {}}
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    result = 0

    def terminate(child, sig):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            pass
    try:
        save(state)
        names = ['clock'] if mode == 'clock' else ['agent', 'px4', 'sensors']
        for name, (command, environment, cwd) in zip(names, commands(mode, dict(os.environ))):
            child = subprocess.Popen(command, env=environment, cwd=cwd,
                                     start_new_session=True, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, errors='replace', bufsize=1)
            children.append(child)
            state['processes'][name] = dict(pid=child.pid, ticks=ticks(child.pid), exit_code=None)
            save(state)
            thread = threading.Thread(target=relay, args=(name, child.stdout), daemon=True)
            thread.start()
            threads.append(thread)
        while not stopping:
            for child in children:
                if child.poll() is not None:
                    print(f'Child {child.pid} exited: {child.returncode}', flush=True)
                    result = child.returncode or 1
                    stopping = True
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
        for name, child in zip(state['processes'], children):
            state['processes'][name]['exit_code'] = child.returncode
        save(state)
        for thread in threads:
            thread.join(timeout=1)
    return result


if __name__ == '__main__':
    sys.exit(main())
