#!/usr/bin/env python3
"""AirSim host lifecycle. Identity/listener checks adapted from LeUAV runtime/processes.py."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

def start_ticks(pid):
    data = Path(f"/proc/{pid}/stat").read_text()
    tail = data[data.rfind(")") + 2:].split()
    if tail[0] == "Z":
        raise ProcessLookupError(pid)
    return tail[19]


def host_alive(state):
    try:
        pid = state["pid"]
        return (start_ticks(pid) == state["start_ticks"]
                and Path(f"/proc/{pid}/exe").resolve() == Path(state["binary"]).resolve()
                and f"-settings={state['runtime_settings']}".encode()
                in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0"))
    except (OSError, KeyError, ProcessLookupError):
        return False


def listen_owned(pid, port):
    """只查监听，不连接 4560，避免消耗 AirSim 的单次 accept。"""
    inodes = set()
    try:
        for fd in Path(f"/proc/{pid}/fd").iterdir():
            try:
                link = os.readlink(fd)
                if link.startswith("socket:["):
                    inodes.add(link[8:-1])
            except OSError:
                pass
        for row in Path("/proc/net/tcp").read_text().splitlines()[1:]:
            fields = row.split()
            if int(fields[1].split(":")[1], 16) == port and fields[3] == "0A" and fields[9] in inodes:
                return True
    except OSError:
        pass
    return False


def save(directory, state):
    temporary = directory / 'state.json.tmp'
    temporary.write_text(json.dumps(state, indent=2) + '\n')
    temporary.replace(directory / 'state.json')


def stop(state, directory):
    if not host_alive(state):
        return
    os.killpg(state['pid'], signal.SIGTERM)
    deadline = time.monotonic() + 5
    while host_alive(state) and time.monotonic() < deadline:
        time.sleep(0.1)
    if host_alive(state):
        os.killpg(state['pid'], signal.SIGKILL)
    state['phase'] = 'stopped'
    save(directory, state)


def load_profile(profile_path, binary_override=None):
    profile = json.loads(profile_path.read_text())
    if binary_override:
        profile['binary'] = binary_override
    def resolve(value):
        path = Path(value).expanduser()
        return (profile_path.parent / path).resolve() if not path.is_absolute() else path.resolve()
    if 'binary' in profile:
        binary = resolve(profile['binary'])
    else:
        from resources import resolve_resource
        binary = resolve_resource(profile['binary_resource'])
    settings_path = resolve(profile['settings'])
    profile['binary'] = str(binary)
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise ValueError('AirSim binary is missing or not executable: ' + str(binary))
    settings = json.loads(settings_path.read_text())
    if settings.get('SimMode') != 'Multirotor' or not settings.get('RpcEnabled', True):
        raise ValueError('settings require Multirotor mode and RPC enabled')
    vehicles = settings.get('Vehicles', {})
    if not isinstance(vehicles, dict) or not vehicles:
        raise ValueError('settings require at least one vehicle')
    udp_ports = []
    for name, vehicle in vehicles.items():
        if (vehicle.get('VehicleType') != 'PX4Multirotor'
                or not vehicle.get('UseTcp') or not vehicle.get('LockStep')):
            raise ValueError(f'{name}: requires PX4Multirotor, UseTcp and LockStep')
        udp_ports.extend(int(vehicle.get(key, default)) for key, default in
                         [('ControlPortLocal', 14540), ('ControlPortRemote', 14580)])
    if settings.get('ClockType') != 'SteppableClock':
        raise ValueError('PX4 lockstep requires SteppableClock')
    if len(set(udp_ports)) != len(udp_ports):
        raise ValueError('duplicate vehicle control UDP ports in settings')
    ports = [int(settings.get('ApiServerPort', 41451))]
    ports += [int(v.get('TcpPort', 4560)) for v in settings.get('Vehicles', {}).values()
              if v.get('UseTcp')]
    if len(set(ports)) != len(ports):
        raise ValueError('duplicate RPC/PX4 TCP ports in settings')
    for port in ports + udp_ports:
        if not 1 <= port <= 65535:
            raise ValueError('invalid TCP/UDP port')
    arguments = profile.get('args', ['-RenderOffscreen', '-vulkan', '-nosound'])
    if not isinstance(arguments, list) or any(not isinstance(arg, str) for arg in arguments):
        raise ValueError('profile args must be a list of strings')
    if any(arg.startswith(('-settings', '-userdir')) for arg in arguments):
        raise ValueError('settings and userdir are managed by the lifecycle manager')
    profile['args'] = arguments
    profile['settings'] = str(settings_path)
    return profile, settings, ports


def start(profile_path, directory, timeout, binary_override=None):
    profile, settings, ports = load_profile(profile_path, binary_override)
    binary = Path(profile['binary'])
    for port in ports:
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', port))
            sock.listen(1)
    runtime_settings = directory / 'settings.json'
    runtime_settings.write_text(json.dumps(settings, indent=2) + '\n')
    (directory / 'profile.json').write_text(json.dumps(profile, indent=2) + '\n')
    userdir = directory / 'user'
    userdir.mkdir(exist_ok=True)
    command = [str(binary), profile.get('project', binary.name)]
    command += profile['args']
    command += [f'-settings={runtime_settings}', f'-userdir={userdir}']
    env = dict(os.environ)
    env['HOME'] = str(userdir)
    with (directory / 'airsim.log').open('w') as log:
        child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
    # exec may not have completed yet; retain the process generation first.
    state = dict(pid=child.pid, start_ticks=start_ticks(child.pid), binary=str(binary),
                 runtime_settings=str(runtime_settings), command=command, ports=ports,
                 phase='starting', log=str(directory / 'airsim.log'))
    save(directory, state)
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError('AirSim exited; see ' + state['log'])
            if host_alive(state) and all(listen_owned(child.pid, port) for port in ports):
                state['phase'] = 'listening'
                save(directory, state)
                return state
            time.sleep(0.2)
        raise TimeoutError('AirSim listener readiness timed out; see ' + state['log'])
    except BaseException:
        stop(state, directory)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['check', 'start', 'status', 'stop'])
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--binary', help='override the scene executable in the profile')
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=45)
    args = parser.parse_args()
    if args.action == 'check':
        if args.profile is None:
            raise ValueError('check requires --profile')
        profile, settings, ports = load_profile(args.profile.expanduser().resolve(), args.binary)
        print(json.dumps(dict(profile=profile, tcp_ports=ports,
                              vehicles=settings['Vehicles']), indent=2))
        return 0
    directory = args.run_dir.expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'manager.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        path = directory / 'state.json'
        state = json.loads(path.read_text()) if path.exists() else {}
        if args.action == 'status':
            state['alive'] = host_alive(state)
            print(json.dumps(state, indent=2))
            return 0 if state['alive'] else 1
        if args.action == 'stop':
            stop(state, directory)
            return 0
        if host_alive(state):
            raise RuntimeError('this run directory already owns a live AirSim instance')
        if args.profile is None:
            raise ValueError('start requires --profile')
        if args.timeout <= 0:
            raise ValueError('timeout must be positive')
        print(json.dumps(start(args.profile.resolve(), directory, args.timeout, args.binary), indent=2))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, KeyError, ImportError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
