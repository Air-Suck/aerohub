"""PC AirSim/PX4 plus RoboPi AS2, configured by scripts/robopi.sh and .env."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import shlex
import subprocess
import tarfile
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'artifacts/robopi-runtime'


def run(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def required(name):
    value = os.environ.get(name)
    if not value:
        raise ValueError(f'{name} is required; configure .env from .env.example')
    return value


def hdc(*args, **kwargs):
    return run([required('HDC_BIN'), '-t', required('HDC_TARGET'), *args], **kwargs)


def board(command, timeout=180):
    # hdc client success alone is insufficient: check a remote exit sentinel.
    text = hdc('shell', command + '; result=$?; echo __AEROHUB_RC=$result',
               capture_output=True, text=True, timeout=timeout).stdout
    output, marker, code = text.rpartition('__AEROHUB_RC=')
    if not marker or code.strip() != '0':
        raise RuntimeError(text)
    return output.strip()


def board_environment(flight=False):
    remote = required('ROBOPI_ROOT')
    values = dict(ROS_DOMAIN_ID=required('ROS_DOMAIN_ID'), ROS_LOCALHOST_ONLY='0',
                  RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                  FASTRTPS_DEFAULT_PROFILES_FILE=remote + '/fastdds_udp.xml',
                  AS2_CONFIG_FILE=remote + '/config.yaml',
                  AS2_FLEET_FILE=remote + '/fleet.yaml',
                  AS2_CONTROLLER_CONFIG_FILE=remote + '/pid_speed_controller.yaml',
                  AS2_RUNTIME_DIR=remote + '/runtime',
                  AIRSIM_SETTINGS=remote + '/settings.json',
                  AS2_MAX_RSS_MB=os.environ.get('AS2_MAX_RSS_MB', '1536'),
                  AS2_RESET_OOM_SCORE='1',
                  VERIFY_AS2_FLIGHT='1' if flight else '0')
    exports = ' '.join(f'{key}={shlex.quote(value)}' for key, value in values.items())
    return (f'cd {shlex.quote(remote)} && . ./board_env.sh && export {exports} && ')


def profile(path, address):
    tree = ET.parse(ROOT / 'config/dds/fastdds_udp.xml')
    ns = '{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}'
    ET.register_namespace('', ns[1:-1])
    transport = tree.find('.//' + ns + 'transport_descriptor')
    whitelist = ET.SubElement(transport, ns + 'interfaceWhiteList')
    ET.SubElement(whitelist, ns + 'address').text = address
    tree.write(path, encoding='utf-8', xml_declaration=True)


def compose(*args):
    return run(['docker', 'compose', '-p', 'aerohub-robopi',
                '-f', ROOT / 'deployment/flight_board/compose.yaml',
                '-f', OUT / 'compose.yaml', *args])


def state_save(state):
    (OUT / 'state.json').write_text(json.dumps(state, indent=2) + '\n')


def clock_offset():
    start = time.time_ns()
    remote = int(board('python3 -c ' + shlex.quote('import time;print(time.time_ns())')))
    end = time.time_ns()
    return (remote - (start + end) // 2) / 1e9


def report_json(output):
    # HDC merges the remote program's stderr progress with stdout JSON.
    decoder = json.JSONDecoder()
    for index, character in enumerate(output):
        if character != '{':
            continue
        try:
            value, _ = decoder.raw_decode(output[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and 'received' in value and 'missing' in value:
            return json.dumps(value, indent=2) + '\n'
    raise ValueError('Remote verification did not return a structured report')


def down():
    path = OUT / 'state.json'
    if not path.exists():
        print('No owned runtime recorded')
        return
    state = json.loads(path.read_text())
    if state.get('board'):
        print(board(board_environment() + 'python3 manage_runtime.py stop'))
        state['board'] = False
        state_save(state)
    if state.get('compose'):
        compose('down')
        state['compose'] = False
        state_save(state)
    if state.get('scene'):
        run(['bash', ROOT / 'scripts/airsim.sh', 'stop', '--run-dir', OUT / 'scene'])
        state['scene'] = False
        state_save(state)


def deploy():
    work_root = Path(os.environ.get('OHOS_WORK_ROOT', 'artifacts/ohos-as2')).expanduser()
    if not work_root.is_absolute():
        work_root = ROOT / work_root
    install = work_root / 'ws/install'
    # The builder embeds this private prefix; do not silently relocate it.
    if required('ROBOPI_ROOT') != '/data/local/tmp/aerohub-as2':
        raise ValueError('Rebuild with a matching --custom-prefix before changing ROBOPI_ROOT')
    expected = ROOT / 'deployment/companion_as2/px4_msgs/overrides/VehicleCommandAck.msg'
    actual = install / 'px4_msgs/share/px4_msgs/msg/VehicleCommandAck.msg'
    if not actual.exists() or actual.read_bytes() != expected.read_bytes():
        raise RuntimeError('OHOS px4_msgs is stale; rebuild the current 1.18 messages first')
    remote = required('ROBOPI_ROOT')
    print(board(f'mkdir -p {shlex.quote(remote)}'))
    # Refuse deployment while AS2 binaries from this overlay are running.
    check = ("import pathlib; root=" + repr(remote + '/install/') + "; "
             "active=[str(p) for p in pathlib.Path('/proc').glob('[0-9]*/cmdline') "
             "if p.read_bytes().split(bytes([0]))[0].startswith(root.encode())]; "
             "assert not active, 'Stop AS2 before deploy: '+str(active)")
    board('python3 -c ' + shlex.quote(check))
    bundle = OUT / 'overlay.tar.gz'
    # Board tar supports GNU long names, but not Python's default PAX headers.
    with tarfile.open(bundle, 'w:gz', dereference=True, format=tarfile.GNU_FORMAT) as archive:
        archive.add(install, arcname='install')
        files = {
            'board_env.sh': ROOT / 'deployment/robopi/board_env.sh',
            'manage_runtime.py': ROOT / 'deployment/robopi/manage_runtime.py',
            'run_as2.py': ROOT / 'runtime/as2/run.py',
            'verify_interaction.py': ROOT / 'runtime/as2/scripts/verify_interaction.py',
            'verify_sensors.py': ROOT / 'runtime/airsim_sensors/scripts/verify_ros2.py',
            'config.yaml': ROOT / 'config/as2/config.yaml',
            'fleet.yaml': ROOT / 'config/as2/fleet.yaml',
            'pid_speed_controller.yaml': ROOT / 'config/as2/pid_speed_controller.yaml',
            'fastdds_udp.xml': OUT / 'board-dds.xml',
            'settings.json': ROOT / 'config/airsim/settings/px4-sensors.json',
            'source-manifest.json': work_root / 'current-sources.json',
        }
        for name, source in files.items():
            archive.add(source, arcname=name)
    hdc('file', 'send', bundle, remote + '/overlay.tar.gz')
    import hashlib
    digest = hashlib.sha256(bundle.read_bytes()).hexdigest()
    board(f'cd {shlex.quote(remote)} && sha256sum overlay.tar.gz')
    check = ("import hashlib; assert hashlib.sha256(open(" + repr(remote + '/overlay.tar.gz') +
             ", 'rb').read()).hexdigest()==" + repr(digest))
    board('python3 -c ' + shlex.quote(check))
    # Extract first so an incompatible archive cannot replace the working install.
    stamp = str(time.time_ns())
    stage = remote + '/deploy-stage-' + stamp
    board(f'mkdir {shlex.quote(stage)} && tar -xzf {shlex.quote(remote + "/overlay.tar.gz")} '
          f'-C {shlex.quote(stage)} && test -f '
          f'{shlex.quote(stage + "/install/as2_platform_pixhawk/share/as2_platform_pixhawk/package.xml")}')
    activate = (
        'from pathlib import Path; import os; '
        f'root=Path({remote!r}); stage=Path({stage!r}); '
        f'backup=root/"install-backup-{stamp}"; '
        'os.rename(root/"install",backup) if (root/"install").exists() else None; '
        'os.rename(stage/"install",root/"install"); '
        '[os.replace(p,root/p.name) for p in stage.iterdir()]; stage.rmdir()')
    board('python3 -c ' + shlex.quote(activate))
    print('OHOS overlay deployed and transfer hash verified')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['check', 'network', 'sync-time', 'build', 'deploy', 'up', 'status', 'verify', 'down'])
    parser.add_argument('--flight', action='store_true', help='verify also executes simulated flight')
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    for key in ['PC_IP', 'ROBOPI_IP']:
        ipaddress.IPv4Address(required(key))
    domain = int(required('ROS_DOMAIN_ID'))
    if not 0 <= domain <= 101:
        raise ValueError('ROS_DOMAIN_ID must be between 0 and 101 for this setup')
    if args.action == 'network':
        interface = shlex.quote(required('ROBOPI_INTERFACE'))
        address = shlex.quote(required('ROBOPI_IP'))
        print(board(f'ifconfig {interface} {address} netmask 255.255.255.0 up'))
        run(['ping', '-c', '3', '-W', '2', required('ROBOPI_IP')])
    elif args.action == 'sync-time':
        code = f'import time;time.clock_settime_ns(time.CLOCK_REALTIME,{time.time_ns()})'
        board('python3 -c ' + shlex.quote(code))
        print(f'Board minus PC clock offset: {clock_offset():.3f} seconds')
    elif args.action == 'build':
        run(['python3', ROOT / 'scripts/prepare_ohos_sources.py'])
        run(['bash', ROOT / 'deployment/oh-ros-build/build_as2.sh', 'runtime'])
    elif args.action == 'check':
        run(['ping', '-c', '3', '-W', '2', required('ROBOPI_IP')])
        print(board('ifconfig ' + shlex.quote(required('ROBOPI_INTERFACE'))))
        print(f'Board minus PC clock offset: {clock_offset():.3f} seconds')
    elif args.action == 'deploy':
        profile(OUT / 'board-dds.xml', required('ROBOPI_IP'))
        deploy()
    elif args.action == 'up':
        state_path = OUT / 'state.json'
        if state_path.exists() and any(json.loads(state_path.read_text()).values()):
            raise RuntimeError('A runtime is already recorded; inspect status or run down first')
        run(['ping', '-c', '1', '-W', '2', required('ROBOPI_IP')])
        if abs(clock_offset()) > 0.5:
            raise RuntimeError('Board/PC clocks differ; run sync-time or configure time synchronization first')
        profile(OUT / 'pc-dds.xml', required('PC_IP'))
        import yaml
        dds_volume = f'{OUT / "pc-dds.xml"}:/opt/fastdds_udp.xml:ro'
        common_env = {'ROS_DOMAIN_ID': required('ROS_DOMAIN_ID'),
                      'FASTRTPS_DEFAULT_PROFILES_FILE': '/opt/fastdds_udp.xml'}
        services = {
            'drone0': {'environment': common_env, 'volumes': [dds_volume]},
            'clock': {'environment': common_env, 'volumes': [dds_volume]},
        }
        (OUT / 'compose.yaml').write_text(yaml.safe_dump({'services': services}))
        state = {}
        state_save(state)
        try:
            run(['bash', ROOT / 'scripts/airsim.sh', 'start', '--run-dir', OUT / 'scene'])
            state['scene'] = True
            state_save(state)
            state['compose'] = True
            state_save(state)
            compose('up', '-d', '--no-build')
            print(board(board_environment() + 'python3 manage_runtime.py start'))
            state['board'] = True
            state_save(state)
        except BaseException:
            down()
            raise
    elif args.action == 'status':
        compose('ps')
        print(board(board_environment() + 'python3 manage_runtime.py status'))
    elif args.action == 'verify':
        for name in ['interaction.json', 'sensors.json']:
            (OUT / name).unlink(missing_ok=True)
        output = board(board_environment(args.flight) + 'python3 verify_interaction.py', timeout=240)
        (OUT / 'interaction.log').write_text(output + '\n')
        (OUT / 'interaction.json').write_text(report_json(output))
        print(output)
        output = board(board_environment() + 'python3 verify_sensors.py')
        (OUT / 'sensors.log').write_text(output + '\n')
        (OUT / 'sensors.json').write_text(report_json(output))
        print(output)
    else:
        down()


if __name__ == '__main__':
    main()
