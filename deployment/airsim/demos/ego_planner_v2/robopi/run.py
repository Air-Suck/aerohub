"""Single RoboPi deployment CLI. prepare is offline; all runtime actions are explicit."""
import argparse
import hashlib
import ipaddress
import json
import os
import re
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from policy import run_id
from dds import tcp_profile
from dds_mapped import mapped_profile
from interfaces import topic_map

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[4]
FLIGHT = 'ghcr.io/popc0rn7/aerohub/flight-board@sha256:1eff3190ca7870d0267ddb1531365dd7c68d85c6e52ade3d25b17b84b4758843'
ONBOARD = 'ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9'


def write(path, value):
    path.write_text(json.dumps(value, indent=2)+'\n', encoding='utf-8')


def execute(args, timeout=120):
    return subprocess.run(list(map(str, args)), check=True, capture_output=True,
                          text=True, timeout=timeout).stdout


def validate(config):
    if type(config['domain']) is not int or not 1 <= config['domain'] <= 100:
        raise ValueError('domain must be 1..100')
    for key in ('pc_ip', 'board_ip', 'airsim_host'):
        ipaddress.IPv4Address(config[key])
    if config['pc_ip'] == config['board_ip']:
        raise ValueError('PC and board must have different addresses')
    candidates = {'/data/aerohub/validation/ego-v2-20261112': '',
                  '/data/aerohub/validation/ego-v2-scoped-20261113': 'drone0',
                  '/data/aerohub/validation/ego-v2-pr1': 'drone0'}
    if config['candidate'] not in candidates or config.get('vehicle', '') != candidates[config['candidate']]:
        raise ValueError('Candidate and explicit vehicle scope must match')
    topic_map(config)
    if not config.get('target'):
        raise ValueError('An explicit HDC target is required')
    if config.get('fault_case') not in (None, 'cancel', 'planner_exit', 'cloud', 'odom'):
        raise ValueError('Unknown fault case')
    if config.get('dds_transport', 'udp') not in ('udp', 'tcp_board_listener', 'udp_mapped'):
        raise ValueError('Unsupported DDS transport')
    if config.get('dds_transport') == 'tcp_board_listener':
        port = config.get('tcp_base_port')
        if type(port) is not int or not 1024 <= port <= 65512:
            raise ValueError('tcp_base_port must leave room for 24 listeners')
        pc_port = config.get('tcp_pc_base_port', 20000)
        if type(pc_port) is not int or not 1024 <= pc_port <= port-5:
            raise ValueError('PC TCP ports must be below board listener ports')


def dds(path, local, peer, domain):
    root = ET.Element('profiles', xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    tr = ET.SubElement(ET.SubElement(root, 'transport_descriptors'), 'transport_descriptor')
    ET.SubElement(tr, 'transport_id').text = 'ego_udp'
    ET.SubElement(tr, 'type').text = 'UDPv4'
    interfaces = ET.SubElement(tr, 'interfaceWhiteList')
    for ip in (local, '127.0.0.1'):
        ET.SubElement(interfaces, 'address').text = ip
    rtps = ET.SubElement(ET.SubElement(root, 'participant', profile_name='ego_single', is_default_profile='true'), 'rtps')
    ET.SubElement(ET.SubElement(rtps, 'userTransports'), 'transport_id').text = 'ego_udp'
    ET.SubElement(rtps, 'useBuiltinTransports').text = 'false'
    peers = ET.SubElement(ET.SubElement(rtps, 'builtin'), 'initialPeersList')
    for ip in (local, peer, '127.0.0.1'):
        for index in range(32):
            locator = ET.SubElement(ET.SubElement(peers, 'locator'), 'udpv4')
            ET.SubElement(locator, 'address').text = ip
            ET.SubElement(locator, 'port').text = str(7400+250*domain+10+2*index)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def prepare(out, config):
    validate(config)
    run_id(out.name)
    out.mkdir(parents=True, exist_ok=False)
    bundle = out/'bundle'
    bundle.mkdir()
    remote = config['candidate']+'/sessions/'+out.name
    config = dict(config, remote=remote, run_id=out.name)
    write(out/'config.json', config)
    write(bundle/'config.json', config)
    for name in ('board.py', 'runtime.py', 'policy.py', 'interfaces.py', 'dds.py', 'dds_exec.py', 'ground.launch.py', 'as2.yaml', 'controller.yaml'):
        shutil.copyfile(HERE/name, bundle/name)
    if config.get('fault_case'):
        for name in ('fault.launch.py','board_fault_injector.py'):
            shutil.copyfile(HERE/name,bundle/name)
        shutil.copyfile(PROJECT/'deployment/airsim/demos/ego_planner_v2/pc/fault_injector.py',bundle/'fault_injector_base.py')
    write(bundle/'fleet.json', {'vehicles': {'drone0': {'fmu_prefix': '/drone0', 'target_system_id': 135}}})
    # JSON is also valid YAML for the existing launch/config readers.
    write(bundle/'ego.json', dict(task={'task_id': 'ego_'+out.name}, vehicles=['drone0'],
        planner_ids={'drone0': 0}, deployment={'drone0': {'namespace': 'drone0',
        'px4_origin_in_fleet_ned': [0, 0, 0]}}, limits={'velocity': .23, 'acceleration': .35, 'tolerance': .05},
        planner={'parameters': {'valid_from_lead': .4, 'sample_dt': .1}}))
    if config.get('dds_transport') == 'tcp_board_listener':
        tcp_profile(bundle/'dds.xml', config['board_ip'], config['tcp_base_port'], listen_port=config['tcp_base_port'], domain=config['domain'])
        for offset, role in enumerate(['agent', 'sensors', 'clock', 'mission', 'observer']):
            path = out/('pc-dds.xml' if role == 'observer' else 'pc-dds-'+role+'.xml')
            tcp_profile(path, config['board_ip'], config['tcp_base_port'],
                listen_port=config.get('tcp_pc_base_port', 20000)+offset, pc_local=config['pc_ip'],
                profile_name='px4_participant' if role == 'agent' else 'ego_tcp', domain=config['domain'])
        shutil.copyfile(HERE/'pc_supervisor.py', out/'pc_supervisor.py')
    elif config.get('dds_transport') == 'udp_mapped':
        dds(bundle/'dds.xml', config['board_ip'], config['pc_ip'], config['domain'])
        for index, role in enumerate(['agent', 'sensors', 'clock', 'mission', 'observer']):
            path = out/('pc-dds.xml' if role == 'observer' else 'pc-dds-'+role+'.xml')
            mapped_profile(path, config['pc_ip'], config['board_ip'], config['domain'], index,
                           profile_name='px4_participant' if role == 'agent' else 'ego_mapped_udp')
        shutil.copyfile(HERE/'pc_supervisor.py', out/'pc_supervisor.py')
    else:
        dds(bundle/'dds.xml', config['board_ip'], config['pc_ip'], config['domain'])
        dds(out/'pc-dds.xml', config['pc_ip'], config['board_ip'], config['domain'])
    settings = json.loads((PROJECT/'deployment/airsim/demos/ego_planner_v2/pc/settings.snapshot.json').read_text())
    settings['Vehicles'] = {'Drone0': settings['Vehicles']['Drone0']}
    write(out/'settings.json', settings)
    env = {'ROS_DOMAIN_ID': str(config['domain']), 'ROS_LOCALHOST_ONLY': '0',
           'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'FASTRTPS_DEFAULT_PROFILES_FILE': '/session/pc-dds.xml',
           'AIRSIM_HOST': config['airsim_host'], 'AIRSIM_RPC_PORT': '41451',
           'VEHICLE_NAME': 'Drone0', 'VEHICLE_NAMESPACE': 'drone0', 'PX4_INSTANCE': '0',
           'PX4_SYSTEM_ID': '135', 'XRCE_AGENT_PORT': '8888', 'FLIGHT_BOARD_CAMERAS': '0'}
    common = dict(image=FLIGHT, pull_policy='never', init=True, network_mode='host',
        environment=env, volumes=[str(out)+':/session:ro', str(out/'settings.json')+':/config/settings.json:ro'])
    if config.get('dds_transport') in ('tcp_board_listener', 'udp_mapped'):
        common['entrypoint'] = ['bash', '-c', 'set -e; source /opt/ros/humble/setup.bash; '
            'source /opt/platform_sensors/install/setup.bash; exec python3 /session/pc_supervisor.py "$@"', 'ego-flight-board']
    services = {name: dict(common, command=[command]) for name, command in [('px4', 'vehicle'), ('clock', 'clock')]}
    if config.get('dds_transport') == 'udp_mapped':
        # Keep native DDS UDP, with explicit Docker NAT port/address mapping.
        # A participant owns a discovery port and the following user-data port.
        meta = 7400+250*config['domain']+10
        for service, first, last in [('px4', meta, meta+3), ('clock', meta+4, meta+5)]:
            services[service].pop('network_mode')
            services[service]['ports'] = [f"{config['pc_ip']}:{first}-{last}:{first}-{last}/udp"]
        control_port = settings['Vehicles']['Drone0']['ControlPortRemote']
        services['px4']['ports'].append(f'127.0.0.1:{control_port}:{control_port}/udp')
    write(out/'compose.json', {'name': 'ego-single-'+out.name.lower(), 'services': services})
    write(out/'sources.json', {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in bundle.iterdir()})
    write(out/'workstation-sources.json', {str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [HERE/'mission_client.py', PROJECT/'deployment/airsim/demos/ego_planner_v2/pc/flight_mission.py',
                  PROJECT/'deployment/airsim/demos/ego_planner_v2/pc/ego_flight_mission.py',
                  PROJECT/'deployment/airsim/demos/ego_planner_v2/pc/ego_reference_check.py', HERE/'interfaces.py',
                  *sorted((PROJECT/'src/planner/ego_planner_v2/ros2_adapter/bridge/uav_ego_planner_adapter').glob('*.py')),
                  HERE/'fault_mission.py']})
    write(out/'roles.json', {'board': ['AS2', 'navigation_inputs', 'EGO', 'Adapter'],
        'pc': ['AirSim (existing, separately managed)', 'PX4/Agent/sensors', 'clock', 'mission client'],
        'runtime_validated': False, 'network': ('Docker bridge with mapped UDP ports and advertised host address' if config.get('dds_transport')=='udp_mapped' else 'ordinary host-network containers; DDS reachability must be validated')})
    return config


def hdc(config, *args, timeout=120):
    return execute([config.get('hdc', 'hdc'), '-t', config['target'], *args], timeout=timeout)


def board_shell(config, command, timeout=120):
    output = hdc(config, 'shell', command+'; result=$?; echo EGO_EXIT=$result', timeout=timeout)
    if not output.splitlines() or output.splitlines()[-1].strip() != 'EGO_EXIT=0':
        raise RuntimeError('Board operation failed: '+output[-2000:])
    return output.rsplit('EGO_EXIT=0', 1)[0].strip()


def board_action(config, action):
    q = shlex.quote
    remote = config['remote']
    command = ('cd '+q(remote)+' && . '+q(config['candidate']+'/board_env.sh')+
        ' && export ROS_DOMAIN_ID='+str(config['domain'])+' ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp'+
        ' FASTRTPS_DEFAULT_PROFILES_FILE='+q(remote+'/dds.xml')+
        ' && python3 '+q(remote+'/board.py')+' '+action+' --run-id '+q(config['run_id']))
    return json.loads(board_shell(config, command))


def mission(config, out, stage, baseline):
    """Explicit motion entry; never called by prepare/start/check."""
    if config.get('dds_transport') == 'tcp_board_listener':
        raise RuntimeError('This network profile still requires complete ground/mission integration validation; flight disabled')
    if stage == 'fault' and not config.get('fault_case'):
        raise RuntimeError('Fault mission requires an explicitly prepared fault session')
    for path, digest in json.loads((out/'workstation-sources.json').read_text()).items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise RuntimeError('Workstation source changed since prepare: '+path)
    readiness = board_action(config, 'check')
    if not readiness['ready']:
        raise RuntimeError('Board capability is not ready')
    if config.get('dds_transport') == 'udp_mapped':
        observer = out/'observer.json'
        obs = json.loads(observer.read_text())
        if (time.time()-observer.stat().st_mtime > 300 or obs['duration'] < 30 or
                not all(row['count'] >= 30 and row.get('receipt_age', 1e9) < .75 and
                        row.get('max_gap', 1e9) < .75 and len(obs['graph'][topic]['publishers']) == 1
                        for topic, row in obs['topics'].items())):
            raise RuntimeError('Fresh continuous ground observation required before motion')
    samples = []
    for _ in range(3):
        before = time.time()
        remote_time = float(board_shell(config, "python3 -c 'import time;print(time.time())'", timeout=5))
        after = time.time()
        samples.append(dict(rtt=after-before, offset=remote_time-(before+after)/2))
    write(out/('clock-before-'+stage+'.json'), samples)
    best = min(samples, key=lambda s:s['rtt'])
    if abs(best['offset'])+best['rtt']/2 > .5:
        raise RuntimeError('Wall-clock offset/uncertainty exceeds 0.5 seconds; no motion requested')
    evidence = out/('mission-'+stage)
    evidence.mkdir(exist_ok=False)
    network, profile = mission_network(config)
    args = ['docker', 'run', '--rm', '--init', '--pull', 'never',
            '--name', 'ego-single-mission-'+config['run_id'].lower(), *network,
            '--add-host', 'host.docker.internal:'+config['airsim_host'],
            '-e', 'ROS_DOMAIN_ID='+str(config['domain']), '-e', 'ROS_LOCALHOST_ONLY=0',
            '-e', 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE='+profile,
            '-v', str(out)+':/session:ro', '-v', str(evidence)+':/evidence',
            '-v', str(PROJECT/'deployment/airsim/demos/ego_planner_v2/pc')+':/input:ro',
            '-v', str(HERE)+':/entry:ro',
            '-v', str(PROJECT/'src/planner/ego_planner_v2/ros2_adapter/bridge')+':/adapter-source:ro',
            '-v', os.environ['EGO_PC_WORK']+':/work:ro']
    if stage in ('ego', 'fault'):
        if baseline is None:
            raise ValueError('mission-ego requires --baseline pointing to this topology\'s basic result directory')
        baseline = baseline.resolve()
        result = json.loads((baseline/'result.json').read_text())
        provenance = json.loads((baseline/'deployment-session.json').read_text())
        if (not result.get('passed') or not result.get('final_disarmed') or result.get('recovery')
                or provenance.get('run_id') != config['run_id']):
            raise ValueError('A successful basic flight in this session is required')
        args += ['-v', str(baseline)+':/baseline:ro']
    write(evidence/'deployment-session.json', dict(run_id=config['run_id'], candidate=config['candidate'],
        image=ONBOARD, role='ROS task client only', script='fault_mission.py' if stage=='fault' else ('ego_flight_mission.py' if stage=='ego' else 'flight_mission.py')))
    restore_ground_hold(out, evidence)
    setup = 'set -e; '
    for prefix in ('/opt/ros/humble', '/opt/px4_ws/install', '/work/install/humble',
                   '/root/aerostack2_ws/install', '/work/ground_overlay/install', '/work/ego_runtime/install'):
        setup += 'source '+prefix+'/local_setup.bash; '
    script = 'ego_flight_mission.py' if stage == 'ego' else 'flight_mission.py'
    args += ['--entrypoint', 'bash', ONBOARD, '-c', setup+'export PYTHONPATH=/adapter-source:$PYTHONPATH; exec python3 /entry/mission_client.py '+stage]
    # A task failure must not automatically stop PX4, clock, or board control.
    with (evidence/'client.log').open('w') as log:
        result = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        while result.poll() is None:
            try:
                before = time.time()
                remote_time = float(board_shell(config, "python3 -c 'import time;print(time.time())'", timeout=5))
                after = time.time()
                sample = dict(rtt=after-before, offset=remote_time-(before+after)/2)
                with (evidence/'clock-offsets.jsonl').open('a') as stream:
                    stream.write(json.dumps(sample)+'\n')
                if abs(sample['offset'])+sample['rtt']/2 > .5:
                    raise RuntimeError('Wall-clock offset/uncertainty exceeds 0.5 seconds')
            except Exception as exc:
                (evidence/'recovery.request').write_text(str(exc))
            time.sleep(2)
    write(evidence/'client-exit.json', {'returncode':result.returncode})
    if result.returncode:
        raise RuntimeError('Task failed; execution environment retained for recovery')


def restore_ground_hold(out, evidence):
    """Restore the existing PC runner's disarmed Hold prerequisite before rearming."""
    prefix = ['docker', 'compose', '-f', str(out/'compose.json'), 'exec', '-T',
              '-w', '/var/lib/flight_board', 'px4']
    commander = prefix+['/opt/px4-gazebo/bin/px4-commander', '--instance', '0']
    rows = []
    def query(args):
        value = execute(args, timeout=10)
        rows.append(dict(wall=time.time(), command=args, output=value))
        return value
    try:
        if not re.search(r'\bDisarmed\b', query(commander+['status'])):
            raise RuntimeError('Ground Hold reset requires confirmed PX4 Disarmed')
        query(commander+['mode', 'auto:loiter'])
        deadline = time.monotonic()+10
        while True:
            status = query(commander+['status'])
            vehicle = query(prefix+['/opt/px4-gazebo/bin/px4-listener', '--instance', '0', 'vehicle_status'])
            if (re.search(r'\bDisarmed\b',status) and 'navigation mode: Hold' in status and
                    'pre_flight_checks_pass: True' in vehicle):
                return
            if time.monotonic()>=deadline:
                raise RuntimeError('Disarmed Hold and PX4 preflight health not confirmed; no motion requested')
            time.sleep(.2)
    finally:
        write(evidence/'ground-hold.json', rows)


def mission_network(config):
    if config.get('dds_transport') == 'udp_mapped':
        port = 7400+250*config['domain']+16
        return ['-p', f"{config['pc_ip']}:{port}-{port+1}:{port}-{port+1}/udp"], '/session/pc-dds-mission.xml'
    return ['--network', 'host'], '/session/pc-dds.xml'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'deploy', 'pc-start', 'start', 'status', 'check', 'stop', 'collect', 'pc-stop', 'mission-basic', 'mission-ego', 'mission-fault'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--baseline', type=Path)
    args = parser.parse_args()
    out = args.output.resolve()
    if args.action == 'prepare':
        if not args.config:
            parser.error('prepare requires --config')
        prepare(out, json.loads(args.config.read_text(encoding='utf-8-sig')))
        print('Prepared offline: '+str(out)); return
    config = json.loads((out/'config.json').read_text())
    validate(config)
    if config['run_id'] != out.name or config['remote'] != config['candidate']+'/sessions/'+run_id(out.name):
        raise ValueError('Session identity mismatch')
    if args.action.startswith('mission-'):
        mission(config, out, args.action.removeprefix('mission-'), args.baseline)
    elif args.action == 'deploy':
        remote = shlex.quote(config['remote'])
        board_shell(config, 'test ! -e '+remote+' && mkdir -p '+remote)
        for p in (out/'bundle').iterdir():
            hdc(config, 'file', 'send', str(p), config['remote']+'/'+p.name)
            digest = hashlib.sha256(p.read_bytes()).hexdigest()
            result = board_shell(config, 'sha256sum '+shlex.quote(config['remote']+'/'+p.name))
            if result.split()[0] != digest:
                raise RuntimeError('Transferred file hash mismatch: '+p.name)
        write(out/'deployed.json', {'verified': True, 'remote': config['remote']})
    elif args.action in ('pc-start', 'pc-stop'):
        if args.action == 'pc-stop':
            status = board_action(config, 'stop')
            write(out/'board-stop.json', status)
        command = ['docker', 'compose', '-f', str(out/'compose.json')]
        if args.action == 'pc-start':
            print(execute(command+['up', '-d', '--no-build']))
        else:
            (out/'pc.log').write_text(execute(command+['logs', '--no-color']))
            print(execute(command+['down']))
    elif args.action == 'collect':
        remote = config['candidate']+'/runtime/ego-single/'+config['run_id']
        archive = config['remote']+'/evidence.tar.gz'
        result = board_shell(config, 'tar -czf '+shlex.quote(archive)+' -C '+shlex.quote(remote)+' . && sha256sum '+shlex.quote(archive))
        digest = result.split()[0]
        destination = out/('board-'+str(time.time_ns())+'.tar.gz')
        hdc(config, 'file', 'recv', archive, str(destination))
        if hashlib.sha256(destination.read_bytes()).hexdigest() != digest:
            raise RuntimeError('Evidence hash mismatch')
        write(destination.with_suffix('.sha256.json'), {'sha256': digest})
        print(destination)
    else:
        result = board_action(config, args.action)
        write(out/(args.action+'-'+str(time.time_ns())+'.json'), result)
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
