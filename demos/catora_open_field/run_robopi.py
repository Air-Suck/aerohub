#!/usr/bin/env python3
"""Four-aircraft CAT-ORA/AS2 on RoboPi; PX4 containers and AirSim on PC."""
import argparse
import json
import os
from pathlib import Path
import shlex
import hashlib
import time
import socket
import socketserver
import select
import threading
import fcntl
import subprocess
import sys
import tarfile
import yaml

from run import APP, BOARD, NAMES, PREFIX, ROOT, cleanup, execute, manager
from sim_startup import launch_vehicles

REMOTE = '/data/aerohub/lib/catora/validation'
PACKAGES = ['mrs_formation_reshaping', 'uav_planner_interfaces', 'uav_cat_ora_adapter',
            'uav_planner_framework', 'uav_as2_setpoint_adapter']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preflight-only', action='store_true', help='Ground startup and clock diagnostics; no arming')
    parser.add_argument('--scenario',choices=('normal','cancel','stream-loss'),default='normal')
    parser.add_argument('--tf-scope',choices=('shared','per-vehicle'),default='per-vehicle')
    parser.add_argument('--mixed',action='store_true',help='Three PC compute containers; only drone0 AS2 on RoboPi')
    parser.add_argument('--compute-image',default='uav/catora-hybrid-compute:local')
    args = parser.parse_args()
    if args.mixed and args.scenario!='normal':raise ValueError('Mixed baseline currently supports normal/preflight only')
    global REMOTE, BOARD
    REMOTE += '/run-'+str(time.time_ns())
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    lock = (ROOT/'artifacts/robopi-runtime/cli.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if any((output/name).exists() for name in ('result.json', 'failure.json', 'mission.log')):
        raise RuntimeError('Use a fresh output directory')
    loaded = subprocess.check_output(['bash', '-c', 'source "$1/scripts/load_env.sh"; env -0', 'env', str(ROOT)])
    os.environ.update(dict(item.decode().split('=', 1) for item in loaded.split(b'\0') if b'=' in item))
    BOARD=os.environ.get('FLIGHT_BOARD_IMAGE',BOARD)
    sys.path.insert(0, str(ROOT/'scripts'))
    from robopi import board, clock_offset, hdc, profile
    identities={'flight_board':subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',BOARD],text=True).strip()}
    if args.mixed:identities['compute']=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',args.compute_image],text=True).strip()
    (output/'images.json').write_text(json.dumps(identities,indent=2)+'\n')
    for name in NAMES:
        if subprocess.run(['docker', 'inspect', name], capture_output=True).returncode == 0:
            raise RuntimeError('Existing app container: '+name)
    offset = clock_offset()
    if abs(offset) > .5:
        raise RuntimeError(f'Board clock offset {offset:.3f}s; run scripts/robopi.sh sync-time')
    board(f'test ! -e {REMOTE}/active && mkdir -p {REMOTE}')
    profile(output/'pc-dds.xml', os.environ['PC_IP'])
    profile(output/'board-dds.xml', os.environ['ROBOPI_IP'])
    # MessagePack's pure Python fallback runs on target Python without a PC extension.
    vendor = output/'vendor'
    vendor.mkdir()
    execute(['docker', 'create', '--name', PREFIX+'vendor', BOARD], stdout=subprocess.DEVNULL)
    try:
        execute(['docker', 'cp', PREFIX+'vendor:/usr/lib/python3/dist-packages/msgpack', str(vendor)])
    finally:
        execute(['docker', 'rm', PREFIX+'vendor'], stdout=subprocess.DEVNULL)
    for binary in vendor.rglob('*.so'): binary.unlink()
    mixed_configs={}
    if args.mixed:
        fleet=yaml.safe_load((APP/'fleet.yaml').read_text());deployment=yaml.safe_load((APP/'deployment.yaml').read_text());task=yaml.safe_load((APP/'task.yaml').read_text())
        task['planner']['parameters']['valid_from_lead']=15.0
        for name,value in [('fleet.yaml',{'vehicles':{'drone0':fleet['vehicles']['drone0']}}),('board-deployment.yaml',{'drone0':deployment['drone0']}),('task.yaml',task)]:
            path=output/('mixed-'+name);path.write_text(yaml.safe_dump(value,sort_keys=False));mixed_configs[name]=path
    with tarfile.open(output/'app.tar.gz', 'w:gz', dereference=True, format=tarfile.GNU_FORMAT) as archive:
        for name in ('mission.py', 'stop_board.py', 'task.yaml', 'deployment.yaml', 'fleet.yaml', 'settings.json'):
            archive.add(mixed_configs.get(name,APP/name), arcname=name)
        if args.mixed:
            archive.add(mixed_configs['board-deployment.yaml'],arcname='board-deployment.yaml')
            archive.add(APP/'hybrid/metrics.py',arcname='metrics.py')
        for source, target in [('runtime/as2/run.py', 'run_as2.py'),
                               ('runtime/as2/selected_behaviors.launch.py', 'selected_behaviors.launch.py'),
                               ('runtime/as2/scoped_tf.launch.py', 'scoped_tf.launch.py'),
                               ('runtime/airsim_sensors/uav_as2_sensor_adapter/rpc.py', 'airsim_rpc.py'),
                               ('config/as2/config.yaml', 'config.yaml'),
                               ('config/as2/pid_speed_controller.yaml', 'pid.yaml')]:
            archive.add(ROOT/source, arcname=target)
        archive.add(output/'board-dds.xml', arcname='dds.xml')
        archive.add(vendor/'msgpack', arcname='msgpack')
    for name in ('app.tar.gz',):
        hdc('file', 'send', str(output/name), REMOTE+'/'+name)
    for name in ('app.tar.gz',):
        digest = hashlib.sha256((output/name).read_bytes()).hexdigest()
        check = f"import hashlib; assert hashlib.sha256(open({REMOTE+'/'+name!r},'rb').read()).hexdigest()=={digest!r}"
        board('python3 -c '+shlex.quote(check))
    execute([sys.executable, str(ROOT/'deployment/robopi/deploy_lib.py'), '--component', 'catora'])
    for component,prefix in [('as2','/data/aerohub/as2'),('catora','/data/aerohub/lib/catora')]:
        (output/('installed-'+component+'.json')).write_text(board('cat '+prefix+'/deployment.json')+'\n')
    work=Path(os.environ.get('OHOS_WORK_ROOT','artifacts/ohos-as2'))
    if not work.is_absolute():work=ROOT/work
    inputs=work/'lib-catora-inputs.json'
    if inputs.exists():(output/'build-inputs.json').write_bytes(inputs.read_bytes())
    board(f'tar -xzf {REMOTE}/app.tar.gz -C {REMOTE} && mkdir -p {REMOTE}/results {REMOTE}/runtime')
    values = {'ROS_DOMAIN_ID': '91', 'ROS_LOCALHOST_ONLY': '0', 'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': REMOTE+'/dds.xml', 'CATORA_APP_ROOT': REMOTE,
        'CATORA_OUTPUT': REMOTE+'/results', 'AIRSIM_HOST': os.environ['PC_IP'], 'AIRSIM_PORT': '42473',
        'AS2_LAUNCHER': REMOTE+'/run_as2.py', 'AS2_CONFIG_FILE': REMOTE+'/config.yaml',
        'AS2_CONTROLLER_CONFIG_FILE': REMOTE+'/pid.yaml', 'AS2_FLEET_FILE': REMOTE+'/fleet.yaml',
        'AS2_RUNTIME_DIR': REMOTE+'/runtime', 'AS2_BEHAVIORS':'takeoff,land', 'AS2_RESET_OOM_SCORE': '1', 'AS2_MAX_RSS_MB': '4096',
        'PLANNER_STARTUP_TIMEOUT': '30.0', 'CATORA_PREFLIGHT_ONLY':'1' if args.preflight_only else '0',
        'CATORA_SCENARIO':args.scenario,
        'AS2_TF_SCOPE':args.tf_scope,
        'PLANNER_REGISTRY': '/data/aerohub/lib/catora/install/uav_planner_framework/share/uav_planner_framework/registry',
        'CATORA_TOPOLOGY': 'RoboPi: CAT-ORA + adapters + Planner + AS2; PC: PX4 Docker + AirSim'}
    if args.mixed:
        values.update(CATORA_MIXED='1',CATORA_LOCAL_VEHICLE='drone0',CATORA_COMPUTE_ID='robopi-drone0',
            CATORA_INPUT_DEPLOYMENT=REMOTE+'/board-deployment.yaml',CATORA_METRICS_SCRIPT=REMOTE+'/metrics.py',CATORA_METRICS_FILE=REMOTE+'/results/metrics.jsonl',
            CATORA_TOPOLOGY='RoboPi: drone0 AS2 + global CAT-ORA/Planner; PC: three independent AS2+adapter containers for drone1..3, PX4/AirSim')
    exports = ' '.join(key+'='+shlex.quote(value) for key, value in values.items())
    supervise = ("import subprocess,pathlib,signal; "
                 "log=open('results/mission.log','w'); "
                 "p=subprocess.Popen(['python3','mission.py'],stdout=log,stderr=subprocess.STDOUT); "
                 "pathlib.Path('mission.pid').write_text(str(p.pid)); "
                 "raise SystemExit(p.wait(timeout=650))")
    command = (f'cd {REMOTE} && . /data/aerohub/lib/catora/env.sh && export {exports} && '
               'python3 -c '+shlex.quote(supervise))
    topology=dict(environment=values,clock_offset_s=offset,image=BOARD)
    if args.mixed:
        topology.update(mode='three-pc-one-robopi',trajectory_sampling='central Planner on RoboPi; AS2 control and output adapter local to each compute node',
            configuration_transfer='one-time HDC copy with SHA256; live state/control/status use DDS UDP',
            compute_nodes={'drone0':{'host':'RoboPi','architecture':'aarch64','ip':os.environ['ROBOPI_IP']},
                **{f'drone{i}':{'host':'PC logical container','container':PREFIX+f'compute{i}','architecture':'x86_64','shared_host_ip':os.environ['PC_IP']} for i in range(1,4)}})
    (output/'topology.json').write_text(json.dumps(topology, indent=2)+'\n')
    class Relay(socketserver.BaseRequestHandler):
        def handle(self):
            with socket.create_connection(('127.0.0.1', 42471), timeout=5) as upstream:
                while True:
                    ready, _, _ = select.select([self.request, upstream], [], [], 10)
                    if not ready: return
                    for source in ready:
                        data = source.recv(65536)
                        if not data: return
                        (upstream if source is self.request else self.request).sendall(data)
    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True
    relay = Server((os.environ['PC_IP'], 42473), Relay)
    sim_relays=[]
    workers=None
    if args.mixed:
        from hybrid.orchestrate import HybridWorkers
        workers=HybridWorkers(output,APP,PREFIX,args.compute_image,REMOTE,board,hdc)
    threading.Thread(target=relay.serve_forever, daemon=True).start()
    try:
        execute(manager('start', output)+['--timeout', '90'])
        probe = f'cd {REMOTE} && python3 -c '+shlex.quote("from airsim_rpc import Rpc; print(Rpc("+repr(os.environ['PC_IP'])+",42473).call('simGetVehiclePose','Drone0'))")
        board(probe, timeout=20)
        sim_relays=launch_vehicles(BOARD,APP,output,PREFIX,output/'pc-dds.xml')
        execute(['docker', 'run', '--rm', '--detach', '--init', '--network', 'host', '--name', PREFIX+'clock',
            '-e', 'ROS_DOMAIN_ID=91', '-e', 'VEHICLE_NAME=Drone0', '-e', 'AIRSIM_RPC_PORT=42471',
            '-v', str(output/'pc-dds.xml')+':/opt/fastdds_udp.xml:ro', BOARD, 'clock'], stdout=subprocess.DEVNULL)
        if workers:workers.start()
        board(f'touch {REMOTE}/active')
        print('RoboPi mission running; remote results: '+REMOTE+'/results', flush=True)
        board(command, timeout=700)
    finally:
        try:
            stopped=board(f'python3 {REMOTE}/stop_board.py {REMOTE}', timeout=20)
            (output/'board-cleanup.json').write_text(stopped+'\n')
            board(f'cd {REMOTE} && tar -czf evidence.tar.gz results runtime; rm -f active')
            hdc('file', 'recv', REMOTE+'/evidence.tar.gz', str(output/'evidence.tar.gz'))
            with tarfile.open(output/'evidence.tar.gz') as archive:
                archive.extractall(output, filter='data')
            for name in ('result.json', 'failure.json', 'mission.log'):
                source = output/'results'/name
                if source.exists(): (output/name).write_bytes(source.read_bytes())
        finally:
            relay.shutdown()
            relay.server_close()
            if workers:workers.stop()
            for transport in sim_relays:transport.close()
            for name in NAMES:
                logs=subprocess.run(['docker','logs',name],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
                if logs.returncode==0:(output/(name+'.log')).write_bytes(logs.stdout)
            cleanup(output)
    if args.preflight_only:
        print((output/'results/clock-probe.json').read_text(),flush=True)
        return
    result = json.loads((output/'result.json').read_text())
    assert result['landed_and_disarmed'] and result['execution_arch'] == 'aarch64', result
    if args.scenario!='normal':
        assert result['scenario']==args.scenario and result['as2_hover_confirmed'] and result['planner_status']=='CANCELED',result
        print(json.dumps(result,indent=2),flush=True)
        return
    assert result['planner_status']=='SUCCEEDED' and not result['planning_collisions'],result
    assert max(result['endpoint_error_m'].values())<=.6,result
    assert max(result['airsim_truth_endpoint_error_m'].values())<=.75,result
    assert result['min_observed_separation_m']>=2 and result['min_truth_separation_m']>=2,result
    assert all(result['as2_trajectory_reference_counts'].values()),result
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
