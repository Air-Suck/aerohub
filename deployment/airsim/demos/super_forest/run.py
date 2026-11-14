#!/usr/bin/env python3
"""Own a separate PC SUPER/AirSim instance; never operate the historical runtime."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import socket
import fcntl
import signal
from robopi_execution import BoardExecution
import yaml

ROOT=Path(__file__).resolve().parents[4]
APP=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'deployment/airsim/demos/catora_open_field'))
PREFIX='aerohub-super-migration-'
NAMES=[PREFIX+'app',PREFIX+'clock',PREFIX+'vehicle']
IMAGE=os.environ.get('SUPER_APP_IMAGE','uav/apps-super-forest:migration-main-behavior')
BOARD='flight_board:v0.1.0'
WORKSTATION_IMAGE='osrf/ros:humble-desktop-full-jammy'


def run(command,**kwargs):return subprocess.run(list(map(str,command)),check=True,**kwargs)


def manager(action,out):
    return [sys.executable,str(ROOT/'simulator/manager/manager.py'),action,
            '--profile',str(out/'profile.json'),'--run-dir',str(out/'scene')]


def owned(name,out):
    value=subprocess.check_output(['docker','inspect','--format','{{index .Config.Labels "aerohub.super.output"}}',name],text=True).strip()
    if value!=str(out):raise RuntimeError('Container belongs to another run: '+name)


def start_relay(out,host='127.0.0.1'):
    with (out/'relay.log').open('w') as stream:
        child=subprocess.Popen([sys.executable,str(APP/'relay.py'),'--output',str(out),'--listen-host',host],
            stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+5
    while not (out/'relay-state.json').exists():
        if child.poll() is not None:raise RuntimeError('Relay exited; inspect relay.log')
        if time.monotonic()>deadline:raise RuntimeError('Relay startup timed out')
        time.sleep(.05)


def stop_relay(out):
    path=out/'relay-state.json'
    if not path.exists():return
    state=json.loads(path.read_text());pid=state['pid']
    def alive():
        try:
            text=Path(f'/proc/{pid}/stat').read_text();fields=text[text.rfind(')')+2:].split()
            return (fields[0]!='Z' and fields[19]==state['ticks'] and
                Path('/proc/sys/kernel/random/boot_id').read_text().strip()==state['boot_id'] and
                str(APP/'relay.py').encode() in Path(f'/proc/{pid}/cmdline').read_bytes().split(bytes([0])))
        except OSError:return False
    if alive():os.kill(pid,signal.SIGTERM)
    deadline=time.monotonic()+5
    while alive() and time.monotonic()<deadline:time.sleep(.1)
    if alive():raise RuntimeError('Owned relay did not stop')
    (out/'relay-stopped.json').write_text(json.dumps({'pid':pid,'alive':False})+'\n')


def cleanup(out):
    for name in NAMES:
        if subprocess.run(['docker','container','inspect',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
            continue
        owned(name,out)
        with (out/(name+'.log')).open('w') as log:
            subprocess.run(['docker','logs',name],stdout=log,stderr=subprocess.STDOUT)
        subprocess.run(['docker','stop','--timeout','10',name],stdout=subprocess.DEVNULL)
    subprocess.run(manager('stop',out),check=False)
    stop_relay(out)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['build','run','stop','status'])
    parser.add_argument('--case', choices=['ground','short','short-cancel','short-input-loss','short-takeover','forest8'], default='forest8')
    parser.add_argument('--execution-location', choices=['pc','robopi','robopi-all','four-part'], default='pc')
    parser.add_argument('--workstation-workspace',type=Path,default=ROOT/'log/airsim/super-four-part/workstation')
    parser.add_argument('--output',type=Path,default=ROOT/'log/airsim/super-migration-pc')
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if args.action=='build' and args.execution_location=='four-part':
        from workstation import prepare
        prepare(args.workstation_workspace.resolve());return
    if args.action=='build':
        run(['docker','build','--network','none','--pull=false','-f',APP/'Dockerfile','-t',IMAGE,ROOT]);return
    if args.action=='stop':
        coordinator=out/'coordinator.json'
        if coordinator.exists():
            state=json.loads(coordinator.read_text())
            if state.get('execution_location') in ('robopi-all','four-part'):
                try:
                    text=Path(f"/proc/{state['pid']}/stat").read_text();fields=text[text.rfind(')')+2:].split()
                    current=(fields[0]!='Z' and fields[19]==state['ticks'] and Path('/proc/sys/kernel/random/boot_id').read_text().strip()==state['boot_id'])
                except OSError:current=False
                if current:
                    os.kill(state['pid'],signal.SIGINT)
                    print('Recovery requested from the owning coordinator; inspect status and result before cleanup')
                    return
        running=subprocess.run(['docker','inspect','--format','{{.State.Running}}',PREFIX+'app'],
            capture_output=True,text=True)
        if running.returncode==0 and running.stdout.strip()=='true':
            owned(PREFIX+'app',out)
            run(['docker','kill','--signal=SIGINT',PREFIX+'app'])
            deadline=time.monotonic()+150
            while subprocess.run(['docker','container','inspect',PREFIX+'app'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:
                if time.monotonic()>deadline:raise RuntimeError('Mission recovery timed out; resources retained')
                time.sleep(1)
        if (out/'execution.json').exists():
            execution=BoardExecution(out);execution.started=True
            if (out/'board-compute-start.json').exists():execution.stop_compute()
            execution.stop()
        cleanup(out);return
    if args.action=='status':
        if (out/'execution.json').exists():
            execution=BoardExecution(out)
            print(execution.link.board(execution.env+'python3 manage_runtime.py status || true'))
        subprocess.run(manager('status',out),check=False);run(['docker','ps','--filter','name='+PREFIX]);return
    lock=(ROOT/'log/airsim/super-execution.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    stat=Path('/proc/self/stat').read_text();ticks=stat[stat.rfind(')')+2:].split()[19]
    (out/'coordinator.json').write_text(json.dumps({'pid':os.getpid(),'ticks':ticks,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'execution_location':args.execution_location})+'\n')
    if (out/'result.json').exists() or (out/'images.json').exists():raise RuntimeError('Choose a fresh output directory')
    for name in NAMES:
        if not subprocess.run(['docker','container','inspect',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
            raise RuntimeError('Reserved container already exists: '+name)
    for image in ((BOARD,WORKSTATION_IMAGE) if args.execution_location=='four-part' else ((BOARD,) if args.execution_location=='robopi-all' else (IMAGE,BOARD))):run(['docker','image','inspect',image],stdout=subprocess.DEVNULL)
    execution = BoardExecution(out) if args.execution_location != 'pc' else None
    settings=json.loads((ROOT/'simulator/airsim/scenarios/super_forest/settings.json').read_text())
    if args.execution_location=='robopi-all':settings['LocalHostIp']=execution.link.required('PC_IP')
    settings['ApiServerPort']=43472
    vehicle=settings['Vehicles']['PX4']
    vehicle.update(TcpPort=4881,ControlPortLocal=15641,ControlPortRemote=15681)
    (out/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    px4_settings=json.loads(json.dumps(settings));px4_settings['Vehicles']['PX4']['TcpPort']=4981
    (out/'px4-settings.json').write_text(json.dumps(px4_settings,indent=2)+'\n')
    profile=json.loads((ROOT/'simulator/airsim/scenarios/catora_open_field/profile.json').read_text())
    profile['settings']=str(out/'settings.json')
    (out/'profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    (out/'fleet.yaml').write_text(yaml.safe_dump({'vehicles':{'drone0':{'fmu_prefix':'/drone0','target_system_id':61}}}))
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for directory in
            (APP,ROOT/'src/tool/navigation/planner/super/ros2_adapter',ROOT/'src/behavior/follow_path',
             ROOT/'src/tool/navigation/framework',ROOT/'deployment/airsim/as2',ROOT/'deployment/airsim/config/as2',ROOT/'deployment/platform/patches') for p in directory.rglob('*')
            if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.png','.log')}
    (out/'source-manifest.json').write_text(json.dumps({'commit':subprocess.check_output(['git','-C',ROOT,'rev-parse','HEAD'],text=True).strip(),
        'super_source':{'path':'src/tool/navigation/planner/super/ros2_adapter/native','owner':'main repository',
                        'import':json.loads((ROOT/'src/tool/navigation/planner/super/ros2_adapter/native/SOURCE.json').read_text())},
        'locked_submodules':{name:subprocess.check_output(['git','-C',ROOT,'ls-tree','HEAD',name],text=True).split()[2]
                      for name in ('as2/aerostack2','as2/as2_platform_pixhawk','src/tool/navigation/planner/super/upstream')},
        'source_hashes':hashes,'ports':{'rpc':43472,'tcp':4881,'relay':4981,'agent':8951},'ros_domain':94},indent=2)+'\n')
    if execution:
        execution.require_super=args.execution_location in ('robopi-all','four-part')
        execution.prepare()
    # Refuse occupied independent ports before starting any simulation resources.
    for kind, ports in ((socket.SOCK_STREAM,(43472,4881,4981)),(socket.SOCK_DGRAM,(8951,15641,15681))):
        for port in ports:
            with socket.socket(socket.AF_INET,kind) as probe:
                if kind==socket.SOCK_STREAM:probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                probe.bind(('0.0.0.0',port))
    dds = ['-e','FASTRTPS_DEFAULT_PROFILES_FILE=/evidence/pc-dds.xml',
           '-e','ROS_LOCALHOST_ONLY=0','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp','-v',str(out)+':/evidence:ro'] if execution else []
    if args.execution_location=='robopi-all':dds+=['-e','AIRSIM_HOST='+execution.link.required('PC_IP')]
    try:
        run(manager('start',out)+['--timeout','90'])
        start_relay(out,execution.link.required('PC_IP') if args.execution_location=='robopi-all' else '127.0.0.1')
        run(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'vehicle','--label','aerohub.super.output='+str(out),
             '-e','ROS_DOMAIN_ID=94','-e','VEHICLE_NAME=PX4','-e','VEHICLE_NAMESPACE=drone0',
             '-e','PX4_INSTANCE=60','-e','PX4_SYSTEM_ID=61','-e','XRCE_AGENT_PORT=8951',
             '-e','AIRSIM_RPC_PORT=43472','-e','FLIGHT_BOARD_SENSORS=1','-e','FLIGHT_BOARD_CAMERAS=1',
             *dds,'-e','PX4_PARAM_MPC_XY_VEL_MAX=16','-e','PX4_PARAM_SENS_GPS0_DELAY=200',
             '-v',str(out/'px4-settings.json')+':/config/settings.json:ro',BOARD,'vehicle'],stdout=subprocess.DEVNULL)
        run(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'clock','--label','aerohub.super.output='+str(out),
             '-e','ROS_DOMAIN_ID=94','-e','VEHICLE_NAME=PX4','-e','AIRSIM_RPC_PORT=43472',*dds,BOARD,'clock'],stdout=subprocess.DEVNULL)
        deadline=time.monotonic()+60
        while True:
            log=subprocess.check_output(['docker','logs',PREFIX+'vehicle'],stderr=subprocess.STDOUT,text=True)
            if 'Ready for takeoff!' in log:
                (out/'px4-startup.json').write_text(json.dumps({'ready':True,'initial_tcp_buffer_seconds':.1,'tcp_nodelay':True})+'\n')
                break
            if time.monotonic()>deadline:raise RuntimeError('PX4 preflight did not become ready')
            time.sleep(.25)
        (out/'images.json').write_text(json.dumps({name:subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
            for name,image in ([('workstation_base',WORKSTATION_IMAGE),('flight_board',BOARD)] if args.execution_location=='four-part' else ([('flight_board',BOARD)] if args.execution_location=='robopi-all' else [('application',IMAGE),('flight_board',BOARD)]))},indent=2)+'\n')
        if execution: execution.start()
        if args.execution_location=='robopi-all':
            execution.run_mission(args.case)
        else:
            if args.execution_location=='four-part':
                workspace=args.workstation_workspace.resolve()
                if not (workspace/'ws/install/setup.bash').exists():raise RuntimeError('Prepare the workstation interface overlay first')
                try:execution.start_compute()
                except BaseException as exc:
                    (out/'result.json').write_text(json.dumps({'success':False,'motion_requested':False,'error':str(exc),'failed_phase':'compute_startup'})+'\n')
                    raise
                command=['docker','run','--rm','--init','--network','host','--name',PREFIX+'app',
                    '--label','aerohub.super.output='+str(out),'--user',f'{os.getuid()}:{os.getgid()}',
                    '-e','ROS_DOMAIN_ID=94','-e','ROS_LOCALHOST_ONLY=0','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
                    '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/evidence/pc-dds.xml',
                    '-e','PYTHONDONTWRITEBYTECODE=1','-e','ROS_LOG_DIR=/evidence/workstation-roslog',
                    '-e','SUPER_EXECUTION_LOCATION=four-part','-e','SUPER_CASE='+args.case,
                    '-e','SUPER_APP_ROOT=/app','-e','PYTHONPATH=/work/runtime:/vendor',
                    '-v',str(workspace)+':/work:ro','-v',str(APP)+':/app:ro',
                    '-v',str(ROOT/'deployment/robopi/vendor')+':/vendor:ro','-v',str(out)+':/evidence',
                    '--entrypoint','/bin/bash',WORKSTATION_IMAGE,'-lc',
                    'source /opt/ros/humble/setup.bash && source /work/ws/install/setup.bash && exec python3 /app/mission.py']
            else:
                command=['docker','run','--rm','--init','--network','host','--name',PREFIX+'app','--label','aerohub.super.output='+str(out),
                     '-e','ROS_DOMAIN_ID=94','-e','SUPER_EXECUTION_LOCATION='+args.execution_location,'-e','SUPER_CASE='+args.case,
                     *(['-e','FASTRTPS_DEFAULT_PROFILES_FILE=/evidence/pc-dds.xml','-e','ROS_LOCALHOST_ONLY=0'] if execution else []),
                     '-v',str(out)+':/evidence',IMAGE]
            app=subprocess.Popen(list(map(str,command)))
            try:
                while app.poll() is None:
                    if execution: execution.check()
                    time.sleep(2)
            except BaseException:
                # Let the mission execute its existing stop/landing recovery before cleanup.
                subprocess.run(['docker','kill','--signal=SIGINT',PREFIX+'app'],check=False)
                try:app.wait(timeout=120)
                except subprocess.TimeoutExpired:pass
                raise
            if app.returncode:raise RuntimeError('Application exited: '+str(app.returncode))
            result=json.loads((out/'result.json').read_text())
            if not result.get('success'):raise RuntimeError('SUPER mission did not pass')
    finally:
        # Retain simulator, clock and board control if recovery has not been confirmed.
        if execution:
            if args.execution_location=='four-part':execution.stop_compute()
            execution.stop()
        cleanup(out)

if __name__=='__main__':main()
