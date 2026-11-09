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
import yaml

ROOT=Path(__file__).resolve().parents[4]
APP=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'deployment/airsim/demos/catora_open_field'))
from sim_startup import StartupRelay
PREFIX='aerohub-super-migration-'
NAMES=[PREFIX+'app',PREFIX+'clock',PREFIX+'vehicle']
IMAGE=os.environ.get('SUPER_APP_IMAGE','uav/apps-super-forest:migration-main-behavior')
BOARD='flight_board:v0.1.0'


def run(command,**kwargs):return subprocess.run(list(map(str,command)),check=True,**kwargs)


def manager(action,out):
    return [sys.executable,str(ROOT/'simulator/airsim/manager/manager.py'),action,
            '--profile',str(out/'profile.json'),'--run-dir',str(out/'scene')]


def cleanup(out):
    for name in NAMES:
        if subprocess.run(['docker','container','inspect',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
            continue
        with (out/(name+'.log')).open('w') as log:
            subprocess.run(['docker','logs',name],stdout=log,stderr=subprocess.STDOUT)
        subprocess.run(['docker','stop','--timeout','10',name],stdout=subprocess.DEVNULL)
    subprocess.run(manager('stop',out),check=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['build','run','stop','status'])
    parser.add_argument('--output',type=Path,default=ROOT/'log/airsim/super-migration-pc')
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if args.action=='build':
        run(['docker','build','--network','none','--pull=false','-f',APP/'Dockerfile','-t',IMAGE,ROOT]);return
    if args.action=='stop':cleanup(out);return
    if args.action=='status':
        subprocess.run(manager('status',out),check=False);run(['docker','ps','--filter','name='+PREFIX]);return
    if (out/'result.json').exists() or (out/'images.json').exists():raise RuntimeError('Choose a fresh output directory')
    for name in NAMES:
        if not subprocess.run(['docker','container','inspect',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
            raise RuntimeError('Reserved container already exists: '+name)
    for image in (IMAGE,BOARD):run(['docker','image','inspect',image],stdout=subprocess.DEVNULL)
    settings=json.loads((ROOT/'simulator/airsim/scenarios/super_forest/settings.json').read_text())
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
            (APP,ROOT/'src/planner/super/ros2_adapter',ROOT/'src/behavior/follow_path',
             ROOT/'src/planner/framework',ROOT/'deployment/airsim/as2',ROOT/'deployment/airsim/config/as2',ROOT/'deployment/platform/patches') for p in directory.rglob('*')
            if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.png','.log')}
    (out/'source-manifest.json').write_text(json.dumps({'commit':subprocess.check_output(['git','-C',ROOT,'rev-parse','HEAD'],text=True).strip(),
        'super_commit':subprocess.check_output(['git','-C',ROOT/'src/planner/super/upstream','rev-parse','HEAD'],text=True).strip(),
        'submodules':{name:subprocess.check_output(['git','-C',ROOT/name,'rev-parse','HEAD'],text=True).strip()
                      for name in ('as2/aerostack2','as2/as2_platform_pixhawk','src/planner/super/upstream')},
        'source_hashes':hashes,'ports':{'rpc':43472,'tcp':4881,'relay':4981,'agent':8951},'ros_domain':94},indent=2)+'\n')
    relay=None
    try:
        run(manager('start',out)+['--timeout','90'])
        relay=StartupRelay(('127.0.0.1',4981),('127.0.0.1',4881))
        run(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'vehicle',
             '-e','ROS_DOMAIN_ID=94','-e','VEHICLE_NAME=PX4','-e','VEHICLE_NAMESPACE=drone0',
             '-e','PX4_INSTANCE=60','-e','PX4_SYSTEM_ID=61','-e','XRCE_AGENT_PORT=8951',
             '-e','AIRSIM_RPC_PORT=43472','-e','FLIGHT_BOARD_SENSORS=1','-e','FLIGHT_BOARD_CAMERAS=1',
             '-e','PX4_PARAM_MPC_XY_VEL_MAX=16','-e','PX4_PARAM_SENS_GPS0_DELAY=200',
             '-v',str(out/'px4-settings.json')+':/config/settings.json:ro',BOARD,'vehicle'],stdout=subprocess.DEVNULL)
        run(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'clock',
             '-e','ROS_DOMAIN_ID=94','-e','VEHICLE_NAME=PX4','-e','AIRSIM_RPC_PORT=43472',BOARD,'clock'],stdout=subprocess.DEVNULL)
        deadline=time.monotonic()+60
        while True:
            log=subprocess.check_output(['docker','logs',PREFIX+'vehicle'],stderr=subprocess.STDOUT,text=True)
            if 'Ready for takeoff!' in log:
                (out/'px4-startup.json').write_text(json.dumps({'ready':True,'initial_tcp_buffer_seconds':.1,'tcp_nodelay':True})+'\n')
                break
            if time.monotonic()>deadline:raise RuntimeError('PX4 preflight did not become ready')
            time.sleep(.25)
        (out/'images.json').write_text(json.dumps({name:subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
            for name,image in [('application',IMAGE),('flight_board',BOARD)]},indent=2)+'\n')
        run(['docker','run','--rm','--init','--network','host','--name',PREFIX+'app',
             '-e','ROS_DOMAIN_ID=94','-v',str(out)+':/evidence',IMAGE])
        result=json.loads((out/'result.json').read_text())
        if not result.get('success'):raise RuntimeError('SUPER mission did not pass')
    finally:
        cleanup(out)
        if relay:relay.close()

if __name__=='__main__':main()
