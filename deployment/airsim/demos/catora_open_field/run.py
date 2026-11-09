#!/usr/bin/env python3
"""Own the isolated AirSim scene and containers for this application only."""
import argparse
import hashlib
import os
import json
from pathlib import Path
import subprocess
import sys
from sim_startup import launch_vehicles

ROOT=Path(__file__).resolve().parents[4]
APP=Path(__file__).resolve().parent
PREFIX='aerohub-catora-field-'
IMAGE=os.environ.get('CATORA_APP_IMAGE','uav/apps-catora-open-field:humble')
BOARD='flight_board:v0.1.0'
NAMES=[PREFIX+'app',PREFIX+'clock']+[PREFIX+f'vehicle{i}' for i in range(4)]


def execute(command,**kwargs):
    return subprocess.run(command,check=True,**kwargs)


def manager(action,output):
    return [sys.executable,str(ROOT/'tools/airsim/airsim/manager.py'),action,
            '--profile',str(APP/'profile.json'),'--run-dir',str(output/'scene')]


def cleanup(output):
    for name in NAMES:
        state=subprocess.run(['docker','inspect',name],capture_output=True,text=True)
        if state.returncode:continue
        # Only names reserved for this application are touched.
        with (output/(name+'.log')).open('w') as stream:
            subprocess.run(['docker','logs',name],stdout=stream,stderr=subprocess.STDOUT)
        subprocess.run(['docker','stop','--timeout','7',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.run(manager('stop',output),check=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['build','run','stop','status'])
    parser.add_argument('--output',type=Path,default=ROOT/'log/airsim/catora-open-field')
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    if args.action=='build':
        return execute(['docker','build','-f',str(APP/'Dockerfile'),'-t',IMAGE,str(ROOT)]).returncode
    if args.action=='stop':cleanup(output);return 0
    if args.action=='status':
        subprocess.run(manager('status',output),check=False)
        return execute(['docker','ps','--filter','name='+PREFIX]).returncode
    if any((output/name).exists() for name in ('result.json','failure.json')):
        raise RuntimeError('output contains a previous flight record; choose a new --output directory')
    for name in NAMES:
        if subprocess.run(['docker','container','inspect',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:
            raise RuntimeError('application container already exists: '+name)
    for image in (IMAGE,BOARD):
        execute(['docker','image','inspect',image],stdout=subprocess.DEVNULL)
    status=subprocess.run(manager('status',output),capture_output=True)
    if status.returncode:
        execute(manager('start',output)+['--timeout','90'])
    sources = ['deployment/airsim/demos/catora_open_field/mission.py',
               'src/planner/framework/uav_planner_framework/ros_node.py',
               'src/behavior/follow_path/src/follow_path_extended.cpp']
    manifest = {'commit': subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
                'execution_backend': os.environ.get('CATORA_EXECUTION_BACKEND','legacy_setpoint'),
                'scenario': os.environ.get('CATORA_SCENARIO','normal'),
                'startup_timeout': float(os.environ.get('PLANNER_STARTUP_TIMEOUT','30.0')),
                'source_hashes': {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources}}
    (output/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    relays=[]
    try:
        relays=launch_vehicles(BOARD,APP,output,PREFIX,ROOT/'deployment/airsim/config/dds/fastdds_udp.xml')
        execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'clock',
                 '-e','ROS_DOMAIN_ID=91','-e','VEHICLE_NAME=Drone0','-e','AIRSIM_RPC_PORT=42471',BOARD,'clock'],stdout=subprocess.DEVNULL)
        # Record the actual image identities alongside simulation evidence.
        identities={name:subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
                    for name,image in (('application',IMAGE),('flight_board',BOARD))}
        (output/'images.json').write_text(json.dumps(identities,indent=2)+'\n')
        execute(['docker','run','--rm','--init','--network','host','--name',PREFIX+'app',
                 '-e','ROS_DOMAIN_ID=91','-e','CATORA_EXECUTION_BACKEND='+os.environ.get('CATORA_EXECUTION_BACKEND','legacy_setpoint'),'-e','PLANNER_STARTUP_TIMEOUT='+os.environ.get('PLANNER_STARTUP_TIMEOUT','30.0'),'-e','CATORA_SCENARIO='+os.environ.get('CATORA_SCENARIO','normal'),'-e','AS2_TF_SCOPE=per-vehicle','-v',str(output)+':/results',IMAGE])
    finally:
        cleanup(output)
        for relay in relays:relay.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
