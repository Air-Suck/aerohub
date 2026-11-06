#!/usr/bin/env python3
"""Own the isolated AirSim scene and containers for this application only."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
APP=Path(__file__).resolve().parent
PREFIX='aerohub-catora-field-'
IMAGE='uav/apps-catora-open-field:humble'
BOARD='flight_board:v0.1.0'
NAMES=[PREFIX+'app',PREFIX+'clock']+[PREFIX+f'vehicle{i}' for i in range(4)]


def execute(command,**kwargs):
    return subprocess.run(command,check=True,**kwargs)


def manager(action,output):
    return [sys.executable,str(ROOT/'scripts/airsim/manager.py'),action,
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
    parser.add_argument('--output',type=Path,default=ROOT/'artifacts/catora-open-field')
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
    try:
        for index in range(4):
            command=['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+f'vehicle{index}',
                '-e','ROS_DOMAIN_ID=91','-e',f'VEHICLE_NAME=Drone{index}',
                '-e',f'VEHICLE_NAMESPACE=drone{index}','-e',f'PX4_INSTANCE={40+index}',
                '-e',f'PX4_SYSTEM_ID={41+index}','-e',f'XRCE_AGENT_PORT={8901+index}',
                '-e','AIRSIM_RPC_PORT=42471','-e','FLIGHT_BOARD_SENSORS=0',
                '-v',str(APP/'settings.json')+':/config/settings.json:ro',BOARD,'vehicle']
            execute(command,stdout=subprocess.DEVNULL)
        execute(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'clock',
                 '-e','ROS_DOMAIN_ID=91','-e','VEHICLE_NAME=Drone0','-e','AIRSIM_RPC_PORT=42471',BOARD,'clock'],stdout=subprocess.DEVNULL)
        # Record the actual image identities alongside simulation evidence.
        identities={name:subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
                    for name,image in (('application',IMAGE),('flight_board',BOARD))}
        (output/'images.json').write_text(json.dumps(identities,indent=2)+'\n')
        execute(['docker','run','--rm','--init','--network','host','--name',PREFIX+'app',
                 '-e','ROS_DOMAIN_ID=91','-v',str(output)+':/results',IMAGE])
    finally:
        cleanup(output)
    return 0


if __name__=='__main__':raise SystemExit(main())
