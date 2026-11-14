#!/usr/bin/env python3
"""Own a pure-PC three-aircraft Swarm/AirSim/PX4/AS2 validation."""
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
SCENE=ROOT/'simulator/airsim/scenarios/swarm_open_field'
PREFIX='aerohub-swarm-pc-'
IMAGE=os.environ.get('SWARM_APP_IMAGE','uav/apps-swarm-open-field:main-e2e-followpath')
BASE=os.environ.get('SWARM_BASE_IMAGE','onboard_computer:main-e2e-msgfix')
BOARD=os.environ.get('FLIGHT_BOARD_IMAGE','flight_board:v0.1.0')
DDS=ROOT/'deployment/airsim/config/dds/fastdds_udp.xml'
NAMES=[PREFIX+'app',PREFIX+'clock',PREFIX+'audit'] + [
    PREFIX+f'{kind}{index}' for kind in ('vehicle','sensor') for index in range(3)
] + [PREFIX+f'compute{index}' for index in (1,2)]


def execute(command,**kwargs):
    return subprocess.run(list(map(str,command)),check=True,**kwargs)


def manager(action,output):
    return [sys.executable,ROOT/'simulator/airsim/manager/manager.py',action,
            '--profile',output/'profile.json','--run-dir',output/'scene']


def cleanup(output):
    for name in NAMES:
        if subprocess.run(['docker','container','inspect',name],capture_output=True).returncode:
            continue
        with (output/(name+'.log')).open('w') as stream:
            subprocess.run(['docker','logs',name],stdout=stream,stderr=subprocess.STDOUT)
        subprocess.run(['docker','stop','--timeout','10',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        subprocess.run(['docker','rm',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.run(manager('stop',output),check=False)


def start_worker(output,index):
    vehicle=f'drone{index}'
    root=output/'compute'/vehicle
    root.mkdir(parents=True)
    fleet=yaml.safe_load((APP/'fleet.yaml').read_text())
    (root/'planner.yaml').write_bytes((APP/'planner.yaml').read_bytes())
    (root/'fleet.yaml').write_text(yaml.safe_dump({'vehicles':{vehicle:fleet['vehicles'][vehicle]}}))
    execute(['docker','run','--detach','--init','--network','host','--name',PREFIX+f'compute{index}',
        '-e','ROS_DOMAIN_ID=91','-e','ROS_LOCALHOST_ONLY=0','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml','-e','COMPUTE_ROOT=/run/node',
        '-e','COMPUTE_VEHICLE='+vehicle,'-e','CATORA_COMPUTE_ID=pc-'+vehicle,
        '-e','AS2_EXECUTION_BACKEND=follow_path',
        '-e','AS2_TIMED_ACCELERATION_CONTINUITY_TOLERANCE=1.6','-v',str(root)+':/run/node',
        '-v',str(output/'results')+':/run/resolved:ro','-v',str(DDS)+':/run/dds.xml:ro',IMAGE],
        stdout=subprocess.DEVNULL)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['build','run','stop','status'])
    parser.add_argument('--output',type=Path,default=ROOT/'log/airsim/swarm-pc')
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    if args.action=='build':
        execute(['docker','build','--network','none','--pull=false','--build-arg','BASE_IMAGE='+BASE,
                 '-f',APP/'Dockerfile','-t',IMAGE,ROOT])
        return
    if args.action=='stop':cleanup(output);return
    if args.action=='status':
        subprocess.run(manager('status',output),check=False)
        subprocess.run(['docker','ps','--filter','name='+PREFIX],check=False)
        return
    if any((output/name).exists() for name in ('result.json','failure.json','images.json')):
        raise RuntimeError('choose a fresh output directory')
    for name in NAMES:
        if subprocess.run(['docker','container','inspect',name],capture_output=True).returncode==0:
            raise RuntimeError('reserved container already exists: '+name)
    for image in (IMAGE,BOARD):execute(['docker','image','inspect',image],stdout=subprocess.DEVNULL)
    settings=json.loads((SCENE/'settings.json').read_text())
    (output/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    profile=json.loads((SCENE/'profile.json').read_text());profile['settings']=str(output/'settings.json')
    (output/'profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    results=output/'results';results.mkdir()
    fleet=yaml.safe_load((APP/'fleet.yaml').read_text())
    (output/'coordinator-fleet.yaml').write_text(yaml.safe_dump({'vehicles':{'drone0':fleet['vehicles']['drone0']}}))
    task=yaml.safe_load((APP/'task.yaml').read_text())
    task['planner']['parameters']['wait_for_output']=False
    (output/'task-follow-path.yaml').write_text(yaml.safe_dump(task,sort_keys=False))
    sources=[APP/'mission.py',APP/'worker.py',APP/'task.yaml',
             ROOT/'src/planner/framework/uav_planner_framework/ros_node.py',
             ROOT/'src/behavior/follow_path/src/follow_path_extended.cpp']
    (output/'source-manifest.json').write_text(json.dumps({
        'commit':subprocess.check_output(['git','-C',ROOT,'rev-parse','HEAD'],text=True).strip(),
        'execution_backend':'follow_path','vehicle_count':3,
        'source_hashes':{str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    },indent=2)+'\n')
    sys.path.insert(0,str(ROOT/'deployment/airsim/demos/catora_open_field'))
    from sim_startup import launch_vehicles
    relays=[]
    try:
        execute(manager('start',output)+['--timeout','90'])
        relays=launch_vehicles(BOARD,SCENE,output,PREFIX,DDS,sensors=False)
        execute(['docker','run','--detach','--init','--network','host','--name',PREFIX+'clock',
            '-e','ROS_DOMAIN_ID=91','-e','VEHICLE_NAME=Drone0','-e','AIRSIM_RPC_PORT=42471',BOARD,'clock'],
            stdout=subprocess.DEVNULL)
        for index in range(3):
            command=('source /opt/ros/humble/setup.bash; source /opt/platform_sensors/install/setup.bash; '
                f'exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args -p use_sim_time:=true '
                f'-p rpc_port:=42471 -p vehicle_name:=Drone{index} -p vehicle_namespace:=drone{index} '
                '-p settings_file:=/config/settings.json -p camera_enabled:=false '
                f'-r /tf_static:=/drone{index}/tf_static')
            execute(['docker','run','--detach','--init','--network','host','--name',PREFIX+f'sensor{index}',
                '-e','ROS_DOMAIN_ID=91','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml',
                '-v',str(DDS)+':/run/dds.xml:ro','-v',str(output/'settings.json')+':/config/settings.json:ro',
                '--entrypoint','bash',BOARD,'-c',command],stdout=subprocess.DEVNULL)
        start_worker(output,1);start_worker(output,2)
        execute(['docker','run','--detach','--init','--network','host','--name',PREFIX+'audit',
            '-e','ROS_DOMAIN_ID=91','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml',
            '-v',str(results)+':/results','-v',str(DDS)+':/run/dds.xml:ro',IMAGE,
            'python3','/opt/swarm_app/audit.py'],stdout=subprocess.DEVNULL)
        identities={name:subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
                    for name,image in (('application',IMAGE),('flight_board',BOARD))}
        (output/'images.json').write_text(json.dumps(identities,indent=2)+'\n')
        topology='PC: coordinator/drone0 plus independent drone1/2 compute containers; three PX4/LiDAR; centralized FollowTimedPath coordinator'
        execute(['docker','run','--detach','--init','--network','host','--name',PREFIX+'app',
            '-e','ROS_DOMAIN_ID=91','-e','ROS_LOCALHOST_ONLY=0','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml','-e','CATORA_EXECUTION_BACKEND=follow_path',
            '-e','CATORA_APP_ROOT=/opt/swarm_app','-e','CATORA_OUTPUT=/results',
            '-e','AIRSIM_HOST=127.0.0.1','-e','AIRSIM_PORT=42471','-e','AS2_LAUNCHER=/opt/swarm_app/as2/run.py',
            '-e','AS2_CONFIG_FILE=/opt/swarm_app/config/config.yaml',
            '-e','AS2_CONTROLLER_CONFIG_FILE=/opt/swarm_app/config/pid_speed_controller.yaml',
            '-e','AS2_FLEET_FILE=/run/fleet.yaml','-e','AS2_RUNTIME_DIR=/results/runtime',
            '-e','AS2_TF_SCOPE=per-vehicle','-e','AS2_RESET_OOM_SCORE=1','-e','AS2_MAX_RSS_MB=4096',
            '-e','AS2_TIMED_ACCELERATION_CONTINUITY_TOLERANCE=1.6',
            '-e','PLANNER_TASK_CONFIG=/run/task.yaml','-e','PLANNER_STARTUP_TIMEOUT=30.0',
            '-e','PLANNER_APP_WALL_TIMEOUT=180','-e','PLANNER_REGISTRY=/opt/swarm_ws/src/framework/registry',
            '-e','CATORA_METRICS_SCRIPT=/opt/swarm_app/metrics.py','-e','CATORA_METRICS_FILE=/results/metrics.jsonl',
            '-e','CATORA_COMPUTE_ID=pc-drone0-coordinator',
            '-e','PLANNER_STATE_SOURCE=registered AirSim odometry in each AS2 execution frame',
            '-e','CATORA_TOPOLOGY='+topology,'-v',str(results)+':/results',
            '-v',str(output/'coordinator-fleet.yaml')+':/run/fleet.yaml:ro',
            '-v',str(output/'task-follow-path.yaml')+':/run/task.yaml:ro',
            '-v',str(DDS)+':/run/dds.xml:ro',IMAGE,'python3','/opt/swarm_app/mission.py'],
            stdout=subprocess.DEVNULL)
        status=subprocess.check_output(['docker','wait',PREFIX+'app'],text=True,timeout=700).strip()
        if status!='0':raise RuntimeError('Swarm application exited '+status)
        result=json.loads((results/'result.json').read_text())
        if result.get('planner_status')!='SUCCEEDED' or not result.get('landed_and_disarmed'):
            raise RuntimeError('Swarm mission did not pass')
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        cleanup(output)
        for relay in relays:relay.close()


if __name__=='__main__':main()
