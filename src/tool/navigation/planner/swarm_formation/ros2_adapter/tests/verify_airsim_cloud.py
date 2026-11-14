#!/usr/bin/env python3
"""Own a stationary, real AirSim/PX4/AS2 LiDAR acceptance instance."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import yaml

ROOT=Path(__file__).resolve().parents[7]
BOARD='flight_board:v0.1.0'
PREFIX='aerohub-swarm-cloud-'
NAMES=[PREFIX+n for n in ('validation','as2','clock','vehicle')]


def run(args,**kwargs):return subprocess.run(args,check=True,**kwargs)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'log/airsim/swarm-airsim-cloud')
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if (out/'result.json').exists():raise RuntimeError('choose a fresh output directory to preserve evidence')
    for name in NAMES:
        if subprocess.run(['docker','inspect',name],capture_output=True).returncode==0:
            raise RuntimeError('validation container already exists: '+name)
    settings=json.loads((ROOT/'simulator/airsim/config/settings/px4-lidar.json').read_text())
    settings['ApiServerPort']=42472
    settings['Vehicles']['PX4'].update(TcpPort=4684,ControlPortLocal=14844,ControlPortRemote=14984,X=0,Y=4,Z=0)
    (out/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    (out/'fleet.yaml').write_text(yaml.safe_dump({'vehicles':{'drone0':{'fmu_prefix':'/drone0','target_system_id':45}}}))
    profile=json.loads((ROOT/'simulator/airsim/config/nh.json').read_text());profile['settings']=str(out/'settings.json')
    (out/'profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    manager=[sys.executable,str(ROOT/'simulator/airsim/manager/manager.py')]
    run(manager+['start','--profile',str(out/'profile.json'),'--run-dir',str(out/'scene'),'--timeout','90'])
    try:
        run(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'vehicle',
             '-e','ROS_DOMAIN_ID=92','-e','PX4_INSTANCE=44','-e','PX4_SYSTEM_ID=45','-e','XRCE_AGENT_PORT=8905',
             '-e','AIRSIM_RPC_PORT=42472','-v',str(out/'settings.json')+':/config/settings.json:ro',BOARD,'vehicle'])
        run(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'clock',
             '-e','ROS_DOMAIN_ID=92','-e','AIRSIM_RPC_PORT=42472',BOARD,'clock'])
        run(['docker','run','--rm','--detach','--init','--network','host','--name',PREFIX+'as2',
             '-e','ROS_DOMAIN_ID=92','-e','AS2_FLEET_FILE=/config/fleet.yaml',
             '-v',str(out/'fleet.yaml')+':/config/fleet.yaml:ro','onboard_computer:v0.1.0'])
        run(['docker','run','--rm','--init','--network','host','--name',PREFIX+'validation',
             '-e','ROS_DOMAIN_ID=92','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/work/config/dds/fastdds_udp.xml',
             '-v',str(ROOT)+':/work:ro','-v',str(out)+':/results','--entrypoint','bash',
             'osrf/ros:humble-desktop-full-jammy','-lc', '''
set -e
source /opt/ros/humble/setup.bash
mkdir -p /results/workspace
cd /results/workspace
MAKEFLAGS=-j2 colcon build --base-paths /work/src/tool/navigation/planner/swarm_formation/ros2_adapter/native /work/src/basic_function/perception/navigation_inputs \
  --packages-up-to plan_env as2_navigation_inputs --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DSWARM_BUILD_VALIDATION=ON > build.log 2>&1
source install/setup.bash
ros2 run as2_navigation_inputs navigation_inputs_node --ros-args -r __ns:=/drone0 \
  --params-file /work/src/basic_function/perception/navigation_inputs/config/drone0.yaml > /results/navigation.log 2>&1 & nav_pid=$!
trap 'kill -TERM "$nav_pid" 2>/dev/null || true; wait "$nav_pid" || true' EXIT
python3 /work/src/tool/navigation/planner/swarm_formation/ros2_adapter/tests/verify_real_cloud.py --output /results
'''])
    finally:
        for name in NAMES:
            if subprocess.run(['docker','inspect',name],capture_output=True).returncode:
                continue
            with (out/(name+'.log')).open('w') as stream:
                subprocess.run(['docker','logs',name],stdout=stream,stderr=subprocess.STDOUT)
            subprocess.run(['docker','stop','--timeout','7',name],capture_output=True)
        subprocess.run(manager+['stop','--run-dir',str(out/'scene')],check=False)
    return 0


if __name__=='__main__':raise SystemExit(main())
