"""Demo-local interfaces overlay in the unchanged shared ROS image."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from robopi_execution import connect

ROOT=Path(__file__).resolve().parents[4]
IMAGE='osrf/ros:humble-desktop-full-jammy'


def prepare(workspace):
    connect()  # Load existing local paths without changing configuration.
    work=Path(os.environ.get('OHOS_WORK_ROOT','log/airsim/ohos-as2'))
    if not work.is_absolute():work=ROOT/work
    workspace.mkdir(parents=True,exist_ok=True)
    source=workspace/'ws/src';source.mkdir(parents=True,exist_ok=True)
    expected=json.loads((work/'current-sources.json').read_text())['source_hashes']
    packages={
        'as2_msgs':work/'ws/src/as2_msgs',
        'px4_msgs':work/'ws/src/px4_msgs',
        'mars_quadrotor_msgs':ROOT/'src/tool/navigation/planner/super/ros2_adapter/native/mars_uav_sim/mars_quadrotor_msgs',
        'uav_planner_interfaces':ROOT/'src/tool/navigation/interfaces'}
    manifest={}
    for name,path in packages.items():
        if name in expected:
            for relative,digest in expected[name].items():
                if hashlib.sha256((path/relative).read_bytes()).hexdigest()!=digest:
                    raise RuntimeError('Prepared interface source differs from manifest: '+name+'/'+relative)
        target=source/name
        if target.exists():shutil.rmtree(target)
        shutil.copytree(path,target,ignore=shutil.ignore_patterns('.git','__pycache__','build','install','log'))
        manifest[name]={str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in target.rglob('*') if p.is_file()}
    # Rebuild canonical interface definitions from the existing target dependency snapshot;
    # never copy its AArch64 binaries into the workstation.
    geographic=work/'root/install/share/geographic_msgs'
    target=source/'geographic_msgs'
    target.mkdir(exist_ok=True)
    shutil.copyfile(geographic/'package.xml',target/'package.xml')
    for directory in ('msg','srv'):
        if (target/directory).exists():shutil.rmtree(target/directory)
        shutil.copytree(geographic/directory,target/directory,ignore=shutil.ignore_patterns('*.idl'))
    (target/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.5)
project(geographic_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(dep geometry_msgs std_msgs unique_identifier_msgs)
  find_package(${dep} REQUIRED)
endforeach()
file(GLOB messages RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg" "srv/*.srv")
rosidl_generate_interfaces(${PROJECT_NAME} ${messages} DEPENDENCIES geometry_msgs std_msgs unique_identifier_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
''')
    manifest['geographic_msgs']={'source_kind':'installed 1.0.6 interface definitions; generated local build glue',
        'files':{str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest() for p in target.rglob('*') if p.is_file()}}
    runtime=workspace/'runtime';runtime.mkdir(exist_ok=True)
    shutil.copyfile(ROOT/'simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py',runtime/'airsim_rpc.py')
    manifest['image']=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',IMAGE],text=True).strip()
    (workspace/'inputs.json').write_text(json.dumps(manifest,indent=2)+'\n')
    command=['docker','run','--rm','--init','--network','none','--user',f'{os.getuid()}:{os.getgid()}',
             '-e','PYTHONDONTWRITEBYTECODE=1','-v',str(workspace)+':/work','-w','/work/ws',
             '--entrypoint','/bin/bash',IMAGE,'-lc',
             'source /opt/ros/humble/setup.bash && colcon build --merge-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF']
    subprocess.run(command,check=True)
    print('Workstation message overlay prepared; shared image unchanged')


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True)
    prepare(parser.parse_args().workspace.resolve())
