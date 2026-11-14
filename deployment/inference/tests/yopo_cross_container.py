"""Requires an already-ready CUDA worker on host port 18081; owns only its Adapter container."""
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT/'log/yopo-single'
GROUND = 'osrf/ros:humble-desktop-full-jammy'
ONBOARD = 'onboard_computer:v0.1.0'


def run(args, timeout=120):
    return subprocess.check_output(args, stderr=subprocess.STDOUT, text=True, timeout=timeout)


def main():
    name = 'yopo-probe-'+str(os.getpid())
    report=OUT/'model-chain'/str(os.getpid())
    report.mkdir(parents=True)
    common = ['docker', 'run', '--init', '--network', 'host', '-e', 'ROS_DOMAIN_ID=197',
              '-e', 'ROS_LOCALHOST_ONLY=1', '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml',
              '-v', str(ROOT)+':/repo:ro', '--entrypoint', 'bash']
    manifest = dict(domain=197, transport='host UDP, localhost only; separate containers',
                    worker='host CUDA worker on 127.0.0.1:18081',
                    overlays=dict(onboard='/repo/log/yopo-single/ros_ws/install', ground='/repo/log/yopo-single/ground_ws/install'))
    for role, image in [('ground', GROUND), ('onboard', ONBOARD)]:
        manifest[role+'_image_id'] = run(['docker','image','inspect',image,'--format','{{.Id}}']).strip()
    (report/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (OUT/'ground_ws').mkdir(exist_ok=True)
    build = ['docker','run','--rm','--network','none','-v',str(ROOT)+':/repo',
             '-w','/repo/log/yopo-single/ground_ws','--entrypoint','bash',GROUND,'-lc',
             'source /opt/ros/humble/setup.bash && colcon build --base-paths /repo/src/basic_function/inference/interfaces --packages-select uav_inference_interfaces --event-handlers console_direct+']
    (report/'ground-build.txt').write_text(run(build))
    created = False
    try:
        run(common[:2]+['-d','--name',name]+common[2:]+[ONBOARD,'-lc',
            'source /opt/ros/humble/setup.bash && source /repo/log/yopo-single/ros_ws/install/setup.bash && '
            'exec python3 -m uav_inference.yopo_node --ros-args -r __ns:=/drone0 -p endpoint:=http://127.0.0.1:18081'])
        created = True
        result = run(common[:2]+['--rm']+common[2:]+[GROUND,'-lc',
                     'source /opt/ros/humble/setup.bash && source /repo/log/yopo-single/ground_ws/install/setup.bash && '
                     'python3 /repo/deployment/inference/tests/yopo_remote_probe.py'])
        (report/'result.json').write_text(result)
        print(result)
        print('Evidence:',report.relative_to(ROOT))
    finally:
        if created:
            (report/'adapter.txt').write_text(run(['docker','logs',name]))
            run(['docker','stop','--timeout','10',name],timeout=20)
            run(['docker','rm',name])


if __name__ == '__main__':
    main()
