"""Isolated inference/Tool integration check; no AirSim, PX4 or motion commands."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import uuid


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--worker-image',required=True,help='Linux Python3.10/CUDA-compatible image; model venv must already be prepared')
    parser.add_argument('--ros-image',default='ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.0')
    parser.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[3])
    parser.add_argument('--ros-workspace',default='log/inference/yolo26n_seg_ros_ws')
    parser.add_argument('--output',default='log/inference/yolo26n-seg-service')
    args=parser.parse_args();repo=args.repo.resolve();image=args.image.resolve()
    output=(repo/args.output).resolve();workspace=(repo/args.ros_workspace).resolve()
    if not output.is_relative_to(repo) or not workspace.is_relative_to(repo):
        raise ValueError('Output and ROS workspace must stay inside the selected repository')
    if not image.is_file() or not (workspace/'install/setup.bash').is_file():
        raise ValueError('Recorded RGB and built ROS overlay are required')
    output.mkdir(parents=True,exist_ok=True)
    token=uuid.uuid4().hex[:10];network='yolo26n-seg-check-'+token
    ids=[];created_network=False

    def run(argv,**kw):
        return subprocess.run(argv,check=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                              encoding='utf-8',errors='replace',**kw).stdout.strip()

    def start(role,container_image,command,gpu=False,fixture=False):
        argv=['docker','run','-d','--name',network+'-'+role,'--label','aerohub.yolo-check='+token,
              '--network',network,'--entrypoint','bash','-e','ROS_DOMAIN_ID=226',
              '-e','ROS_LOCALHOST_ONLY=0','-e','PYTHONDONTWRITEBYTECODE=1',
              '-v',str(repo)+':/repo','-w','/tmp']
        if gpu:argv+=['--gpus','all']
        if fixture:argv+=['-v',str(image.parent)+':/fixture:ro']
        identifier=run(argv+[container_image,'-lc',command]);ids.append((role,identifier));return identifier

    overlay='/repo/'+workspace.relative_to(repo).as_posix()+'/install/setup.bash'
    evidence='/repo/'+output.relative_to(repo).as_posix()
    source='source /opt/ros/humble/setup.bash; source '+overlay+'; '
    try:
        run(['docker','network','create','--label','aerohub.yolo-check='+token,network]);created_network=True
        worker=start('worker',args.worker_image,
            'export YOLO_CONFIG_DIR=/repo/src/inference/models/yolo26n_seg/artifacts/ultralytics; '
            'INFERENCE_HOST=0.0.0.0 bash /repo/deployment/inference/start_yolo26n_seg.sh',gpu=True)
        deadline=time.monotonic()+60;metadata=None
        while time.monotonic()<deadline:
            try:
                metadata=json.loads(run(['docker','exec',worker,'/usr/bin/python3','-c',
                    'import urllib.request; print(urllib.request.urlopen("http://127.0.0.1:8002/health/ready",timeout=1).read().decode())']))
                break
            except (subprocess.CalledProcessError,json.JSONDecodeError):time.sleep(.25)
        if metadata is None:raise TimeoutError('CUDA worker did not become ready')
        if metadata['device']!='cuda:0':raise AssertionError('Expected actual CUDA worker')
        server=source+(
            'python3 -m uav_inference.yolo_node --ros-args -r __ns:=/probe '
            '-p endpoint:=http://'+network+'-worker:8002 > '+evidence+'/inference-ros.log 2>&1 & a=$!; '
            'python3 -m uav_object_detection.node --ros-args -r __ns:=/probe '
            '-p registered_depth:=true -p stream_enabled:=true -p optical_frame:=probe/camera_optical '
            '-p output_frame:=probe/odom > '+evidence+'/tool-ros.log 2>&1 & b=$!; '
            'trap "kill -INT $a $b 2>/dev/null || true; wait" EXIT; wait -n $a $b')
        start('server',args.ros_image,server)
        # Pass fixture paths as positional shell arguments through JSON-safe filenames.
        if any(c in image.name for c in "'\n\r"):
            raise ValueError('Fixture basename contains unsupported shell characters')
        client=start('client',args.ros_image,source+
            "python3 /repo/deployment/object_detection/ros_probe.py --image '/fixture/"+image.name+
            "' --output "+evidence+'/service-result.json',fixture=True)
        code=run(['docker','wait',client],timeout=120)
        if code!='0':raise RuntimeError('Independent ROS client exited '+code)
        (output/'topology.json').write_text(json.dumps(dict(network=network,worker=metadata,
            worker_image=args.worker_image,ros_image=args.ros_image,
            scope='three isolated containers; no simulator or vehicle control'),indent=2)+'\n')
    finally:
        errors=[]
        for role,identifier in reversed(ids):
            log=subprocess.run(['docker','logs',identifier],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            (output/(role+'.log')).write_bytes(log.stdout)
            removed=subprocess.run(['docker','rm','-f',identifier],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            if removed.returncode:errors.append(removed.stdout.decode(errors='replace'))
        if created_network:
            removed=subprocess.run(['docker','network','rm',network],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            if removed.returncode:errors.append(removed.stdout.decode(errors='replace'))
        (output/'cleanup.json').write_text(json.dumps(dict(owned_container_ids=[i for _,i in ids],
            owned_network=network,success=not errors,errors=errors),indent=2)+'\n')
        if errors:raise RuntimeError('Owned-resource cleanup failed: '+str(errors))


if __name__=='__main__':main()
