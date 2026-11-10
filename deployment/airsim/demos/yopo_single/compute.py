"""Compute role supervisor; task execution lives in the workstation process."""
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]
APP = Path(__file__).resolve().parent


def main():
    commands = [
        ['python3', str(ROOT/'deployment/airsim/as2/run.py')],
        ['python3', '-m', 'uav_inference.yopo_node', '--ros-args', '-r', '__ns:=/drone0',
         '-p', 'use_sim_time:=true', '-p', 'endpoint:='+os.environ.get('YOPO_ENDPOINT', 'http://127.0.0.1:18081')],
        ['ros2', 'run', 'uav_yopo_planner', 'yopo_planner', '--ros-args', '-r', '__ns:=/drone0',
         '--params-file', str(APP/'planner.yaml'),
         '-r', 'yopo/depth:=sensor_measurements/yopo_front/depth_planar/image',
         '-r', 'yopo/camera_info:=sensor_measurements/yopo_front/depth_planar/camera_info'],
    ]
    children = []
    stopping = False
    reported = set()
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    try:
        for command in commands:
            children.append(subprocess.Popen(command, start_new_session=True))
        while not stopping:
            if children[0].poll() is not None:
                raise RuntimeError('AS2 execution process exited')
            for index, child in enumerate(children[1:],1):
                if child.poll() is not None and index not in reported:
                    print('Planning process exited; retain AS2 for bounded trajectory expiry and native Land recovery:', index, child.returncode, flush=True)
                    reported.add(index)
            time.sleep(.1)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try: child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=3)


if __name__ == '__main__':
    main()
