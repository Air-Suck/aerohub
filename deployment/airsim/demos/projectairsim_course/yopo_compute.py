"""Compute-only supervisor; retain AS2 if planning exits to allow recovery."""
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]

def main():
    children = []
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    commands = [
        ['python3','/opt/onboard_computer/run.py'],
        ['python3','-m','uav_inference.yopo_node','--ros-args','-r','__ns:=/drone0',
         '-p','use_sim_time:=true','-p','endpoint:=http://127.0.0.1:18081'],
        ['ros2','run','uav_yopo_planner','yopo_planner','--ros-args','-r','__ns:=/drone0',
         '--params-file',str(ROOT/'deployment/airsim/demos/projectairsim_course/yopo.yaml'),
         '-p','max_speed:='+os.environ.get('YOPO_COURSE_SPEED','5.0'),
         '-r','yopo/depth:=sensor_measurements/yopo_front/depth_planar/image',
         '-r','yopo/camera_info:=sensor_measurements/yopo_front/depth_planar/camera_info']]
    try:
        for command in commands:
            children.append(subprocess.Popen(command,start_new_session=True))
        while not stopping:
            if children[0].poll() is not None:
                raise RuntimeError('AS2 exited')
            time.sleep(.2)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait()

if __name__ == '__main__':main()
