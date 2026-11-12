"""Compute-role owner; task client does not start AS2 or the high-frequency source."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

def main():
    commands = {
        'platform':['ros2','launch','as2_platform_pixhawk','pixhawk_launch.py',
                    'namespace:=drone0','platform_config_file:=/evidence/platform.yaml'],
        'state':['ros2','launch','as2_state_estimator','state_estimator_launch.py',
                 'namespace:=drone0','plugin_name:=raw_odometry','use_sim_time:=true',
                 'config_file:=/evidence/config.yaml','plugin_config_file:=/evidence/config.yaml'],
        'controller':['ros2','launch','as2_motion_controller','controller_launch.py',
                      'namespace:=drone0','plugin_name:=pid_speed_controller','use_sim_time:=true',
                      'config_file:=/evidence/config.yaml',
                      'plugin_config_file:=/repo/deployment/airsim/config/as2/pid_speed_controller.yaml'],
        'behaviors':['ros2','launch','/repo/deployment/airsim/as2/selected_behaviors.launch.py',
                     'namespace:=drone0','use_sim_time:=true','config_file:=/evidence/config.yaml'],
        'acro':['ros2','launch','uav_follow_reference_acro','external_acro.launch.py',
                'namespace:=drone0','use_sim_time:=true','output_enabled:=true'],
        'source':['python3','/repo/deployment/airsim/demos/ipc_control/control_source.py'],
    }
    if os.environ.get('IPC_NATIVE') == '1':
        commands['record']=['ros2','bag','record','--output','/evidence/rosbag',
            '/clock','/drone0/navigation/odom','/drone0/ipc/processed_points',
            '/drone0/external_acro/session','/drone0/external_acro/control','/drone0/ipc/health',
            '/drone0/ipc/path','/drone0/controller/info','/drone0/self_localization/pose','/drone0/fmu/out/vehicle_control_mode']
        commands['source']=['python3','/repo/deployment/airsim/demos/ipc_navigation/source.py']
        commands['navigation']=['ros2','run','as2_navigation_inputs','navigation_inputs_node','--ros-args',
            '-r','__ns:=/drone0','-p','use_sim_time:=true','-p','target_frame:=earth',
            '-p','body_frame:=drone0/base_link']
    out = Path('/evidence'); children = []; stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    try:
        for name, command in commands.items():
            stream = (out/(name+'.log')).open('w')
            child = subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            children.append((name,child,stream))
        while not stopping:
            dead = {name:child.returncode for name,child,_ in children if child.poll() is not None}
            if dead: (out/'compute-error.json').write_text(json.dumps(dead)+'\n')
            time.sleep(.1)
    finally:
        cleanup = {}
        for name, child, stream in children:
            if child.poll() is None: os.killpg(child.pid,signal.SIGINT)
        for name, child, stream in children:
            escalated = False
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                escalated = True; os.killpg(child.pid,signal.SIGKILL); child.wait(timeout=5)
            cleanup[name] = {'returncode':child.returncode,'forced_kill':escalated}
            stream.close()
        (out/'compute-cleanup.json').write_text(json.dumps(cleanup,indent=2)+'\n')

if __name__ == '__main__': main()
