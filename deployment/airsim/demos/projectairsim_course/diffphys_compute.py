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
    }
    commands['acro'].append('control_mode:=attitude')
    commands['acro'].append('solve_timeout:=0.2')
    commands['source']=['ros2','run','uav_diffphys_planner','diffphys_planner','--ros-args','-r','__ns:=/drone0','-p','use_sim_time:=true']
    commands['source'] += ['-p','camera_x_m:='+os.environ.get('DIFFPHYS_CAMERA_X','0.5')]
    commands['source'] += ['-p','policy_profile:='+os.environ.get('DIFFPHYS_PROFILE','high_speed')]
    commands['source'] += ['-p','target_speed:='+str(float(os.environ.get('DIFFPHYS_SPEED','5.0')))]
    commands['source'] += ['-p','goal_lookahead_m:='+str(float(os.environ.get('DIFFPHYS_GOAL_LOOKAHEAD','0.0')))]
    commands['source'] += ['-p','max_command_tilt_rad:='+os.environ.get('DIFFPHYS_COMMAND_TILT','0.0')]
    commands['model_adapter']=['python3','-m','uav_inference.diffphys_node','--ros-args','-r','__ns:=/drone0','-p','use_sim_time:=true','-p','endpoint:='+os.environ['DIFFPHYS_ENDPOINT']]
    commands['model_adapter'] += ['-p','timing_file:=/evidence/inference-timing.jsonl']
    commands['record']=['ros2','bag','record','--output','/evidence/rosbag',
        '/clock','/drone0/sensor_measurements/diffphys_front/depth_planar/image',
        '/drone0/sensor_measurements/diffphys_front/depth_planar/camera_info',
        '/drone0/self_localization/pose','/drone0/self_localization/twist',
        '/drone0/external_acro/session','/drone0/external_acro/control','/drone0/diffphys/status',
        '/drone0/controller/info','/drone0/fmu/out/vehicle_control_mode']
    commands['source'] += ['-p','camera_reliable:=false','-p','camera_info_transient_local:=true']
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
