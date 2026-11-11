from ament_index_python.packages import get_package_prefix
"""Compute deployment owner for the real IPC process and deliberate loss injection."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
out=Path('/evidence');stopping=False
def stop(*_):
    global stopping
    stopping=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
while not stopping and not (out/'hover.json').exists() and os.environ.get('IPC_GROUND_ONLY')!='1':time.sleep(.05)
if stopping:raise SystemExit(0)
hover=json.loads((out/'hover.json').read_text())['normalized_thrust'] if (out/'hover.json').exists() else .48821
task=json.loads((out/'forest-task.json').read_text()) if (out/'forest-task.json').exists() else {}
child=subprocess.Popen([get_package_prefix('ipc_native_core')+'/lib/ipc_native_core/ipc_node','--ros-args','-r','__ns:=/drone0','-p','use_sim_time:=true','-p','hover_percentage:='+str(hover),
    '-p','diagnostic_qp_path:='+str(out/'failed-qp.json'),'-p','planar_path:='+str(task.get('planar_path',False)).lower(),'-p','reference_speed_mps:='+str(task.get('reference_speed_m_s',.25)),
    '-p','obstacle_clearance:='+str(task.get('planning_clearance_m',.55)),
    '-p','map.z_min:='+str(task.get('map_z_min_m',0.)),
    '-p','map.z_max:='+str(task.get('map_z_max_m',8.)),
    '-p','mpc.velocity_limit:='+str(task.get('mpc_velocity_limit_m_s',.5)),
    '-p','reference_speed_ratio:='+str(task.get('reference_speed_ratio',.6)),
    '-p','reference_acceleration_mps2:='+str(task.get('reference_acceleration_m_s2',0.)),
    '-p','mpc.acceleration_limit:='+str(task.get('mpc_acceleration_limit_m_s2',1.)),
    '-p','mpc.jerk_limit:='+str(task.get('mpc_jerk_limit_m_s3',5.))])
try:
    while not stopping and not (out/'stop-source').exists():
        if child.poll() is not None:raise RuntimeError('IPC exited unexpectedly')
        time.sleep(.02)
finally:
    if child.poll() is None:child.send_signal(signal.SIGINT)
    child.wait(timeout=5)
while not stopping:time.sleep(.05)

