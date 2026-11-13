"""Own PC-co-located task/compute processes; retain compute if recovery is unconfirmed."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

def main():
    base = Path(__file__).resolve().parent
    compute = subprocess.Popen(['python3',str(base/'compute_runtime.py')],start_new_session=True)
    task_base = Path('/repo/deployment/airsim/demos/ipc_navigation') if os.environ.get('IPC_NATIVE') == '1' else base
    if os.environ.get('EXTERNAL_ATTITUDE_TEST') == '1': task_base=Path('/repo/deployment/airsim/demos/attitude_control')
    if os.environ.get('DIFFPHYS_TEST') == '1': task_base=Path('/repo/deployment/airsim/demos/diffphys_forest')
    task_name='ground_probe.py' if os.environ.get('IPC_GROUND_ONLY')=='1' else ('forest_mission.py' if os.environ.get('IPC_FOREST')=='1' else 'mission.py')
    task = subprocess.Popen(['python3',str(task_base/task_name)],start_new_session=True)
    def recover(*_):
        if task.poll() is None: task.send_signal(signal.SIGINT)
    signal.signal(signal.SIGINT,recover); signal.signal(signal.SIGTERM,recover)
    code = task.wait()
    out = Path('/evidence')
    result = json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {}
    if not result.get('landed_and_disarmed'):
        print('Recovery not confirmed; compute retained',flush=True)
        while compute.poll() is None: time.sleep(1)
        raise SystemExit(1)
    compute.send_signal(signal.SIGINT)
    compute.wait(timeout=35)
    raise SystemExit(code)

if __name__ == '__main__': main()
