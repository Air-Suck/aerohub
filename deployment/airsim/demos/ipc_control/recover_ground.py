"""Recover only this test's resources after independently confirming PX4 is disarmed."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

def main():
    p = argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True)
    out = p.parse_args().output.resolve()
    state = subprocess.check_output(['docker','exec','aerohub-ipc-control-vehicle',
        '/opt/px4-gazebo/bin/px4-listener','--instance','72','vehicle_control_mode','-n','1'],text=True)
    (out/'recovery-control-mode.txt').write_text(state)
    if 'flag_armed: False' not in state: raise RuntimeError('Disarmed state not confirmed; keep control resources')
    for suffix in ('app','clock','vehicle'):
        name = 'aerohub-ipc-control-'+suffix
        info = subprocess.run(['docker','inspect',name],capture_output=True,text=True)
        if info.returncode: continue
        record = json.loads(info.stdout)[0]
        if record['Config']['Labels'].get('aerohub.ipc.output') != str(out):
            raise RuntimeError('Container owned by another run')
        with (out/(name+'.log')).open('w') as log:
            subprocess.run(['docker','logs',name],stdout=log,stderr=subprocess.STDOUT)
        subprocess.run(['docker','stop','--time','10',name],check=True)
    root = Path(__file__).resolve().parents[4]
    subprocess.run([sys.executable,str(root/'simulator/manager/manager.py'),
        'stop','--run-dir',str(out/'scene')],check=True)
    relay = json.loads((out/'relay.json').read_text())['pid']
    try:
        cmd = Path(f'/proc/{relay}/cmdline').read_bytes()
        if str(out).encode() in cmd and b'ipc_control/relay.py' in cmd:
            os.kill(relay,signal.SIGTERM)
    except FileNotFoundError: pass
    (out/'cleanup.json').write_text(json.dumps({'confirmed_disarmed':True,'owned_resources_stopped':True})+'\n')

if __name__ == '__main__': main()
