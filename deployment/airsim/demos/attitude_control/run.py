"""Use the existing isolated simulator lifecycle with explicit attitude mode."""
import os
from pathlib import Path
import sys

if __name__ == '__main__':
    root=Path(__file__).resolve().parents[4]
    args=sys.argv[1:]
    if '--workspace' not in args:
        args += ['--workspace',str(root/'log/diffphys/attitude-overlay')]
    if '--clock-rate-hz' not in args:
        args += ['--clock-rate-hz','100']
    os.execv(sys.executable,[sys.executable,str(root/'deployment/airsim/demos/ipc_control/run.py'),'--attitude']+args)
