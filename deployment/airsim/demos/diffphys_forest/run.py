"""Launch the shared isolated forest deployment with the actual DiffPhys policy."""
import os
from pathlib import Path
import sys
if __name__=='__main__':
    root=Path(__file__).resolve().parents[4]
    args=sys.argv[1:]
    if '--diffphys-camera-x' not in args:args += ['--diffphys-camera-x','.5']
    if '--diffphys-long' in args:
        if '--diffphys-profile' not in args:args += ['--diffphys-profile','high_speed']
        if '--diffphys-speed' not in args:args += ['--diffphys-speed','8']
        for flag,value in [('--diffphys-takeoff-height','4'),
                           ('--diffphys-goal-lookahead','10'),
                           ('--diffphys-command-tilt','.5'),
                           ('--diffphys-tilt-limit','.6')]:
            if flag not in args:args += [flag,value]
    if '--workspace' not in args:args += ['--workspace',str(root/'log/diffphys/attitude-overlay')]
    if '--clock-rate-hz' not in args:args += ['--clock-rate-hz','100']
    os.execv(sys.executable,[sys.executable,str(root/'deployment/airsim/demos/ipc_control/run.py'),'--diffphys']+args)
