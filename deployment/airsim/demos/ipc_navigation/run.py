"""Free-XYZ IPC SUPER A-to-B, nominal 3m/s; AirSim/PX4 and PC compute."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ipc_control'))
import run as control_run

def default_arguments(arguments):
    result=list(arguments)
    if not any(a in ('--task','--criteria') or a.startswith(('--task=','--criteria=')) for a in result):
        base=Path(__file__).resolve().parent
        result.extend(['--task',str(base/'super-3mps-balanced-task.json'),
                       '--criteria',str(base/'super-3mps-balanced-criteria.json')])
    if not any(a=='--clock-rate-hz' or a.startswith('--clock-rate-hz=') for a in result):
        result.extend(['--clock-rate-hz','200'])
    return result+['--native','--forest']

if __name__=='__main__':
    sys.argv=[sys.argv[0]]+default_arguments(sys.argv[1:])
    control_run.main()
