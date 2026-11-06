"""One simulated compute board: one AS2, one input adapter, one owned output adapter."""
import json,os,signal,subprocess,time
from pathlib import Path
root=Path(os.environ['COMPUTE_ROOT']);vehicle=os.environ['COMPUTE_VEHICLE'];children=[];running=True
root.mkdir(parents=True,exist_ok=True)

def stop(*_):
    global running
    running=False
signal.signal(signal.SIGTERM,stop)

def start(name,command):
    log=(root/(name+'.log')).open('w')
    p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);log.close();children.append(p)
    return p

os.environ.update(AS2_LAUNCHER='/opt/catora_hybrid/as2/run.py',AS2_FLEET_FILE=str(root/'fleet.yaml'),
 AS2_CONFIG_FILE='/opt/catora_hybrid/config/config.yaml',AS2_CONTROLLER_CONFIG_FILE='/opt/catora_hybrid/config/pid_speed_controller.yaml',
 AS2_RUNTIME_DIR=str(root/'as2'),AS2_BEHAVIORS='takeoff,land',AS2_TF_SCOPE='per-vehicle',AS2_MAX_RSS_MB='2048')
os.environ['CATORA_METRICS_FILE']=str(root/'metrics.jsonl')
try:
    start('as2',['python3',os.environ['AS2_LAUNCHER']])
    start('input',['ros2','run','uav_cat_ora_adapter','cat_ora_state_adapter_node','--ros-args',
        '-r','__node:=cat_ora_state_'+vehicle,'-p','deployment_config:='+str(root/'deployment.yaml'),'-p','use_sim_time:=true'])
    start('metrics',['python3','/opt/catora_hybrid/metrics.py'])
    configured=False
    while running:
        if any(p.poll() is not None for p in children):raise RuntimeError('compute child exited')
        configs=list(Path('/run/resolved').glob('*.yaml'))
        if configs and not configured:
            p=sorted(configs)[-1]
            start('output',['ros2','run','uav_cat_ora_adapter','cat_ora_as2_adapter_node','--ros-args',
                '-r','__node:=cat_ora_as2_'+vehicle,'-p','resolved_config:='+str(p),'-p','vehicle_id:='+vehicle,'-p','use_sim_time:=true'])
            configured=True
        time.sleep(.2)
finally:
    for p in reversed(children):
        try:os.killpg(p.pid,signal.SIGTERM)
        except ProcessLookupError:pass
    for p in reversed(children):
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:pass
        try:os.killpg(p.pid,signal.SIGKILL)
        except ProcessLookupError:pass
    (root/'worker-stopped.json').write_text(json.dumps({'vehicle':vehicle,'child_returncodes':[p.poll() for p in children]})+'\n')
