"""One PC logical board: local AS2, navigation registration, native planner and adapters."""
import json,os,signal,subprocess,time
from pathlib import Path
root=Path(os.environ['COMPUTE_ROOT']);vehicle=os.environ['COMPUTE_VEHICLE'];children=[];running=True
backend=os.environ.get('AS2_EXECUTION_BACKEND','legacy_setpoint')
if backend not in ('follow_path','legacy_setpoint'):raise ValueError('invalid AS2 execution backend')
app=Path('/opt/swarm_app');shared=Path('/run/resolved')
def stop(*_):
    global running
    running=False
signal.signal(signal.SIGTERM,stop)
def start(name,command):
    with (root/(name+'.log')).open('w') as log:
        children.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
os.environ.update(AS2_FLEET_FILE=str(root/'fleet.yaml'),AS2_RUNTIME_DIR=str(root/'as2'),
    AS2_CONFIG_FILE=str(app/'config/config.yaml'),AS2_CONTROLLER_CONFIG_FILE=str(app/'config/pid_speed_controller.yaml'),
    AS2_EXECUTION_BACKEND=backend,AS2_TF_SCOPE='per-vehicle',CATORA_METRICS_FILE=str(root/'metrics.jsonl'))
if backend=='legacy_setpoint':os.environ['AS2_BEHAVIORS']='takeoff,land'
else:os.environ.pop('AS2_BEHAVIORS',None)
try:
    start('as2',['python3',str(app/'as2/run.py')])
    start('navigation',['ros2','run','as2_navigation_inputs','navigation_inputs_node','--ros-args','-r','__ns:=/'+vehicle,
        '-r','/tf:=/'+vehicle+'/tf','-r','/tf_static:=/'+vehicle+'/tf_static','-p','use_sim_time:=true',
        '-p','target_frame:='+vehicle+'/odom','-p','body_frame:='+vehicle+'/base_link'])
    start('input',['ros2','run','uav_swarm_formation_adapter','swarm_input_adapter_node','--ros-args','-p','vehicle_id:='+vehicle,'-p','use_sim_time:=true'])
    start('metrics',['python3',str(app/'metrics.py')])
    native=False;output=False
    while running:
        if any(p.poll() is not None for p in children):raise RuntimeError('compute child exited')
        fleet=shared/'nav-fleet.yaml'
        if fleet.exists() and not native:
            start('native-planner',['ros2','launch','ego_planner','seven_planners.launch.py','input_mode:=navigation',
                'vehicle_count:=3','local_vehicle:='+vehicle,'start_proxy:=false','fleet_config:='+str(fleet),'parameters_file:='+str(root/'planner.yaml')]);native=True
        configs=list((shared/'resolved').glob('*.yaml'))
        if backend=='legacy_setpoint' and configs and not output:
            start('output',['ros2','run','uav_swarm_formation_adapter','swarm_as2_adapter_node','--ros-args',
                '-r','__node:=swarm_as2_'+vehicle,'-p','resolved_config:='+str(configs[0]),'-p','vehicle_id:='+vehicle,'-p','use_sim_time:=true']);output=True
        time.sleep(.2)
except Exception as exc:
    (root/'worker-failure.json').write_text(json.dumps({'error':str(exc),'returncodes':[p.poll() for p in children]})+'\n')
    raise
finally:
    for p in reversed(children):
        try:os.killpg(p.pid,signal.SIGTERM)
        except ProcessLookupError:pass
    for p in reversed(children):
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:pass
        try:os.killpg(p.pid,signal.SIGKILL)
        except ProcessLookupError:pass
    (root/'worker-stopped.json').write_text(json.dumps({'vehicle':vehicle,'returncodes':[p.poll() for p in children]})+'\n')
