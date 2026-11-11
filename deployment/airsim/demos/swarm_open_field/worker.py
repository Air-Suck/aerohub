"""One PC logical board: local AS2, navigation registration, native planner and adapters."""
import json,os,signal,subprocess,time
from pathlib import Path
root=Path(os.environ['COMPUTE_ROOT']);vehicle=os.environ['COMPUTE_VEHICLE'];children=[];running=True
vehicle_count=int(os.environ['AEROHUB_SWARM_VEHICLE_COUNT'])
if not 3<=vehicle_count<=7:raise ValueError('native Swarm-Formation launch supports 3..7 vehicles')
backend=os.environ.get('AS2_EXECUTION_BACKEND','legacy_setpoint')
if backend not in ('follow_path','legacy_setpoint'):raise ValueError('invalid AS2 execution backend')
scenario=os.environ['AEROHUB_SWARM_SCENARIO']
center_vehicle=os.environ['AEROHUB_SWARM_CENTER']
app=Path('/opt/swarm_app');shared=Path('/run/resolved')
def stop(*_):
    global running
    running=False
signal.signal(signal.SIGTERM,stop)
def start(name,command):
    with (root/(name+'.log')).open('w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    children.append((name,process))
    return process
os.environ.update(AS2_FLEET_FILE=str(root/'fleet.yaml'),AS2_RUNTIME_DIR=str(root/'as2'),
    AS2_CONFIG_FILE=str(app/'config/config.yaml'),AS2_CONTROLLER_CONFIG_FILE=str(app/'config/pid_speed_controller.yaml'),
    AS2_EXECUTION_BACKEND=backend,AS2_TF_SCOPE='per-vehicle',CATORA_METRICS_FILE=str(root/'metrics.jsonl'))
if backend=='legacy_setpoint':os.environ['AS2_BEHAVIORS']='takeoff,land'
else:os.environ.pop('AS2_BEHAVIORS',None)
injected=set()
try:
    start('as2',['python3',str(app/'as2/run.py')])
    start('navigation',['ros2','run','as2_navigation_inputs','navigation_inputs_node','--ros-args','-r','__ns:=/'+vehicle,
        '-r','/tf:=/'+vehicle+'/tf','-r','/tf_static:=/'+vehicle+'/tf_static','-p','use_sim_time:=true',
        '-p','target_frame:='+vehicle+'/odom','-p','body_frame:='+vehicle+'/base_link'])
    start('input',['ros2','run','uav_swarm_formation_adapter','swarm_input_adapter_node','--ros-args','-p','vehicle_id:='+vehicle,'-p','use_sim_time:=true'])
    start('metrics',['python3',str(app/'metrics.py')])
    native=False;output=False
    while running:
        failed=[(name,p.returncode) for name,p in children if p.poll() is not None and name not in injected]
        if failed:raise RuntimeError('compute child exited: '+str(failed))
        fleet=shared/'nav-fleet.yaml'
        if fleet.exists() and not native:
            start('native-planner',['ros2','launch','ego_planner','seven_planners.launch.py','input_mode:=navigation',
                'vehicle_count:='+str(vehicle_count),'local_vehicle:='+vehicle,'start_proxy:=false','fleet_config:='+str(fleet),'parameters_file:='+str(root/'planner.yaml')])
            if scenario=='board_worker' and vehicle==center_vehicle:
                start('swarm-proxy',['ros2','run','ego_planner','ros2_proxy.py','--ros-args',
                    '-p','use_sim_time:=true','-p','bind_host:=0.0.0.0','-p','vehicle_count:='+str(vehicle_count),
                    '-p','publish_odometry:=false','-p','relay_broadcast:=false',
                    '-p','map_frame:=swarm_map'])
            native=True
        configs=list((shared/'resolved').glob('*.yaml'))
        if backend=='legacy_setpoint' and configs and not output:
            start('output',['ros2','run','uav_swarm_formation_adapter','swarm_as2_adapter_node','--ros-args',
                '-r','__node:=swarm_as2_'+vehicle,'-p','resolved_config:='+str(configs[0]),'-p','vehicle_id:='+vehicle,'-p','use_sim_time:=true']);output=True
        for marker,name in (('inject-native-planner-exit','native-planner'),('inject-center-proxy-exit','swarm-proxy')):
            path=root/marker
            if path.exists() and name not in injected:
                matches=[process for child_name,process in children if child_name==name and process.poll() is None]
                if len(matches)!=1:raise RuntimeError(f'cannot inject {name}: active matches={len(matches)}')
                os.killpg(matches[0].pid,signal.SIGTERM)
                try:matches[0].wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(matches[0].pid,signal.SIGKILL);matches[0].wait()
                injected.add(name)
                (root/'fault-injected.json').write_text(json.dumps({
                    'vehicle':vehicle,'process':name,'pid':matches[0].pid,
                    'returncode':matches[0].returncode})+'\n')
        time.sleep(.2)
except Exception as exc:
    (root/'worker-failure.json').write_text(json.dumps({
        'error':str(exc),'returncodes':{n:p.poll() for n,p in children}})+'\n')
    raise
finally:
    for _,p in reversed(children):
        try:os.killpg(p.pid,signal.SIGTERM)
        except ProcessLookupError:pass
    for _,p in reversed(children):
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:pass
        try:os.killpg(p.pid,signal.SIGKILL)
        except ProcessLookupError:pass
    (root/'worker-stopped.json').write_text(json.dumps({
        'vehicle':vehicle,'injected':sorted(injected),
        'returncodes':{n:p.poll() for n,p in children}})+'\n')
