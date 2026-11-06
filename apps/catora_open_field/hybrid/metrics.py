"""Read-only one-second process/host counters; 100 process CPU percent = one core."""
import json,os,signal,time
from pathlib import Path
running=True

def stop(*_):
    global running
    running=False
signal.signal(signal.SIGTERM,stop)
identity=os.environ['CATORA_COMPUTE_ID'];output=Path(os.environ['CATORA_METRICS_FILE']);output.parent.mkdir(parents=True,exist_ok=True)

def snapshot():
    processes={}
    for p in Path('/proc').glob('[0-9]*'):
        try:
            if ('CATORA_COMPUTE_ID='+identity).encode() not in (p/'environ').read_bytes().split(b'\0'):continue
            fields=(p/'stat').read_text().rsplit(')',1)[1].split()
            command=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode()
            status=dict(x.split(':',1) for x in (p/'status').read_text().splitlines() if ':' in x)
            group='other'
            if any('/'+n in command for n in ('as2_platform_pixhawk_node','as2_state_estimator_node','as2_motion_controller_node','takeoff_behavior_node','land_behavior_node')):group='as2_native'
            elif '/lib/ego_planner/ego_planner_node' in command:group='swarm_planner_native'
            elif '/lib/plan_env/navigation_bridge_node' in command:group='swarm_map_bridge_native'
            elif '/lib/as2_navigation_inputs/navigation_inputs_node' in command:group='navigation_native'
            elif 'ros2 launch' in command or 'run_as2.py' in command:group='as2_launch'
            elif 'cat_ora_node' in command or 'ros2_proxy.py' in command:group='algorithm_service'
            elif 'planner_framework_node' in command:group='coordinator'
            elif 'cat_ora_' in command or 'swarm_formation_adapter' in command or 'swarm_as2_adapter' in command or 'swarm_input_adapter' in command:group='adapter'
            elif 'mission.py' in command:group='mission'
            processes[p.name]=(int(fields[11])+int(fields[12]),int(status.get('VmRSS','0 kB').split()[0]),group,command)
        except (OSError,ValueError):pass
    cpu=list(map(int,Path('/proc/stat').read_text().splitlines()[0].split()[1:9]))
    return time.monotonic(),processes,cpu

previous=snapshot()
with output.open('w',buffering=1) as stream:
    while running:
        time.sleep(1);current=snapshot();elapsed=current[0]-previous[0];groups={}
        for pid,(ticks,rss,group,command) in current[1].items():
            g=groups.setdefault(group,{'cpu_core_percent':0.,'rss_sum_mib':0.,'processes':0})
            g['processes']+=1;g['rss_sum_mib']+=rss/1024
            if pid in previous[1]:g['cpu_core_percent']+=100*(ticks-previous[1][pid][0])/os.sysconf('SC_CLK_TCK')/elapsed
        total=sum(current[2])-sum(previous[2]);idle=sum(current[2][3:5])-sum(previous[2][3:5])
        result={'unix_time':time.time(),'compute_id':identity,'machine':os.uname().machine,'host_logical_cpus':os.cpu_count(),
            'host_cpu_percent':100*(1-idle/total) if total else 0.,'process_cpu_core_percent':sum(x['cpu_core_percent'] for x in groups.values()),
            'rss_sum_mib':sum(x['rss_sum_mib'] for x in groups.values()),'groups':groups}
        stream.write(json.dumps(result)+'\n');previous=current
