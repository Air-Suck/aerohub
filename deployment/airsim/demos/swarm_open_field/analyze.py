#!/usr/bin/env python3
"""Summarize raw mixed-deployment evidence without changing the raw run directories."""
import argparse,json,math,re,statistics,itertools,hashlib
from pathlib import Path


def read(path):return json.loads(path.read_text())
def jsonl(path):return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
def stats(xs):
    if not xs:return None
    ordered=sorted(xs)
    return {'median':statistics.median(xs),'p95':ordered[max(0,math.ceil(.95*len(xs))-1)],'max':max(xs)}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--ground',type=Path,required=True);parser.add_argument('--flight',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    ground=read(a.ground/'results/ground-probe.json');topology=read(a.flight/'topology.json');rows=jsonl(a.flight/'results/flight.jsonl')
    positions=[r for r in rows if r.get('trajectory_mode_claimed') and 'positions_ned' in r]
    if not positions:raise ValueError('no actual TRAJECTORY mode execution window')
    begin,end=positions[0]['unix_time'],positions[-1]['unix_time'];elapsed=end-begin
    phase_timing={}
    for phase in sorted({r.get('phase') for r in rows if 'positions_ned' in r}):
        group=[r for r in rows if r.get('phase')==phase and 'positions_ned' in r]
        wall=group[-1]['unix_time']-group[0]['unix_time'];sim=group[-1]['sim_time']-group[0]['sim_time']
        if wall>=2:phase_timing[phase]={'wall_seconds':wall,'sim_seconds':sim,'real_time_factor':sim/wall}
    metrics={};inputs={};receipts={};warnings={}
    for i in range(3):
        vehicle=f'drone{i}';folder=a.flight/'results' if i==0 else a.flight/'compute'/vehicle
        # Board flight markers and board metrics use the same wall clock; compensate
        # the measured small board-PC clock offset for the x86 metrics window.
        offset=0 if i==0 else topology['clock_offset_s']
        samples=[r for r in jsonl(folder/'metrics.jsonl') if begin<=r['unix_time']+offset<=end]
        groups={key:{field:statistics.median(r['groups'].get(key,{}).get(field,0) for r in samples)
                     for field in ('cpu_core_percent','rss_sum_mib','processes')}
                for key in sorted({key for r in samples for key in r['groups']})}
        metrics[vehicle]={'machine':samples[0]['machine'],'samples':len(samples),'groups':groups,
            **{field:stats([r[field] for r in samples]) for field in ('host_cpu_percent','process_cpu_core_percent','rss_sum_mib')}}
        native=(folder/'native-planner.log').read_text(errors='replace')
        ages=[(int(peer),float(age)) for peer,age in re.findall(r'Time stamp diff: Local - Remote Agent (\d+) = ([\d.-]+)s',native)]
        warnings[vehicle]={'native_start_time_rejections':len(ages),'peer_start_time_rejections':sum(peer!=i for peer,_ in ages),
                          'max_abs_start_age_s':max((abs(age) for _,age in ages),default=0),
                          'scope':'age from polynomial start_time to callback; not pure network delay'}
        observations=[r for r in rows if begin<=r.get('unix_time',0)<=end and r.get('swarm_input',{}).get('vehicle')==vehicle]
        first,last=observations[0],observations[-1];duration=last['unix_time']-first['unix_time']
        inputs[vehicle]={'samples':len(observations),'healthy_fraction':sum(r['swarm_input']['healthy'] for r in observations)/len(observations),
            'map_healthy_fraction':sum(r['swarm_input']['map_healthy'] for r in observations)/len(observations),
            'counts':{k:last['swarm_input']['counts'][k]-first['swarm_input']['counts'][k] for k in ('cloud','odom')},
            'receipt_wall_hz':{k:(last['swarm_input']['counts'][k]-first['swarm_input']['counts'][k])/duration for k in ('cloud','odom')}}
        for row in rows:
            state=row.get('bridge_state',{})
            if vehicle in state.get('owned_vehicles',[]):receipts[vehicle]=state.get('receipt',{})
    spatial={}
    import yaml
    targets=yaml.safe_load(Path(__file__).with_name('task.yaml').read_text())['targets']
    for key in ('positions_ned','airsim_truth_ned','desired_positions_ned'):
        observations=[r for r in rows if r.get('phase')=='planning' and r.get(key)]
        if not observations:continue
        closest=min((math.dist(row[key][v],row[key][w]),row['sim_time'],v,w) for row in observations for v,w in itertools.combinations(row[key],2))
        spatial[key]={'min_separation_m':closest[0],'closest_sim_time':closest[1],'closest_pair':list(closest[2:]),
            'last_observed_endpoint_error_m':{v:math.dist(pos,[targets[v][axis] for axis in ('x','y','z')]) for v,pos in observations[-1][key].items()}}
    failure=read(a.flight/'failure.json') if (a.flight/'failure.json').exists() else None
    failure_stop=read(a.flight/'results/failure-stop.json') if (a.flight/'results/failure-stop.json').exists() else None
    result=read(a.flight/'result.json') if (a.flight/'result.json').exists() else None
    audit=read(a.flight/'dds-audit.json')
    as2_logs={f'drone{i}':(a.flight/('results/as2.log' if i==0 else f'compute/drone{i}/as2.log')).read_text(errors='replace') for i in range(3)}
    execution_errors={v:{'stream_lost':len(re.findall('Trajectory stream lost',text)),
                          'rejected_reference':len(re.findall('Rejecting trajectory|Rejected trajectory|Invalid trajectory reference',text,re.IGNORECASE))}
                      for v,text in as2_logs.items()}
    passed=bool(ground['passed'] and result and result['planner_status']=='SUCCEEDED' and result['landed_and_disarmed'] and not result['planning_collisions'])
    passed=passed and all(m['groups']['as2_native']['processes']==5 and m['groups']['swarm_planner_native']['processes']==1 for m in metrics.values())
    passed=passed and audit['bad_references']==audit['bad_segments']==0 and audit['segments']==audit['pva_segments'] and audit['pva_segments']>1
    passed=passed and all(v['native_polynomials']>1 and v['broadcasts']>1 and v['finite_pva_references']>10 for v in audit['vehicles'].values())
    passed=passed and all(x['healthy_fraction']>.95 for x in inputs.values()) and not any(any(x.values()) for x in execution_errors.values())
    checks={'ground_real_inputs_and_dds':ground['passed'],
        'planner_task_succeeded':bool(result and result.get('planner_status')=='SUCCEEDED') or bool(failure and failure.get('planner_status')==3),
        'landed_and_disarmed':bool(result and result.get('landed_and_disarmed')) or bool(failure_stop and failure_stop.get('landed_and_disarmed')),
        'minimum_observed_separation_2m':spatial['positions_ned']['min_separation_m']>=2,
        'minimum_truth_separation_2m':spatial['airsim_truth_ned']['min_separation_m']>=2,
        'valid_pva_segments_and_references':audit['bad_references']==audit['bad_segments']==0 and audit['segments']==audit['pva_segments'],
        'native_planner_and_as2_on_each_node':all(m['groups']['as2_native']['processes']==5 and m['groups']['swarm_planner_native']['processes']==1 for m in metrics.values())}
    report={'passed':passed,'checks':checks,'spatial_observations':spatial,'failure_stop':failure_stop,'ground_evidence':str(a.ground),'flight_evidence':str(a.flight),'ground':ground,'flight':result,
        'failure':failure,
        'topology':topology,'images':read(a.flight/'images.json'),'startup_transfers':read(a.flight/'startup-transfers.json'),
        'phase_timing':phase_timing,
        'trajectory_window':{'wall_seconds':elapsed,'sim_seconds':positions[-1]['sim_time']-positions[0]['sim_time'],
            'real_time_factor':(positions[-1]['sim_time']-positions[0]['sim_time'])/elapsed},
        'cpu_rss':metrics,'real_input_during_execution':inputs,'setpoint_receipts':receipts,'dds_audit':audit,
        'native_broadcast_start_time_rejections':warnings,'as2_execution_errors':execution_errors,
        'limits':['One RoboPi plus two x86 logical Docker nodes sharing one NIC; not three physical ARM boards.',
                  'Normal LAN, three vehicles and open-field real LiDAR; no injected loss/disconnect or seven-vehicle stress.',
                  'Board load includes global proxy, sampler and observer; RSS process sums include shared pages.',
                  'Native timing logs use ROS/simulation time and may say 0 ms; they do not measure CPU solver wall time.',
                  'Native start_time age rejection includes callback queueing and trajectory scheduling; it is not wire delay.']}
    for filename,key in [('native-parameters.json','native_runtime_parameters'),('cleanup.json','cleanup')]:
        if (a.flight/filename).exists():report[key]=read(a.flight/filename)
    report['installed_manifests']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in a.flight.glob('installed-*.json')}
    if (a.flight/'planner-configuration.json').exists():report['planner_configuration']=read(a.flight/'planner-configuration.json')
    passed=passed and all(checks.values()) and all(report[key].get('passed',False) for key in ('native_runtime_parameters','cleanup') if key in report)
    report['passed']=passed
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'passed':passed,'trajectory_window':report['trajectory_window'],'cpu':metrics['drone0']['host_cpu_percent'],
                      'native_warnings':warnings,'real_input':inputs,'audit':audit},indent=2))
    if not passed:raise SystemExit(1)

if __name__=='__main__':main()
