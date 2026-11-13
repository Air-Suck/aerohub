"""Offline short-flight audit: original coefficients versus actual AS2 references.

Uses NumPy/Matplotlib in the existing fixed image, with networking disabled.
Does not import ROS, the adapter conversion, or the mission assertions.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit_basic import audit as flight_audit


def sample(poly, stamp, derivative):
    ends = np.cumsum(poly['duration'])
    knots = [poly['start_ns']+round(float(end)*1e9) for end in ends]
    if stamp in knots:
        index = knots.index(stamp)
        elapsed = poly['duration'][index]
    else:
        elapsed = (stamp-poly['start_ns'])/1e9
        index = min(int(np.searchsorted(ends, elapsed, side='right')), len(ends)-1)
        elapsed -= ends[index-1] if index else 0
    return [float(np.polyval(np.polyder(poly['coef_'+axis][6*index:6*(index+1)], derivative), elapsed)) for axis in 'xyz']


def source_peak(poly, derivative):
    values = []
    for index, duration in enumerate(poly['duration']):
        coefficients = [np.polyder(poly['coef_'+axis][6*index:6*(index+1)], derivative) for axis in 'xyz']
        norm_squared = np.poly1d([0.])
        for c in coefficients:
            norm_squared += np.polymul(c,c)
        roots = np.roots(np.polyder(norm_squared))
        times = [0., duration]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<duration]
        values.extend(math.sqrt(sum(float(np.polyval(c,t))**2 for c in coefficients)) for t in times)
    return max(values)


def authorized_holds(polys, segments, goal):
    """Independent raw-record check; never extrapolate an unrecorded final hold."""
    holds = []
    for poly in polys:
        end = poly['start_ns']+round(sum(poly['duration'])*1e9)
        pva = [sample(poly,end,d) for d in range(3)]
        if math.dist(pva[0],goal)>=.03 or any(math.dist(x,[0,0,0])>=1e-4 for x in pva[1:]):
            continue
        for segment in segments:
            if not segment['is_final']:
                continue
            start = segment['start_ns']
            for vehicle in segment['vehicles']:
                if vehicle['vehicle_id']!='drone0':
                    continue
                points = vehicle['points']
                stop = start+points[-1]['t_ns']
                if not end<stop<=end+100_000_000:
                    continue
                terminal = points[-1]
                if math.dist(terminal['p'],pva[0])>1e-9 or any(math.dist(terminal[k],[0,0,0])>1e-12 for k in ('v','a')):
                    continue
                if any(start+point['t_ns']==end and max(math.dist(point[k],v) for k,v in zip(('p','v','a'),pva))<=1e-9 for point in points):
                    holds.append((end,stop,pva[0]))
    return holds


def reference_errors(polys, refs, holds):
    errors = []
    for ref in refs:
        candidates = []
        for poly in polys:
            if poly['start_ns']<=ref['stamp_ns']<=poly['start_ns']+round(sum(poly['duration'])*1e9):
                candidates.append([math.dist(sample(poly,ref['stamp_ns'],d),ref[k]) for d,k in enumerate(('position','velocity','acceleration'))])
        candidates.extend([math.dist(p,ref['position']),math.dist([0,0,0],ref['velocity']),math.dist([0,0,0],ref['acceleration'])]
                          for start,end,p in holds if start<ref['stamp_ns']<=end)
        errors.append(min(candidates,key=max) if candidates else [math.inf]*3)
    return errors


def planning_rejections(rejected):
    input_keys = {'odom_stamp','cloud_stamp','odom_contract','cloud_frame','cloud_contract'}
    return {key:value for key,value in rejected.items() if key not in input_keys and value}


def execution_counts_valid(state, navigation_count):
    return navigation_count >= state['counts']['trajectories'] >= state['counts']['segments'] == state['revision'] > 0


def audit(out):
    common = flight_audit(out, 'ego')
    baseline = flight_audit(out, 'basic')
    mission = out/'mission-ego'
    report = json.loads((mission/'result.json').read_text())
    rows = [json.loads(line) for line in (mission/'flight.jsonl').read_text().splitlines()]
    polys = json.loads((mission/'polynomials.json').read_text())
    refs = json.loads((mission/'references.json').read_text())
    checks = dict(common['checks'])
    checks['same_session_basic_flight_passed'] = baseline['passed']
    checks['criteria_unchanged'] = report['gates'] == json.loads((mission/'criteria.json').read_text())
    for stage in ('basic','ego'):
        reset = json.loads((out/('mission-'+stage)/'ground-hold.json').read_text())
        checks[stage+'_disarmed_hold_before_flight'] = ('Disarmed' in reset[0]['output'] and
            any('navigation mode: Hold' in r['output'] for r in reset[2:]) and
            'pre_flight_checks_pass: True' in reset[-1]['output'])
    state = report['ego_state']
    checks['one_goal_succeeded_and_stopped'] = state['state']=='SUCCEEDED' and state['stop_confirmed'] and state['counts']['goals']==1 and state['completed']==['drone0']
    navigation_polys = [p for p in polys if source_peak(p,1)>1e-10]
    checks['native_plans_and_execution_updates'] = execution_counts_valid(state,len(navigation_polys))
    checks['no_rejected_plans'] = not planning_rejections(state['rejected'])
    checks['one_metre_target'] = abs(math.dist(report['goal_enu'],report['ego_start_position'])-1.)<1e-6
    checks['clear_observed_corridor'] = any(c['cloud_clearance_m'] >= .6 and math.dist(
        [report['ego_start_position'][0]+c['direction'][0],report['ego_start_position'][1]+c['direction'][1],report['ego_start_position'][2]], report['goal_enu'])<1e-6 for c in report['goal_candidates'])
    active = [r for r in rows if r['phase']=='ego']
    odom = [r for r in active if r.get('topic')=='odom']
    metrics = dict(common['metrics'])
    metrics.update(native_trajectories=len(polys), native_navigation_trajectories=len(navigation_polys),
        stationary_native_outputs=len(polys)-len(navigation_polys), adapter_accepted_trajectories=state['counts']['trajectories'],
        adapter_rejections=state['rejected'], execution_revisions=state['revision'], references=len(refs),
        goal_error_m=math.dist(odom[-1]['position'],report['goal_enu']) if odom else math.inf,
        duration_wall_s=active[-1]['wall']-active[0]['wall'] if active else math.inf,
        actual_speed_peak_m_s=max((r['speed'] for r in odom),default=math.inf))
    checks['goal_accuracy'] = metrics['goal_error_m'] <= .25 and report.get('goal_error_m', math.inf) <= .25
    checks['goal_timeout'] = metrics['duration_wall_s'] <= 60
    checks['continuous_navigation'] = len(odom)>=30 and max((b['wall']-a['wall'] for a,b in zip(odom,odom[1:])),default=math.inf)<1.
    checks['trajectory_reference_consumed'] = len(refs)>=20 and sum(r.get('topic')=='reference_trajectory' for r in active)>=20
    checks['ego_controller_and_px4_consumed'] = sum(r.get('topic') in ('actuator','actuator_twist') for r in active)>=20 and sum(r.get('topic')=='px4_setpoint' for r in active)>=20
    segments = json.loads((mission/'segments.json').read_text()) if (mission/'segments.json').exists() else []
    holds = authorized_holds(polys, segments, report['goal_enu'])
    errors = reference_errors(polys, refs, holds)
    metrics['explicit_terminal_holds'] = len(holds)
    metrics['max_pva_error'] = [max((e[i] for e in errors),default=math.inf) for i in range(3)]
    checks['same_time_original_pva_preserved'] = bool(refs) and max(metrics['max_pva_error'])<=1e-4
    for order,label,limit in [(1,'velocity',.2415),(2,'acceleration',.3675)]:
        metrics['native_peak_'+label] = max((source_peak(p,order) for p in polys),default=math.inf)
        metrics['reference_peak_'+label] = max((math.sqrt(sum(v*v for v in r[label])) for r in refs),default=math.inf)
        checks['native_'+label+'_bound'] = metrics['native_peak_'+label] <= limit+1e-8
        checks['reference_'+label+'_bound'] = metrics['reference_peak_'+label] <= limit+1e-8
    checkpoints = {r['collision_checkpoint']:r for r in rows if 'collision_checkpoint' in r}
    checks['no_new_collision_during_ego'] = ('before_ego' in checkpoints and 'after_ego' in checkpoints and
        checkpoints['before_ego']['collision']['time_stamp']==checkpoints['after_ego']['collision']['time_stamp'])
    checks['stop_confirmed_before_landing'] = any(r.get('ego_state',{}).get('state')=='SUCCEEDED' and r['ego_state'].get('stop_confirmed') and r['phase']=='ego' for r in rows)
    metrics['latest_pose_reference_tracking_difference_m'] = max((math.dist(r['position'],r['actual']) for r in refs),default=math.inf)
    result = dict(passed=all(checks.values()),passed_checks=sum(checks.values()),total_checks=len(checks),checks=checks,metrics=metrics,
        scope='One-metre RoboPi EGO/Adapter/AS2 trajectory execution in AirSim/PX4; no obstacle-avoidance, fault-injection or mixed-fleet acceptance',
        limitations=['PVA source match takes the best covering native revision; raw records retained.',
                    'Tracking difference uses latest received odometry, not synchronized truth.',
                    'Native/reference speed bounds do not assert identical actual-vehicle speed bounds.'])
    (out/'ego-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    if odom and refs:
        t0=active[0]['sim']
        fig,axes=plt.subplots(3,1,figsize=(10,9),constrained_layout=True)
        axes[0].plot([r['position'][0] for r in odom],[r['position'][1] for r in odom],label='Actual odometry')
        axes[0].plot([r['position'][0] for r in refs],[r['position'][1] for r in refs],label='AS2 reference')
        axes[0].scatter(*report['goal_enu'][:2],marker='x',label='Goal')
        axes[0].set(xlabel='ENU x (m)',ylabel='ENU y (m)',title='RoboPi EGO short flight')
        axes[0].axis('equal');axes[0].legend()
        axes[1].plot([r['sim']-t0 for r in odom],[r['speed'] for r in odom],label='Actual speed')
        axes[1].plot([(r['stamp_ns']/1e9)-t0 for r in refs],[math.sqrt(sum(v*v for v in r['velocity'])) for r in refs],label='Reference speed')
        axes[1].axhline(.2415,color='gray',linestyle='--',label='Reference limit')
        axes[1].set(xlabel='Simulation time from EGO phase (s)',ylabel='Speed (m/s)');axes[1].legend()
        for i,label in enumerate(('Position (m)','Velocity (m/s)','Acceleration (m/s^2)')):
            axes[2].semilogy([(r['stamp_ns']/1e9)-t0 for r in refs],[max(e[i],1e-12) for e in errors],label=label)
        axes[2].axhline(1e-4,color='gray',linestyle='--',label='PVA tolerance')
        axes[2].set(xlabel='Simulation time from EGO phase (s)',ylabel='Native-to-AS2 PVA difference');axes[2].legend()
        for axis in axes:axis.grid(True,alpha=.25)
        fig.savefig(out/'ego-short-audit.png',dpi=160)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    result=audit(parser.parse_args().output)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
