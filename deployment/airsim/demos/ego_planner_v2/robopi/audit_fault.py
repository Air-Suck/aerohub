"""Offline board fault audit from raw events, coefficients and board lifecycle logs."""
import argparse
import json
import math
from pathlib import Path
import tarfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit_basic import audit as flight_audit
from audit_ego import sample, authorized_holds, reference_errors


def audit(out):
    common=flight_audit(out,'fault')
    baseline=flight_audit(out,'basic')
    checks=dict(common['checks'])
    metrics=dict(common['metrics'])
    mission=out/'mission-fault'
    report=json.loads((mission/'result.json').read_text())
    rows=[json.loads(line) for line in (mission/'flight.jsonl').read_text().splitlines()]
    timeline=json.loads((mission/'fault-timeline.json').read_text())
    polys=json.loads((mission/'polynomials.json').read_text())
    refs=json.loads((mission/'references.json').read_text())
    config=json.loads((out/'config.json').read_text())
    case=config['fault_case'];injection=report['injection_wall']
    checks['same_session_basic_flight']=baseline['passed']
    checks['criteria_frozen']=report['gates']==json.loads((mission/'criteria.json').read_text())
    checks['correct_fault_case']=report['case']==case
    checks['mission_fault_assertions']=report.get('fault_assertions_passed') is True
    for stage in ('basic','fault'):
        reset=json.loads((out/('mission-'+stage)/'ground-hold.json').read_text())
        checks[stage+'_ground_hold_and_health']=('Disarmed' in reset[0]['output'] and
            any('navigation mode: Hold' in r['output'] for r in reset[2:]) and 'pre_flight_checks_pass: True' in reset[-1]['output'])
    states=[r for r in rows if 'ego_state' in r and r['wall']>=injection]
    failed=[r for r in states if r['ego_state']['state']=='FAILED']
    stopped=[r for r in failed if r['ego_state']['stop_confirmed']]
    metrics['detection_wall_s']=failed[0]['wall']-injection if failed else math.inf
    metrics['stop_confirmation_wall_s']=stopped[0]['wall']-injection if stopped else math.inf
    expected='CANCELED:' if case=='cancel' else 'STALE_INPUT:'
    checks['specific_failure']=bool(failed) and failed[0]['ego_state']['reason'].startswith(expected)
    checks['detection_bound']=metrics['detection_wall_s']<=1.5
    checks['stop_confirmation_bound']=metrics['stop_confirmation_wall_s']<=5
    checks['closed_state_does_not_resume']=bool(failed) and all(r['ego_state']['state']=='FAILED' for r in states if r['wall']>=failed[0]['wall'])
    checks['new_goal_rejected']=report['ego_state']['rejected'].get('goal',0)>=1 and report.get('no_old_task_resume') is True
    checks['no_further_execution_updates']=bool(stopped) and all(r['ego_state']['revision']==stopped[0]['ego_state']['revision'] and
        r['ego_state']['counts']['segments']==stopped[0]['ego_state']['counts']['segments'] for r in stopped)
    odom=[r for r in rows if r.get('topic')=='odom']
    pre=[r for r in odom if r['phase']=='ego' and r['wall']<injection]
    after=[r for r in odom if r['wall']>=injection and r['phase'] in ('ego','fault_hold')]
    checks['moving_at_injection']=bool(pre) and pre[-1]['speed']>=.1 and math.dist(pre[-1]['position'],report['ego_start_position'])>=.25
    metrics['prefault_peak_actual_speed_m_s']=max((r['speed'] for r in pre),default=math.inf)
    checks['prefault_flight_speed_bound']=metrics['prefault_peak_actual_speed_m_s']<=.5
    checks['prefault_real_planning']=len(refs)>=20 and report['pre_fault']['ego_state']['counts']['segments']>0
    checks['live_as2_navigation']=len(after)>30 and max((b['wall']-a['wall'] for a,b in zip(after,after[1:])),default=math.inf)<1
    first=None;stable_end=None
    for r in after:
        if r['speed']>.1:first=None;continue
        if first is None:first=r
        if r['sim']-first['sim']>=1:
            stable_end=r;break
    metrics['physical_stop_wall_s']=first['wall']-injection if stable_end else math.inf
    checks['physical_stop_bound']=metrics['physical_stop_wall_s']<=5
    metrics['residual_displacement_m']=max((math.dist(r['position'],report['pre_fault']['position']) for r in after if stable_end and r['wall']<=stable_end['wall']),default=math.inf)
    checks['residual_displacement']=metrics['residual_displacement_m']<=.75
    hold=[r for r in after if r['phase']=='fault_hold']
    metrics['hold_sim_s']=hold[-1]['sim']-hold[0]['sim'] if hold else 0
    metrics['post_stop_drift_m']=max((math.dist(r['position'],report['stopped_position']) for r in hold),default=math.inf)
    checks['hold_duration']=metrics['hold_sim_s']>=3
    checks['hold_drift']=metrics['post_stop_drift_m']<=.3
    checks['no_old_references_after_stop']=bool(hold) and not any(r.get('topic')=='reference_trajectory' and r['wall']>hold[0]['wall']+.6 and r['phase']=='fault_hold' for r in rows)
    checks['action_terminated']=bool(report['terminal_action_status']) and report['terminal_action_status'][-1] in (5,6)
    checks['idle_behavior_before_land']=any(r.get('timed_behavior_status')==0 and r['phase']=='fault_hold' for r in rows)
    checks['timeline_dense']=len(timeline)>30 and max((b['wall']-a['wall'] for a,b in zip(timeline,timeline[1:])),default=math.inf)<.5
    checks['no_new_collision']=report.get('ego_collision_before')==report.get('ego_collision_after') and 'ego_collision_after' in report
    checks['landing_physics_scene_consistent']=report.get('max_landing_physics_scene_z_gap_m',math.inf)<=.3
    checks['normal_land_after_confirmed_stop']=bool(stopped) and min((r['wall'] for r in rows if r['phase']=='landing'),default=0)>stopped[0]['wall']
    segments=json.loads((mission/'segments.json').read_text()) if (mission/'segments.json').exists() else []
    holds=authorized_holds(polys,segments,report['goal_enu'])
    errors=[max(e) for e in reference_errors(polys,refs,holds)]
    metrics['native_reference_pva_error']=max(errors,default=math.inf)
    checks['native_pva_consumed']=len(refs)>=20 and metrics['native_reference_pva_error']<=1e-4
    graph=report['input_graph']
    checks['planner_gate_graph']=all(graph[t]['publishers']==['acceptance_fault_injector'] and 'ego_as2_adapter' in graph[t]['subscribers'] for t in ('/acceptance/drone0/odom','/acceptance/drone0/points'))
    checks['original_navigation_unmodified']=all(graph[t]['publishers']==['navigation_inputs'] for t in ('/drone0/navigation/odom','/drone0/navigation/points'))
    injector=report['final_injector']
    if case=='cancel':
        checks['cancel_service_success']=any(r.get('service')=='/ego/cancel' and r.get('success') for r in rows)
        checks['no_unrequested_injection']=not injector['events']
    elif case=='planner_exit':
        events=[r for r in injector['events'] if r['kind']=='planner_exit']
        checks['one_owned_planner_killed']=len(events)==1 and events[0]['run_id']==config['run_id'] and events[0]['signal']=='SIGKILL' and '__ns:=/drone0' in events[0]['argv'] and events[0]['domain']==['ROS_DOMAIN_ID='+str(config['domain'])] and events[0]['argv'][0].startswith(config['candidate']+'/install/') and events[0]['parent']>1 and bool(events[0]['ticks'])
    else:
        events=[r for r in injector['events'] if r['kind']==case]
        checks['input_blocked_then_restored']=len(events)==2 and events[0]['blocked'] and not events[1]['blocked']
        checks['no_forwarding_while_blocked']=len(events)==2 and events[0]['counts']['forwarded']==events[1]['counts']['forwarded']
        checks['live_input_dropped']=len(events)==2 and events[1]['counts']['dropped']-events[0]['counts']['dropped']>=10
        checks['live_input_restored']=len(events)==2 and injector['counts'][case]['forwarded']>=events[1]['counts']['forwarded']+10
        checks['only_requested_input_gated']=all(r['kind']==case for r in injector['events'])
    result=dict(passed=all(checks.values()),case=case,passed_checks=sum(checks.values()),total_checks=len(checks),checks=checks,metrics=metrics,
        scope='RoboPi single UAV planner-boundary fault recovery; AS2 localization remains live; no dual-UAV or AS2 localization failure claim')
    (out/'fault-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    if pre and after:
        track=pre+after
        fig,axes=plt.subplots(2,1,figsize=(9,6),constrained_layout=True)
        axes[0].plot([r['wall']-injection for r in track],[r['speed'] for r in track],label='Actual speed')
        axes[0].axhline(.1,color='gray',linestyle='--',label='Stop threshold')
        axes[0].set(ylabel='Speed (m/s)',title='RoboPi fault: '+case);axes[0].legend()
        axes[1].plot([r['wall']-injection for r in after],[math.dist(r['position'],report['pre_fault']['position']) for r in after],label='Displacement since injection')
        axes[1].axhline(.75,color='gray',linestyle='--',label='Residual limit')
        axes[1].set(xlabel='Wall time from injection (s)',ylabel='Displacement (m)');axes[1].legend()
        for axis in axes:axis.axvline(0,color='red',linestyle=':');axis.grid(True,alpha=.25)
        fig.savefig(out/'fault-audit.png',dpi=150);plt.close(fig)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('outputs',nargs='+',type=Path)
    results=[audit(path) for path in parser.parse_args().outputs]
    print(json.dumps(results,indent=2))
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)
