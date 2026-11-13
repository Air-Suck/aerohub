"""Independent offline audit of raw events; standard library, no ROS runtime.

Run only after flight and disarm. Evaluates native polynomial coefficients
directly, without importing the adapter's conversion or mission assertions.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))


def polynomial(poly, stamp, order):
    t=(stamp-poly['start_ns'])/1e9
    for index,duration in enumerate(poly['duration']):
        if t<=duration+1e-7:break
        t-=duration
    else:raise ValueError('outside source interval')
    result=[]
    for axis in 'xyz':
        coeff=poly['coef_'+axis][index*6:(index+1)*6]
        for _ in range(order):coeff=[c*(len(coeff)-i-1) for i,c in enumerate(coeff[:-1])]
        value=0.
        for c in coeff:value=value*t+c
        result.append(value)
    return result


def audit(folder):
    report=read(folder/'result.json');criteria=read(folder/'criteria.json')
    checks=[];metrics={}
    def check(name,passed,value=None):checks.append(dict(name=name,passed=bool(passed),value=value))
    check('mission_success',report.get('passed'))
    check('both_disarmed',report.get('both_disarmed'))
    check('no_emergency_fallback',not report.get('recovery'))
    check('criteria_frozen',report['gates']==criteria)
    check('client_exit_zero',(folder/'exit_code.txt').read_text().strip()=='0')
    for role in ('mission','compute'):
        state=read(folder/(role+'-inspect.json'))[0]['State']
        check(role+'_stopped_not_oom',not state['Running'] and not state['OOMKilled'],state)
    if not report.get('injection_wall'):
        check('fault_injected',False)
    else:
        injection=report['injection_wall']
        events=[]
        for line in (folder/'drone0/flight.jsonl').open():
            row=json.loads(line)
            if 'ego_state' in row:events.append(row)
        failed=[e for e in events if e['wall']>=injection and e['ego_state']['state']=='FAILED']
        stopped=[e for e in failed if e['ego_state']['stop_confirmed']]
        expected=criteria['expected_failure']+':'
        check('specific_failure',bool(failed) and failed[0]['ego_state']['reason'].startswith(expected))
        detected=failed[0]['wall']-injection if failed else math.inf
        confirmed=stopped[0]['wall']-injection if stopped else math.inf
        metrics.update(detection_wall_s=detected,stop_confirmation_wall_s=confirmed)
        check('detection_bound',detected<=criteria['detection_wall_s'],detected)
        check('stop_confirmation_bound',confirmed<=criteria['stop_confirmation_wall_s'],confirmed)
        check('closed_state_does_not_resume',bool(failed) and all(e['ego_state']['state']=='FAILED' for e in events if e['wall']>=failed[0]['wall']))
        check('goals_rejected_after_close',report['ego_state']['rejected'].get('goal',0)>=len(report['active']))
        check('fleet_separation',report['minimum_odom_separation_m']>=2.,report['minimum_odom_separation_m'])
        timeline=read(folder/'fault-timeline.json')
        check('timeline_dense',len(timeline)>30 and max((b['wall']-a['wall'] for a,b in zip(timeline,timeline[1:])),default=math.inf)<.5)
        if stopped:
            end=stopped[0]['wall']
            check('no_more_plan_segments',all(e['ego_state']['counts']['segments']==stopped[0]['ego_state']['counts']['segments'] for e in stopped))
            check('no_more_execution_revisions',all(e['ego_state']['revision']==stopped[0]['ego_state']['revision'] for e in stopped))
        for i in report['active']:
            vehicle=read(folder/f'drone{i}/result.json')
            data=[json.loads(line) for line in (folder/f'drone{i}/flight.jsonl').open()]
            odom=[e for e in data if e.get('topic')=='odom']
            after=[e for e in odom if e['wall']>=injection and e['phase'] in ('ego','holding')]
            pre=[e for e in odom if e['wall']<injection and e['phase']=='ego']
            check(f'drone{i}_moving_when_injected',bool(pre) and pre[-1]['speed']>=.1)
            check(f'drone{i}_actual_speed_envelope',bool(pre) and max(e['speed'] for e in pre)<=.5)
            first=None;stable_end=None
            for e in after:
                if e['speed']>.1:first=None;continue
                if first is None:first=e
                if e['sim']-first['sim']>=1.:
                    stable_end=e;break
            stop_time=first['wall']-injection if stable_end else math.inf
            check(f'drone{i}_physical_stop_bound',stop_time<=5.,stop_time)
            if stable_end:
                residual=max(math.dist(e['position'],report['pre_fault']['positions'][i]) for e in after if e['wall']<=stable_end['wall'])
                hold=[e for e in after if e['wall']>=stable_end['wall']]
                drift=max((math.dist(e['position'],stable_end['position']) for e in hold),default=math.inf)
                check(f'drone{i}_residual_displacement',residual<=.75,residual)
                check(f'drone{i}_hold_drift',drift<=.3,drift)
                check(f'drone{i}_hold_duration',bool(hold) and hold[-1]['sim']-hold[0]['sim']>=3.)
                # No resumption of any old trajectory during subsequent hold.
                refs=[e for e in data if e.get('topic')=='reference_trajectory' and e['wall']>stable_end['wall']+.6 and e['phase']=='holding']
                check(f'drone{i}_no_old_references',not refs,len(refs))
            modes=[e for e in data if e.get('topic')=='mode']
            check(f'drone{i}_final_px4_disarmed',bool(modes) and not modes[-1]['armed'])
            check(f'drone{i}_native_land',any(a.get('result')=='land_success' and a['success'] and a['status']==4 for a in vehicle['actions']))
            check(f'drone{i}_no_new_collision',vehicle.get('ego_collision_before')==vehicle.get('ego_collision_after') and 'ego_collision_after' in vehicle)
            check(f'drone{i}_ground_height',vehicle.get('landing_height_error_m',math.inf)<=.2)
            check(f'drone{i}_physics_scene_ground_gap',vehicle.get('max_landing_physics_scene_z_gap_m',math.inf)<=.3)
            check(f'drone{i}_action_terminated',report.get('terminal_action_status',{}).get(str(i),[0])[-1] in (5,6))
            polys=read(folder/f'drone{i}/polynomials.json');refs=read(folder/f'drone{i}/references.json')
            maximum=0.
            for ref in refs:
                errors=[]
                for poly in polys:
                    if poly['start_ns']<=ref['stamp_ns']<=poly['start_ns']+sum(poly['duration'])*1e9:
                        errors.append(max(math.dist(polynomial(poly,ref['stamp_ns'],order),ref[key])
                            for order,key in enumerate(('position','velocity','acceleration'))))
                maximum=max(maximum,min(errors,default=math.inf))
            check(f'drone{i}_native_pva_consumed',len(refs)>=20 and maximum<=1e-4,dict(samples=len(refs),maximum_error=maximum))
            # AS2's raw navigation stream continues during planner-only faults.
            check(f'drone{i}_navigation_continuous',len(after)>30 and max((b['wall']-a['wall'] for a,b in zip(after,after[1:])),default=math.inf)<1.)
        case=report['case'];injector=report.get('final_injector',{})
        if case=='planner_exit':
            kills=[e for e in injector.get('events',[]) if e['kind']=='planner_exit']
            runtime=(folder/'ego-runtime.log').read_text(encoding='utf-8-sig')
            check('real_owned_process_exit',len(kills)==1 and '__ns:=/drone0' in kills[0]['argv'] and 'exit code -9' in runtime)
        if case in ('odom','cloud'):
            gate=[e for e in injector.get('events',[]) if e['kind']==case]
            check('input_actually_blocked_then_restored',len(gate)==2 and gate[0]['blocked'] and not gate[1]['blocked'])
            if len(gate)==2:
                check('no_forwarding_while_blocked',gate[1]['counts']['forwarded']==gate[0]['counts']['forwarded'])
                check('live_input_dropped',gate[1]['counts']['dropped']>=10)
                check('live_input_restored',injector['counts'][case]['forwarded']>=gate[1]['counts']['forwarded']+10)
    result=dict(passed=all(c['passed'] for c in checks),checks_passed=sum(c['passed'] for c in checks),
        checks_total=len(checks),metrics=metrics,checks=checks,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (folder/'independent-fault-audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(run=folder.name,passed=result['passed'],checks=f"{result['checks_passed']}/{len(checks)}",
        failures=[c for c in checks if not c['passed']])))
    return result['passed']


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folders',nargs='+',type=Path)
    args=p.parse_args();results=[audit(folder) for folder in args.folders]
    raise SystemExit(0 if all(results) else 1)
