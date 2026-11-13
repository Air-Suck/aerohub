"""Independent raw-evidence audit of a RoboPi single-aircraft takeoff/hover/land."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import tarfile
from interfaces import topic_map


def audit(out, stage='basic'):
    mission = out/('mission-'+stage)
    report = json.loads((mission/'result.json').read_text())
    rows = [json.loads(line) for line in (mission/'flight.jsonl').read_text().splitlines()]
    checks = {}
    checks['client_success'] = report['passed'] and json.loads((mission/'client-exit.json').read_text())['returncode'] == 0
    checks['no_recovery'] = not report['recovery'] and 'error' not in report and 'recovery_error' not in report
    checks['native_takeoff_and_land'] = ([a['result'] for a in report['actions']] == ['takeoff_success','land_success'] and
        all(a['status'] == 4 and a['success'] for a in report['actions']))
    checks['arm_and_offboard_acknowledged'] = ([(c['service'], c['value'], c['ack']) for c in report['commands']] ==
        [('set_arming_state', True, 0), ('set_offboard_mode', True, 0)])
    hold = [r for r in rows if r.get('topic') == 'odom' and r['phase'] == 'holding']
    origin = report['origin_enu']
    metrics = dict(hover_samples=len(hold),
        hover_height_error_m=max((abs(r['position'][2]-origin[2]-2) for r in hold), default=math.inf),
        hover_horizontal_drift_m=max((math.dist(r['position'][:2],origin[:2]) for r in hold), default=math.inf),
        hover_speed_m_s=max((r['speed'] for r in hold), default=math.inf))
    checks['hover_samples'] = len(hold) >= 20
    # Callback timestamps bracket the five-second interval, with one odometry period tolerance.
    checks['hover_duration'] = bool(hold) and hold[-1]['sim']-hold[0]['sim'] >= 4.9
    checks['hover_height'] = metrics['hover_height_error_m'] <= .3
    checks['hover_horizontal'] = metrics['hover_horizontal_drift_m'] <= .5
    checks['hover_speed'] = metrics['hover_speed_m_s'] <= .3
    modes = [r for r in rows if r.get('topic') == 'mode']
    platforms = [r for r in rows if r.get('topic') == 'platform']
    checks['initially_disarmed'] = bool(modes) and not modes[0]['armed']
    checks['finally_disarmed'] = bool(modes and platforms) and not modes[-1]['armed'] and not platforms[-1]['armed'] and report['final_disarmed']
    checks['armed_offboard_in_air'] = any(r['armed'] for r in modes) and all(r['armed'] and r['offboard'] for r in modes if r['phase'] in ('taking_off','settling','holding'))
    checks['flight_envelope'] = all(r['speed'] <= 2 and all(math.isfinite(v) for v in r['position']) for r in rows if r.get('topic') == 'odom')
    scene = [r for r in rows if 'truth' in r and 'scene_pose' in r]
    metrics['landing_height_error_m'] = abs(scene[-1]['scene_pose']['position']['z_val']-report['truth_origin_ned'][2]) if scene else math.inf
    checks['landed_at_ground'] = report.get('landing_height_error_m', math.inf) <= .2 and metrics['landing_height_error_m'] <= .2
    checks['actual_controller_output'] = sum(r.get('topic') in ('actuator','actuator_twist') for r in rows) >= 30
    checks['actual_px4_setpoints'] = sum(r.get('topic') == 'px4_setpoint' for r in rows) >= 30
    peer = [r for r in rows if 'inactive_vehicle' in r]
    metrics['inactive_peer_displacement_m'] = max((r['displacement_m'] for r in peer), default=math.inf)
    checks['inactive_peer_stationary'] = len(peer) >= 10 and metrics['inactive_peer_displacement_m'] <= .15
    checks['inactive_peer_never_observed_armed'] = not any(r.get('topic') == 'drone1_mode' and r['armed'] for r in rows)
    offsets = [json.loads(line) for line in (mission/'clock-offsets.jsonl').read_text().splitlines()]
    metrics['max_clock_bound_s'] = max((abs(r['offset'])+r['rtt']/2 for r in offsets), default=math.inf)
    checks['runtime_clock_gate'] = metrics['max_clock_bound_s'] <= .5 and not (mission/'recovery.request').exists()
    obs = json.loads((out/'observer.json').read_text())
    checks['ground_observation'] = obs['duration'] >= 30 and all(r['count'] >= 30 and r['receipt_age'] < .75 and r['max_gap'] < .75 for r in obs['topics'].values())
    checks['ground_unique_publishers'] = all(len(g['publishers']) == 1 for g in obs['graph'].values())
    archive = max(out.glob('board-*.tar.gz'))
    checks['board_archive_hash'] = hashlib.sha256(archive.read_bytes()).hexdigest() == json.loads(archive.with_suffix('.sha256.json').read_text())['sha256']
    with tarfile.open(archive) as tar:
        def read(name):
            return tar.extractfile(next(m for m in tar.getmembers() if m.name.removeprefix('./') == name)).read().decode()
        stop = json.loads(read('stopped.json'))
        logs = {name:read(name+'.log') for name in ('as2','ego','runtime')}
        states = [json.loads(line) for line in read('status-history.jsonl').splitlines()]
    config = json.loads((out/'config.json').read_text())
    if config.get('vehicle'):
        mapping = topic_map(config)
        checks['scoped_client_mapping'] = report.get('interface_mapping')==mapping
        checks['scoped_ground_topics'] = all(obs.get('resolved_topics',{}).get(t)==mapping.get(t,t) for t in obs['topics'])
        checks['scoped_adapter_publisher'] = [p['namespace'] for p in obs['graph']['/ego/status']['publishers']]==['/drone0']
    intentional_exit = config.get('fault_case') == 'planner_exit'
    safe = stop['safe_state']
    checks['safe_stop'] = safe['fresh'] and safe['armed'] is False and safe['px4_armed'] is False and safe['behavior_idle']
    checks['launches_exited_zero'] = stop['children'] == dict(as2=0,ego=0)
    expected_ego = (2 if intentional_exit else 3) if config.get('fault_case') else 2
    checks['all_expected_nodes_exited_cleanly'] = {name:logs[name].count('process has finished cleanly') for name in ('as2','ego')} == dict(as2=8,ego=expected_ego)
    errors = [line for s in logs.values() for line in s.splitlines() if re.search(r'exit code (-?\d+)',line)]
    checks['no_unexpected_forced_exit'] = not any(term in s for s in logs.values() for term in ('SIGABRT','uncaught exception','escalating to')) and all(
        int(re.search(r'exit code (-?\d+)',line)[1]) in (0,-2) or
        (intentional_exit and 'ego_planner_node' in line and 'exit code -9' in line) for line in errors)
    if intentional_exit:
        checks['expected_planner_exit_logged'] = sum('ego_planner_node' in line and 'exit code -9' in line for line in errors)==1
    if stage == 'basic':
        checks['ego_idle_no_planning_goal'] = all(s.get('ego_goals') in (None,0) and s.get('ego_state') in (None,'WAITING') for s in states if s['written_at'] <= rows[-1]['wall'])
    elif stage == 'ego':
        checks['ego_successfully_stopped'] = safe['ego_state'] == 'SUCCEEDED' and safe['stop_confirmed']
    else:
        checks['fault_stopped_and_closed'] = safe['ego_state'] == 'FAILED' and safe['stop_confirmed']
    result = dict(passed=all(checks.values()), passed_checks=sum(checks.values()), total_checks=len(checks), checks=checks, metrics=metrics,
        scope='RoboPi AS2 takeoff/hover/native land common checks; trajectory/fault semantics audited separately' if stage != 'basic' else
        'RoboPi AS2 single-aircraft takeoff/hover/native land in AirSim/PX4; EGO running idle; no EGO trajectory or fault recovery acceptance')
    (out/('basic-audit.json' if stage == 'basic' else stage+'-common-audit.json')).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    result = audit(parser.parse_args().output)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
