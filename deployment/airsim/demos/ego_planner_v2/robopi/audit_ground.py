"""Audit a stopped ground session from board archive and independent PC observations."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import tarfile
from interfaces import topic_map


def audit(out):
    archive = max(out.glob('board-*.tar.gz'), key=lambda p: p.name)
    expected = json.loads(archive.with_suffix('.sha256.json').read_text())['sha256']
    checks = {}
    checks['archive_sha256'] = hashlib.sha256(archive.read_bytes()).hexdigest() == expected
    with tarfile.open(archive) as tar:
        def read(name):
            member = next(m for m in tar.getmembers() if m.name.removeprefix('./') == name)
            return tar.extractfile(member).read().decode()
        rows = [json.loads(line) for line in read('status-history.jsonl').splitlines()]
        stop = json.loads(read('stopped.json'))
        logs = {name: read(name+'.log') for name in ('as2', 'ego', 'runtime')}
    start = None
    longest = 0
    for row in rows:
        if row['ready']:
            if start is None:
                start = row['written_at']
            longest = max(longest, row['written_at']-start)
        else:
            start = None
    final_streak = rows[-1]['written_at']-start if start is not None else 0
    checks['continuous_ready_30_seconds'] = final_streak >= 30
    checks['never_armed'] = all(not r.get('armed') and not r.get('px4_armed') for r in rows)
    checks['no_goal_submitted'] = all(r.get('ego_goals') in (None, 0) for r in rows)
    safe = stop['safe_state']
    checks['stop_while_disarmed_idle_waiting'] = bool(safe['fresh'] and safe['armed'] is False and
        safe['px4_armed'] is False and safe['behavior_idle'] and safe['ego_state']=='WAITING' and safe['ego_goals']==0)
    checks['both_launches_exited_zero'] = stop['children']=={'as2':0,'ego':0}
    exit_codes = {name:[int(code) for code in re.findall(r'exit code (-?\d+)', text)] for name,text in logs.items()}
    clean_exits = {name:logs[name].count('process has finished cleanly') for name in ('as2','ego')}
    checks['no_abnormal_child_exit'] = (all(code in (0,-2) for codes in exit_codes.values() for code in codes)
                                      and clean_exits == {'as2':8,'ego':2})
    checks['no_forced_kill_or_uncaught_exception'] = not any(
        term in text for text in logs.values() for term in ('SIGKILL', 'uncaught exception', 'escalating to'))
    obs = json.loads((out/'observer.json').read_text())
    checks['independent_observation_at_least_30_seconds'] = obs['duration'] >= 30
    for topic,row in obs['topics'].items():
        checks['messages:'+topic] = row['count'] >= 30 and row.get('receipt_age',1e9)<.75 and row.get('max_gap',1e9)<.75
        checks['unique_publisher:'+topic] = len(obs['graph'][topic]['publishers'])==1
    authority = {'/clock':'platform_clock_bridge','/drone0/sensor_measurements/odom':'platform',
        '/drone0/sensor_measurements/Lidar/points':'platform_sensor_adapter',
        '/drone0/navigation/odom':'navigation_inputs','/drone0/navigation/points':'navigation_inputs',
        '/ego/status':'ego_as2_adapter','/drone0/ego_planner/planning/heartbeat':'ego_planner'}
    for topic,name in authority.items():
        checks['authority:'+topic] = [p['node'] for p in obs['graph'][topic]['publishers']]==[name]
    adapter = json.loads(obs['topics']['/ego/status']['data'])
    config = json.loads((out/'config.json').read_text())
    if config.get('vehicle'):
        mapping = topic_map(config)
        checks['scoped_actual_topics'] = all(obs['resolved_topics'].get(t)==mapping.get(t,t) for t in obs['topics'])
        checks['scoped_adapter_authority'] = [p['namespace'] for p in obs['graph']['/ego/status']['publishers']]==['/drone0']
        checks['scoped_vehicle_ownership'] = adapter.get('vehicles')==['drone0'] and adapter.get('interface_scope')=='drone0'
        checks['no_global_or_peer_ego_endpoints'] = not any(t.startswith('/ego/') or t.startswith('/drone1/ego/') for t,_ in obs['all_topics'])
    checks['pc_observed_waiting_zero_goals'] = adapter['state']=='WAITING' and adapter['counts']['goals']==0
    result = dict(success=all(checks.values()), passed=sum(checks.values()), total=len(checks),
        checks=checks, board_samples=len(rows), longest_ready_seconds=longest,
        final_ready_seconds=final_streak, observer_duration=obs['duration'], child_exit_codes=exit_codes,
        clean_node_exits=clean_exits,
        archive=archive.name, archive_sha256=expected,
        scope='DDS and disarmed single-aircraft ground readiness only; no flight, planning goal or fault recovery')
    (out/'ground-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    result=audit(parser.parse_args().output)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['success'] else 1)
