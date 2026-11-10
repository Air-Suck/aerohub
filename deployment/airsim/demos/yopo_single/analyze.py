#!/usr/bin/env python3
"""Summarize preserved raw task/plan evidence, without upgrading failed runs."""
import argparse
import json
import math
from pathlib import Path


def analyze(path):
    result = json.loads((path/'result.json').read_text())
    rows = [json.loads(line) for line in (path/'mission.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (path/'plans.jsonl').read_text().splitlines()] if (path/'plans.jsonl').exists() else []
    accepted = [r for r in events if r.get('event') == 'ack' and r.get('accepted')]
    stopping = [r for r in rows if r['state']=='STOPPING']
    stopped = next((r for r in rows if r['state']=='STOPPED'),None)
    hold = 0.
    if stopped and stopping:
        end = rows.index(stopped)
        start = end
        while start > 0:
            r = rows[start-1]
            if r['state'] not in ('STOPPING','STOPPED') or math.sqrt(sum(x*x for x in r.get('velocity',[math.inf]))) >= .2:
                break
            start -= 1
        hold = stopped['sim']-rows[start]['sim']
    publishers = {p['node'] for values in result.get('motion_reference_publishers',{}).values() for p in values}
    allowed = {'TakeoffBehavior','LandBehavior','GoToBehavior','FollowPathBehavior'}
    summary = dict(path=str(path), success=result['success'], motion_requested=result['motion_requested'],
                   arrived_5m=result['arrived_5m'], stopped=result['stopped'],
                   land_success=result['land_success'], disarmed=result['disarmed'],
                   collision=result['collision'], fault=result.get('fault','none'),
                   stop_reason=result.get('stop_reason'), accepted_revisions=len(accepted),
                   low_speed_hold_ros_s=hold,
                   stop_elapsed_wall_s=(stopped['wall']-stopping[0]['wall'] if stopped and stopping else None),
                   only_native_behavior_reference_publishers=bool(publishers) and publishers <= allowed)
    if result.get('error'): summary['error']=result['error']
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence_root',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    summaries=[]
    for path in sorted(args.evidence_root.glob('sitl-*/*/result.json')):
        try: summaries.append(analyze(path.parent))
        except (OSError,KeyError,ValueError) as error:
            summaries.append(dict(path=str(path.parent), evidence_error=str(error)))
    text=json.dumps(summaries,indent=2,ensure_ascii=False)+'\n'
    if args.output:args.output.write_text(text)
    else:print(text,end='')


if __name__=='__main__':
    main()
