#!/usr/bin/env python3
"""Offline summaries of actual state-probe traces; no ROS dependencies."""
import argparse
from collections import defaultdict
import json
from pathlib import Path


def analyze(rows, warmup=15.):
    grouped=defaultdict(list);events=[]
    for row in rows:
        if row.get('kind')=='sample':
            if row['elapsed_s']>=warmup:grouped[row['topic']].append(row)
        else:events.append(row)
    result={}
    for topic,samples in grouped.items():
        if not samples:continue
        gaps=[(b['elapsed_s']-a['elapsed_s'],a,b) for a,b in zip(samples,samples[1:])]
        gap,a,b=max(gaps,key=lambda x:x[0],default=(0,None,None))
        ages=[s['age_s'] for s in samples if 'age_s' in s]
        result[topic]={'count':len(samples),'unique_stamps':len({s['stamp_ns'] for s in samples}),
            'max_receipt_gap_s':gap,'age_min_s':min(ages,default=None),'age_max_s':max(ages,default=None),
            'largest_gap_interval':None if a is None else {
                'start_elapsed_s':a['elapsed_s'],'end_elapsed_s':b['elapsed_s'],
                'start_wall_ns':a['wall_ns'],'end_wall_ns':b['wall_ns'],
                'source_stamp_step_s':(b['stamp_ns']-a['stamp_ns'])/1e9}}
    return {'warmup_s':warmup,'topics':result,'events':events}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path)
    parser.add_argument('--warmup',type=float,default=15.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    with args.trace.open() as stream:result=analyze((json.loads(line) for line in stream),args.warmup)
    text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text)
    else:print(text,end='')


if __name__=='__main__':main()
