"""Reject accidental changes to application acceptance while tuning algorithms."""
import json
from pathlib import Path
import sys

def validate(task, record):
    mismatches={key:{'required':value,'task':task.get(key)}
                for key,value in record['criteria'].items() if task.get(key)!=value}
    if mismatches:
        raise ValueError('Application criteria changed: '+json.dumps(mismatches)+
                         '; keep algorithm tuning separate from acceptance experiments')
    return record['id']

if __name__=='__main__':
    task,criteria=map(Path,sys.argv[1:])
    print(json.dumps({'criteria_id':validate(json.loads(task.read_text()),json.loads(criteria.read_text())),'passed':True}))
