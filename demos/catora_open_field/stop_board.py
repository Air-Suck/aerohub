"""Stop only processes carrying this exact CAT-ORA validation run's ownership tag."""
import json
import os
from pathlib import Path
import signal
import sys
import time

root=Path(sys.argv[1])
assert any(str(root).startswith('/data/aerohub/lib/'+app+'/validation/run-') for app in ('catora','swarm-formation'))
expected=('CATORA_APP_ROOT='+str(root)).encode()

def owned():
    result={}
    for path in Path('/proc').glob('[0-9]*'):
        try:
            if expected not in (path/'environ').read_bytes().split(b'\0'): continue
            fields=(path/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]=='Z':continue
            result[int(path.name)]=fields[19]
        except (OSError,IndexError):pass
    return result

def send(processes,sig):
    current=owned()
    for pid,ticks in processes.items():
        if current.get(pid)!=ticks:continue
        try:os.kill(pid,sig)
        except ProcessLookupError:pass

processes=owned()
send(processes,signal.SIGTERM)
deadline=time.monotonic()+8
while owned() and time.monotonic()<deadline:time.sleep(.1)
send(owned(),signal.SIGKILL)
time.sleep(.2)
remaining=owned()
print(json.dumps({'initial_processes':len(processes),'remaining':remaining}),flush=True)
if remaining:raise SystemExit(1)
