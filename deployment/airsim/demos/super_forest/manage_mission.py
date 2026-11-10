"""Board-side lifecycle for one evidence-scoped SUPER mission."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

APP=Path(__file__).resolve().parent


def alive(state, out, script="mission.py"):
    try:
        pid=state['pid'];text=Path(f'/proc/{pid}/stat').read_text();fields=text[text.rfind(')')+2:].split()
        return (fields[0]!='Z' and fields[19]==state['ticks'] and
            Path('/proc/sys/kernel/random/boot_id').read_text().strip()==state['boot_id'] and
            str(APP/script).encode() in Path(f'/proc/{pid}/cmdline').read_bytes().split(bytes([0])) and
            ('SUPER_EVIDENCE_ROOT='+str(out)).encode() in Path(f'/proc/{pid}/environ').read_bytes().split(bytes([0])))
    except (OSError,KeyError):return False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['start','status','stop'])
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=['ground','short','short-cancel','short-input-loss','short-takeover','forest8'],default='short')
    parser.add_argument('--rpc-host')
    parser.add_argument('--kind',choices=['mission','compute'],default='mission')
    args=parser.parse_args();out=args.output
    if not out.is_absolute() or '..' in out.parts or not out.is_relative_to(APP.parent/'runtime/super-runs'):
        raise ValueError('Evidence must be under candidate runtime/super-runs')
    out.mkdir(parents=True,exist_ok=True);path=out/'mission-state.json'
    script='compute_runtime.py' if args.kind=='compute' else 'mission.py'
    path=out/('compute-state.json' if args.kind=='compute' else 'mission-state.json')
    if args.kind=='mission' and not args.rpc_host:parser.error('Legacy mission requires --rpc-host')
    state=json.loads(path.read_text()) if path.exists() else {}
    if args.action=='start':
        if path.exists():raise RuntimeError('Use a fresh board evidence directory')
        env=os.environ.copy();env.update(SUPER_APP_ROOT=str(APP),SUPER_EVIDENCE_ROOT=str(out))
        if args.kind=='mission':env.update(AIRSIM_RPC_HOST=args.rpc_host,SUPER_EXECUTION_LOCATION='robopi-all',SUPER_CASE=args.case)
        with (out/('compute-runtime.log' if args.kind=='compute' else 'mission.log')).open('w') as stream:
            child=subprocess.Popen([sys.executable,str(APP/script)],env=env,stdout=stream,
                stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
        text=Path(f'/proc/{child.pid}/stat').read_text();fields=text[text.rfind(')')+2:].split()
        state={'pid':child.pid,'ticks':fields[19],'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
        path.write_text(json.dumps(state)+'\n')
    elif args.action=='stop' and alive(state,out,script):
        os.kill(state['pid'],signal.SIGINT)
    state['alive']=alive(state,out,script)
    print(json.dumps(state))


if __name__=='__main__':main()
