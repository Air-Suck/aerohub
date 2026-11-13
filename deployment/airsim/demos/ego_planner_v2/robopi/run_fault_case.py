"""Explicit live single-board fault case: fresh session, basic flight, fault, cleanup, audit.

An unsuccessful motion step retains simulator/control resources for inspection.
Use only with authorization to fly the simulation and inject the configured fault.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    out=args.output.resolve()
    config=json.loads(args.config.read_text(encoding='utf-8-sig'))
    if not config.get('fault_case'):raise ValueError('Explicit fault_case required')
    config=run.prepare(out,config)
    snapshot=out/'client-sources';snapshot.mkdir()
    for source in [run.HERE/'run.py',run.HERE/'run_fault_case.py',run.HERE/'mission_client.py',run.HERE/'fault_mission.py',
                   run.PROJECT/'deployment/airsim/demos/ego_planner_v2/pc/flight_mission.py',run.PROJECT/'deployment/airsim/demos/ego_planner_v2/pc/ego_flight_mission.py']:
        shutil.copyfile(source,snapshot/source.name)
    run.write(out/'host-source-hashes.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()})
    def step(name,command):
        print(json.dumps(dict(step=name,event='start',wall=time.time(),run=config['run_id'])),flush=True)
        with (out/('host-'+name+'.log')).open('w',encoding='utf-8') as log:
            result=subprocess.run(list(map(str,command)),stdout=log,stderr=subprocess.STDOUT)
        run.write(out/('host-'+name+'-exit.json'),dict(returncode=result.returncode,wall=time.time()))
        print(json.dumps(dict(step=name,event='done',returncode=result.returncode,run=config['run_id'])),flush=True)
        if result.returncode:raise RuntimeError(name+' failed; inspect log, environment retained if not already stopped')
    def action(name,extra=()):
        step(name,[sys.executable,'-X','utf8','-B',run.HERE/'run.py',name,'--output',out,*extra])
    for name in ('deploy','pc-start','start'):action(name)
    deadline=time.monotonic()+90
    while True:
        status=run.board_action(config,'status')
        if status['ready']:break
        if status.get('status',{}).get('error') or time.monotonic()>deadline:
            run.write(out/'unready.json',status);raise RuntimeError('Board readiness failed')
        time.sleep(1)
    run.write(out/'ready-before-observer.json',status)
    step('observe',[sys.executable,'-X','utf8','-B',run.HERE/'observe_ground.py','--output',out,'--duration','40'])
    action('mission-basic')
    action('mission-fault',['--baseline',out/'mission-basic'])
    for name in ('stop','collect','pc-stop'):action(name)
    step('audit',['docker','run','--rm','--network','none','--memory','512m','--cpus','1','--pull','never',
        '-e','OPENBLAS_NUM_THREADS=1','-v',str(run.HERE)+':/input:ro','-v',str(out)+':/evidence',
        '--entrypoint','python3',run.ONBOARD,'/input/audit_fault.py','/evidence'])
    print((out/'fault-audit.json').read_text(),flush=True)

if __name__=='__main__':main()
