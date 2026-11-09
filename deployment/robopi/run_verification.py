"""Transfer a board-local smoke checker and collect its scoped evidence."""
import os
from pathlib import Path
import shlex
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from robopi import board,hdc

component=sys.argv[1]
components=['as2','catora','swarm-formation'] if component=='all' else [component]
out=ROOT/'log/airsim/robopi-lib-acceptance'
out.mkdir(parents=True,exist_ok=True)
hdc('file','send',str(ROOT/'deployment/robopi/verify_lib.py'),'/data/aerohub/as2/verify_lib.py')
failed=[]
for name in components:
    directory='/data/aerohub/as2' if name=='as2' else '/data/aerohub/lib/'+name
    environment=directory+('/board_env.sh' if name=='as2' else '/env.sh')
    command=('. '+shlex.quote(environment)+' && export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && '
             'python3 /data/aerohub/as2/verify_lib.py '+shlex.quote(name))
    try:
        text=board(command,timeout=90)
        (out/(name+'.log')).write_text(text+'\n')
        print(text,flush=True)
    except Exception as error:
        (out/(name+'.log')).write_text(str(error)+'\n')
        print(error,flush=True)
        failed.append(name)
    # Evidence retrieval runs on failures as well; business success comes from result.json.
    board('cd '+shlex.quote(directory)+' && tar -czf runtime/verification.tar.gz runtime/verification')
    hdc('file','recv',directory+'/runtime/verification.tar.gz',str(out/(name+'.tar.gz')))
    import tarfile
    with tarfile.open(out/(name+'.tar.gz')) as archive:
        archive.extractall(out/name,filter='data')
    import json
    result=json.loads((out/name/'runtime/verification/result.json').read_text())
    if not result.get('passed') and name not in failed: failed.append(name)
if failed: raise SystemExit('Failed checks: '+', '.join(failed))
