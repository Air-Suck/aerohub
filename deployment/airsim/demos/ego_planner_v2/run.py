"""PC build/lifecycle entry. Flight and RoboPi operations have explicit sub-entries.

Uses an existing UE scene. Does not download images, change settings or arm UAVs.
"""
import argparse
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
IMAGE='ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['build','pc-start','pc-status','pc-stop'])
    parser.add_argument('--workspace',type=Path,default=os.environ.get('EGO_PC_WORK'))
    parser.add_argument('--airsim-host',default=os.environ.get('EGO_AIRSIM_HOST'))
    args=parser.parse_args()
    if args.workspace is None:parser.error('--workspace or EGO_PC_WORK is required')
    work=args.workspace.expanduser().resolve()
    if work == ROOT or ROOT in work.parents:
        parser.error('Use an external build directory, separate from the repository')
    env=dict(os.environ,EGO_PC_WORK=str(work))
    if args.airsim_host:env['EGO_AIRSIM_HOST']=args.airsim_host
    def call(cmd,**kw):return subprocess.run(cmd,check=True,env=env,**kw)
    if args.action=='build':
        work.mkdir(parents=True,exist_ok=True)
        call(['docker','image','inspect',IMAGE],stdout=subprocess.DEVNULL)
        call(['docker','run','--rm','--pull','never','--network','none','--cpus','3','--memory','4g',
              '--name','aerohub-ego-pr1-build','-v',str(ROOT)+':/input:ro','-v',str(work)+':/work',
              '--entrypoint','bash',IMAGE,'/input/deployment/airsim/demos/ego_planner_v2/build_pc.sh'])
        return
    if not args.airsim_host:parser.error('--airsim-host or EGO_AIRSIM_HOST is required')
    if not (work/'source-manifest.json').is_file():parser.error('Build this external workspace first')
    compose=['docker','compose','-f',str(HERE/'pc/compose.yaml'),'--profile','as2-ground']
    if args.action=='pc-status':call(compose+['ps','-a']);return
    if args.action=='pc-start':
        old=call(['docker','ps','-aq','--filter','label=com.docker.compose.project=ego-migration-ground'],capture_output=True,text=True).stdout.strip()
        if old:raise RuntimeError('Existing EGO Compose containers; inspect pc-status instead of replacing them')
        call(compose+['up','-d','drone0','drone1','clock','as2']);return
    # Stop only after actual PX4 feedback; absence/unknown state is not disarmed.
    for i in (0,1):
        result=call(['docker','exec','-w','/var/lib/flight_board',f'ego-migration-ground-drone{i}-1',
                    '/opt/px4-gazebo/bin/px4-commander','--instance',str(i),'status'],capture_output=True,text=True)
        if 'Disarmed' not in result.stdout:raise RuntimeError('Both vehicles must be confirmed disarmed')
    active=call(['docker','ps','--format','{{.Names}}'],capture_output=True,text=True).stdout.splitlines()
    if any(n.startswith(('ego-compute-','ego-mission-')) for n in active):
        raise RuntimeError('Stop/collect the owned task and compute session first')
    call(compose+['down'])

if __name__=='__main__':main()
