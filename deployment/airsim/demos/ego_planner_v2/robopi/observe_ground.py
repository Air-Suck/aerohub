import os
"""Read-only workstation observer using the same fixed image and UDP mapping as the ground test."""
import argparse
import json
from pathlib import Path
import subprocess
import time
from run import HERE, ONBOARD, validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=int, default=90)
    parser.add_argument('--refresh', action='store_true', help='Preserve previous observations under a unique name before collecting fresh evidence')
    args = parser.parse_args()
    if not 30 <= args.duration <= 300:
        parser.error('duration must be 30..300 seconds')
    out = args.output.resolve()
    config = json.loads((out/'config.json').read_text())
    validate(config)
    if any((out/name).exists() for name in ('observer.log', 'observer.json')):
        if not args.refresh:
            raise RuntimeError('Observer evidence already exists; use --refresh to preserve and renew it')
        previous = out/('observation-'+str(time.time_ns()))
        previous.mkdir(exist_ok=False)
        for name in ('observer.log', 'observer.json'):
            if (out/name).exists():
                (out/name).rename(previous/name)
    if config.get('dds_transport') == 'udp_mapped':
        port = 7400+250*config['domain']+18
        network = ['-p', f"{config['pc_ip']}:{port}-{port+1}:{port}-{port+1}/udp"]
    else:
        network = ['--network', 'host']
    command = ['docker', 'run', '--rm', '--init', '--pull', 'never',
        '--name', 'ego-ground-observer-'+config['run_id'], *network,
        '-v', os.environ['EGO_PC_WORK']+':/work:ro', '-v', str(HERE)+':/entry:ro',
        '-v', str(out)+':/session', '-e', 'ROS_DOMAIN_ID='+str(config['domain']),
        '-e', 'ROS_LOCALHOST_ONLY=0', '-e', 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/session/pc-dds.xml']
    setup = 'set -e; '
    for prefix in ('/opt/ros/humble', '/opt/px4_ws/install', '/work/install/humble',
                   '/root/aerostack2_ws/install', '/work/ground_overlay/install', '/work/ego_runtime/install'):
        setup += 'source '+prefix+'/local_setup.bash; '
    command += ['--entrypoint', 'bash', ONBOARD, '-c', setup+
        'exec python3 /entry/ground_observer.py /session/observer.json '+str(args.duration)]
    with (out/'observer.log').open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=args.duration+30)
    if result.returncode:
        raise RuntimeError('Observer failed; inspect observer.log')
    print(out/'observer.json')


if __name__ == '__main__':
    main()
