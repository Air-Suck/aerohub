#!/usr/bin/env python3
"""Ground-only Town05/PX4 readiness probe; never arms or starts a mission."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
SCENE = ROOT / 'simulator/airsim/scenarios/catora_town05'
sys.path.insert(0, str(ROOT / 'deployment/airsim/demos/catora_open_field'))
from sim_startup import launch_vehicles

PREFIX = 'aerohub-catora-town05-ground-'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--image', default=os.environ.get('FLIGHT_BOARD_IMAGE', 'flight_board:v0.1.0'))
    parser.add_argument('--observe-seconds', type=float, default=30.)
    args = parser.parse_args()
    if not 1 <= args.observe_seconds <= 300:
        parser.error('observe-seconds must be between 1 and 300')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manager = [sys.executable, str(ROOT / 'simulator/airsim/manager/manager.py')]
    def scene(action):
        return manager + [action, '--profile', str(SCENE / 'profile.json'),
                          '--run-dir', str(output / 'scene'), '--timeout', '60']
    # A failed Docker query is not evidence that names are free.
    listed = subprocess.check_output(['docker', 'ps', '-a', '--format', '{{.Names}}'], text=True).splitlines()
    names = [PREFIX + f'vehicle{i}' for i in range(4)]
    if any(name in listed for name in names):
        raise RuntimeError('owned probe names already exist; inspect before reuse')
    image_id = subprocess.check_output(['docker', 'image', 'inspect', '--format', '{{.Id}}', args.image], text=True).strip()
    report = {'ground_only': True, 'arming_commands_sent': False, 'passed': False,
              'image': args.image, 'image_id': image_id, 'observe_seconds': args.observe_seconds}
    (output / 'source.patch').write_text(subprocess.check_output(['git', 'diff', 'HEAD'], cwd=ROOT, text=True))
    (output / 'settings.json').write_bytes((SCENE / 'settings.json').read_bytes())
    relays = []
    try:
        with (output / 'start.json').open('w') as stream:
            subprocess.run(scene('start'), check=True, stdout=stream)
        relays = launch_vehicles(args.image, SCENE, output, PREFIX,
                                ROOT / 'deployment/airsim/config/dds/fastdds_udp.xml')
        deadline = time.monotonic() + args.observe_seconds
        while time.monotonic() < deadline:
            subprocess.run(scene('status'), check=True, stdout=subprocess.DEVNULL)
            for name in names:
                state = subprocess.check_output(['docker', 'inspect', '--format', '{{.State.Running}}', name], text=True)
                if state.strip() != 'true':
                    raise RuntimeError(name + ' stopped during ground observation')
            time.sleep(.5)
        report['passed'] = True
    except BaseException as exc:
        report['error'] = str(exc)
        raise
    finally:
        cleanup = []
        for name in names:
            logs = subprocess.run(['docker', 'logs', name], capture_output=True)
            (output / (name + '.log')).write_bytes(logs.stdout + logs.stderr)
            stopped = subprocess.run(['docker', 'stop', '--timeout', '8', name], capture_output=True)
            cleanup.append({'container': name, 'stop_returncode': stopped.returncode})
        for relay in relays:
            relay.close()
        # manager owns all scene identities, including partially started scenes.
        stopped = subprocess.run(scene('stop'), capture_output=True)
        if stopped.returncode or any(item['stop_returncode'] for item in cleanup):
            report['passed'] = False
            report.setdefault('error', 'ground probe resource cleanup failed')
        report['cleanup'] = cleanup
        report['scene_stop_returncode'] = stopped.returncode
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    if not report['passed']:
        raise RuntimeError(report['error'])
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
