#!/usr/bin/env python3
"""Preview the CAT-ORA Town05 site without starting flight or algorithm nodes."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]
SCENE = ROOT / 'simulator/airsim/scenarios/catora_town05'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check', 'start', 'status', 'stop'))
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'log/airsim/catora-town05-preview')
    args = parser.parse_args()
    return subprocess.run([
        sys.executable, str(ROOT / 'simulator/airsim/manager/manager.py'), args.action,
        '--profile', str(SCENE / 'profile.json'),
        '--run-dir', str(args.output.resolve()), '--timeout', '60',
    ]).returncode


if __name__ == '__main__':
    raise SystemExit(main())
