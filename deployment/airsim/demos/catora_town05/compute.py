#!/usr/bin/env python3
"""Own one UAV compute node and optional single centralized CAT-ORA service.

Requires a preloaded ROS/AS2/CAT-ORA environment. No arming, mission, or
simulator RPC is performed. A running supervisor is not capability readiness.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml


def commands(args):
    app = args.app.resolve()
    deployment = yaml.safe_load((app / 'deployment.yaml').read_text())
    fleet = yaml.safe_load((app / 'fleet.yaml').read_text())
    if args.vehicle not in deployment or args.vehicle not in fleet['vehicles']:
        raise ValueError('unknown compute vehicle')
    if args.central and not args.registry:
        raise ValueError('central service requires --registry')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'fleet.yaml').write_text(yaml.safe_dump({'vehicles': {
        args.vehicle: fleet['vehicles'][args.vehicle]}}))
    (args.output / 'input-deployment.yaml').write_text(yaml.safe_dump({
        args.vehicle: deployment[args.vehicle]}))
    environment = dict(os.environ)
    environment.pop('AS2_BEHAVIORS', None)
    environment.update(AS2_TIMED_POSITION_TOLERANCE='0.08',
                       AS2_FOLLOW_PATH_LAUNCH_FILE=str(args.app/'as2/motion_behaviors.launch.py') if (args.app/'as2/motion_behaviors.launch.py').exists() else str(Path(__file__).resolve().parents[4]/'src/behavior/follow_path/launch/motion_behaviors.launch.py'),
                       AS2_EXECUTION_BACKEND='follow_path', AS2_TF_SCOPE='per-vehicle',
        AS2_FLEET_FILE=str(args.output / 'fleet.yaml'),
        AS2_RUNTIME_DIR=str(args.output / 'as2'), AS2_MAX_RSS_MB='2048')
    (args.output / 'as2').mkdir()
    result = [('as2', ['python3', str(args.as2_launcher)]),
              ('state-adapter', ['ros2', 'run', 'uav_cat_ora_adapter', 'cat_ora_state_adapter_node',
                '--ros-args', '-r', '__node:=catora_state_' + args.vehicle,
                '-p', 'deployment_config:=' + str(args.output / 'input-deployment.yaml'),
                '-p', 'use_sim_time:=true'])]
    if args.central:
        if not args.registry:
            raise ValueError('central service requires --registry')
        result.extend([
            ('cat-ora', ['ros2', 'run', 'mrs_formation_reshaping', 'cat_ora_node',
                         '--ros-args', '-p', 'use_sim_time:=true']),
            ('proxy', ['ros2', 'run', 'mrs_formation_reshaping', 'ros2_proxy.py',
                       '--ros-args', '-p', 'use_sim_time:=true']),
            ('planner', ['ros2', 'run', 'uav_planner_framework', 'planner_framework_node',
                         '--ros-args', '-p', 'execution_backend:=follow_path',
                         '-p', 'runtime_startup_timeout:=30.0',
                         '-p', 'registry_root:=' + str(args.registry),
                         '-p', 'resolved_dir:=' + str(args.output / 'resolved'),
                         '-p', 'use_sim_time:=true'])])
    return environment, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--vehicle', required=True)
    parser.add_argument('--as2-launcher', type=Path, required=True)
    parser.add_argument('--central', action='store_true')
    parser.add_argument('--registry', type=Path)
    args = parser.parse_args()
    args.output = args.output.resolve()
    environment, launch = commands(args)
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    children = []
    try:
        for name, command in launch:
            stream = (args.output / (name + '.log')).open('w')
            child = subprocess.Popen(command, env=environment, stdout=stream,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            children.append((name, child, stream))
        (args.output / 'supervisor.json').write_text(json.dumps({
            'vehicle': args.vehicle, 'central': args.central, 'capability_ready': False,
            'role': 'compute_only', 'pid': os.getpid(),
            'children': {name: child.pid for name, child, _ in children}}, indent=2) + '\n')
        while not stopping:
            for name, child, _ in children:
                if child.poll() is not None:
                    raise RuntimeError(name + ' exited: ' + str(child.returncode))
            time.sleep(.2)
    finally:
        for _, child, _ in reversed(children):
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for _, child, stream in reversed(children):
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait(timeout=5)
            stream.close()
        (args.output / 'stopped.json').write_text(json.dumps({
            'children': {name: child.returncode for name, child, _ in children}}) + '\n')


if __name__ == '__main__':
    main()
