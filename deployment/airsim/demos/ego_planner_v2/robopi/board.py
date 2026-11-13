"""Candidate-local session manager. Never arms or sends a motion goal."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from policy import run_id

ROOT = Path(__file__).resolve().parent


def identity(pid):
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    if fields[0] == 'Z':
        raise ProcessLookupError(pid)
    return dict(pid=pid, ticks=fields[19],
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def alive(state):
    try:
        return (identity(state['pid']) == state and
                str(ROOT / 'runtime.py').encode() in
                Path(f"/proc/{state['pid']}/cmdline").read_bytes().split(b'\0'))
    except (OSError, KeyError):
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['start', 'status', 'check', 'stop'])
    parser.add_argument('--run-id', required=True, type=run_id)
    args = parser.parse_args()
    config = json.loads((ROOT / 'config.json').read_text())
    candidate = Path(config['candidate'])
    out = candidate / 'runtime/ego-single' / args.run_id
    out.parent.mkdir(parents=True, exist_ok=True)
    with (out.parent / 'operation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        state_file = out / 'identity.json'
        state = json.loads(state_file.read_text()) if state_file.exists() else {}
        if args.action == 'start':
            if out.exists():
                raise RuntimeError('Run directory already exists; use a new session')
            manifest = json.loads((candidate / 'deployment.json').read_text())
            required = {'ego_planner', 'uav_ego_planner_adapter', 'as2_navigation_inputs',
                        'uav_follow_path_execution', 'as2_platform_pixhawk'}
            if manifest['prefix'] != str(candidate) or not required.issubset(manifest['packages']):
                raise RuntimeError('Wrong candidate deployment')
            for name, digest in manifest['files'].items():
                if hashlib.sha256((candidate / 'install' / name).read_bytes()).hexdigest() != digest:
                    raise RuntimeError('Candidate checksum mismatch: ' + name)
            domain = ('ROS_DOMAIN_ID=' + str(config['domain'])).encode()
            for proc in Path('/proc').glob('[0-9]*'):
                if int(proc.name) in (os.getpid(), os.getppid()):
                    continue
                try:
                    env = (proc / 'environ').read_bytes().split(b'\0')
                except OSError:
                    continue
                if domain in env:
                    raise RuntimeError('Board ROS domain already occupied: ' + proc.name)
            out.mkdir()
            (out / 'deployment.json').write_text(json.dumps(manifest))
            with (out / 'runtime.log').open('w') as log:
                child = subprocess.Popen([sys.executable, str(ROOT / 'runtime.py'), str(out)],
                    stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
            state = identity(child.pid)
            state_file.write_text(json.dumps(state))
        if args.action == 'stop' and alive(state):
            request = out / 'stop.request'
            request.write_text(str(time.time()))
            deadline = time.monotonic() + 25
            while alive(state) and time.monotonic() < deadline:
                # Runtime removes the request before destroying its ROS context.
                # A completion record means wait for that final exit, not fail early.
                if not request.exists() and not (out / 'stopped.json').exists():
                    break
                time.sleep(.2)
            if alive(state):
                raise RuntimeError('Stop not confirmed; retain simulator and control. Inspect status/logs')
        status = json.loads((out / 'status.json').read_text()) if (out / 'status.json').exists() else {}
        running = alive(state)
        fresh = time.time() - status.get('written_at', 0) < 2
        result = dict(identity=state, alive=running, status_fresh=fresh, status=status,
                      ready=running and fresh and status.get('ready', False))
        print(json.dumps(result))
        if args.action == 'check' and not result['ready']:
            return 1
        if args.action == 'stop' and not (out / 'stopped.json').exists():
            return 1
        return 0


if __name__ == '__main__':
    sys.exit(main())
