"""Manage only this private AS2 overlay's launcher on RoboPi."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
STATE = ROOT / 'runtime' / 'state.json'


def ticks(pid):
    text = Path(f'/proc/{pid}/stat').read_text()
    fields = text[text.rfind(')') + 2:].split()
    if fields[0] == 'Z':
        raise ProcessLookupError(pid)
    return fields[19]


def alive(state):
    try:
        return (state.get('boot_id') == Path('/proc/sys/kernel/random/boot_id').read_text().strip() and
                ticks(state['pid']) == state['ticks'] and
                str(ROOT / 'run_as2.py').encode() in
                Path(f"/proc/{state['pid']}/cmdline").read_bytes().split(b'\0'))
    except (OSError, KeyError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'status', 'stop'])
    args = parser.parse_args()
    STATE.parent.mkdir(exist_ok=True)
    state = json.loads(STATE.read_text()) if STATE.exists() else {}
    if args.action == 'start':
        if alive(state):
            raise RuntimeError('This overlay already owns a running AS2 launcher')
        with (STATE.parent / 'as2.log').open('w') as log:
            child = subprocess.Popen([sys.executable, str(ROOT / 'run_as2.py')],
                                     cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, start_new_session=True)
        state = dict(pid=child.pid, ticks=ticks(child.pid),
                     boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        STATE.write_text(json.dumps(state) + '\n')
        time.sleep(3)
        if child.poll() is not None:
            raise RuntimeError('AS2 exited; read runtime/as2.log')
    elif args.action == 'stop' and alive(state):
        os.kill(state['pid'], signal.SIGTERM)
        deadline = time.monotonic() + 15
        while alive(state) and time.monotonic() < deadline:
            time.sleep(0.2)
        if alive(state):
            raise RuntimeError('AS2 did not stop; inspect its launcher and children')
    state['alive'] = alive(state)
    print(json.dumps(state))
    return int(args.action == 'status' and not state['alive'])


if __name__ == '__main__':
    sys.exit(main())
