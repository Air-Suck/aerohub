"""ROS launch prefix: allocate one TCP listener per board process, then exec it."""
import fcntl
import json
import os
from pathlib import Path
import socket
import sys
from dds import tcp_profile


def main():
    root = Path(__file__).resolve().parent
    config = json.loads((root/'config.json').read_text())
    out = Path(os.environ['EGO_GROUND_STATE'])/'dds'
    out.mkdir(exist_ok=True)
    with (out/'allocation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        index = out/'assignments.json'
        assigned = json.loads(index.read_text()) if index.exists() else []
        base = config['tcp_base_port']
        port = base+1+len(assigned)
        if port >= base+24:
            raise RuntimeError('Board TCP participant pool exhausted')
        with socket.socket() as check:
            check.bind((config['board_ip'], port))
        profile = out/(str(port)+'.xml')
        tcp_profile(profile, config['board_ip'], base, listen_port=port, domain=config['domain'])
        assigned.append(dict(pid=os.getpid(), port=port, command=sys.argv[1:]))
        index.write_text(json.dumps(assigned, indent=2))
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
    os.execvp(sys.argv[1], sys.argv[1:])


if __name__ == '__main__':
    main()
