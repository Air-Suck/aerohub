"""Owned first-packet guard for the isolated control test."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'catora_open_field'))
from sim_startup import StartupRelay

def main():
    p = argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--listen-port',type=int,default=5583)
    p.add_argument('--upstream-port',type=int,default=5483)
    args = p.parse_args()
    relay = StartupRelay(('127.0.0.1', args.listen_port), ('127.0.0.1', args.upstream_port))
    (args.output/'relay.json').write_text(json.dumps({'pid':os.getpid()})+'\n')
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    try:
        while not stopping: time.sleep(.1)
    finally: relay.close()

if __name__ == '__main__': main()
