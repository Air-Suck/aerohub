"""Owned standalone relay; survives coordinator exit until explicit cleanup."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

APP=Path(__file__).resolve().parent
sys.path.insert(0,str(APP.parent/'catora_open_field'))
from sim_startup import StartupRelay


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--listen-host',default='127.0.0.1')
    args=parser.parse_args();out=args.output
    relay=StartupRelay((args.listen_host,4981),('127.0.0.1',4881))
    text=Path('/proc/self/stat').read_text();ticks=text[text.rfind(')')+2:].split()[19]
    state={'pid':os.getpid(),'ticks':ticks,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
    (out/'relay-state.json').write_text(json.dumps(state)+'\n')
    stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        while not stopping:time.sleep(.2)
    finally:relay.close()


if __name__=='__main__':main()
