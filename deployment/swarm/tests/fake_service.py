#!/usr/bin/env python3
import signal
import time


stopping = False


def stop(_signum, _frame):
    global stopping
    stopping = True


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

while not stopping:
    time.sleep(0.05)
