#!/usr/bin/env python3
"""Forward Gazebo simulation time to the ROS 2 runtime over host-loopback UDP."""

import os
import pty
import socket
import struct
import subprocess
import sys

sys.path.insert(0, "/integration/src/planner_framework")
from uav_planner_framework.clock_parser import ClockTextParser


def main() -> int:
    destinations = [("127.0.0.1", 28502), ("127.0.0.1", 28503)]
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    master, slave = pty.openpty()
    process = subprocess.Popen(
        ["gz", "topic", "-e", "-t", "/clock"],
        stdout=slave,
        stderr=slave,
        close_fds=True,
    )
    os.close(slave)
    stream = os.fdopen(master, "r", buffering=1)
    parser = ClockTextParser()
    try:
        for line in stream:
            value = parser.feed(line)
            if value is not None:
                for destination in destinations:
                    sender.sendto(struct.pack("!qI", *value), destination)
    finally:
        stream.close()
        process.terminate()
        process.wait(timeout=2.0)
        sender.close()
    return process.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
