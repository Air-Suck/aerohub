#!/usr/bin/env python3
"""Forward upstream ProjectAirSim ground-truth speed to the observation HUD."""
import argparse
import json
import math
import time
from urllib.request import Request, urlopen

from projectairsim import ProjectAirSimClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', default='127.0.0.1')
    parser.add_argument('--service-port', type=int, default=8990)
    parser.add_argument('--topic-port', type=int, default=8989)
    parser.add_argument('--display-port', type=int, default=8991)
    parser.add_argument('--scene', required=True, help='Resolved scene JSON emitted by manager')
    parser.add_argument('--vehicle', default='Drone1')
    args = parser.parse_args()
    with open(args.scene) as stream:
        scene = json.load(stream)
    client = ProjectAirSimClient(args.address, args.topic_port, args.service_port)
    client.connect_services()
    try:
        while True:
            state = client.request({'method': f"/Sim/{scene['id']}/robots/{args.vehicle}/GetGroundTruthKinematics",
                                    'params': {}, 'version': 1.0})
            velocity = state['twist']['linear']
            speed = math.sqrt(sum(velocity[axis] ** 2 for axis in ('x', 'y', 'z')))
            payload = json.dumps({'vehicle': args.vehicle, 'speed_mps': speed}).encode()
            request = Request(f'http://127.0.0.1:{args.display_port}/aerohub/telemetry',
                              data=payload, headers={'Content-Type': 'application/json'})
            with urlopen(request, timeout=2) as response:
                response.read()
            time.sleep(.05)
    finally:
        client.disconnect()


if __name__ == '__main__':
    main()
