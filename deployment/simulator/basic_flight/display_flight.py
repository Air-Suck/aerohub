#!/usr/bin/env python3
"""Observe the existing native AS2 flight with a six-second stop before landing."""
import argparse
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen

import rclpy
from mission import Mission


class DisplayFlight(Mission):
    def __init__(self, output, port):
        super().__init__()
        self.output = output
        self.port = port
        self.phase = 'ready'
        self.next_sample = 0
        self.samples = (output / 'samples.jsonl').open('w')

    def display(self, event):
        path = 'status' if event == 'status' else 'mission/' + event
        req = Request(f'http://127.0.0.1:{self.port}/aerohub/{path}',
                      data=None if event == 'status' else b'')
        with urlopen(req, timeout=2) as response:
            return json.load(response)

    def wait(self, predicate, timeout=30):
        def observed():
            now = time.monotonic()
            if now >= self.next_sample:
                self.next_sample = now + .2
                sample = {'wall_monotonic': now, 'phase': self.phase,
                          'display': self.display('status')}
                if self.pose:
                    p = self.pose.pose.position
                    stamp = self.pose.header.stamp
                    sample['as2_pose'] = {'xyz': [p.x, p.y, p.z],
                                         'stamp': stamp.sec + stamp.nanosec * 1e-9}
                if self.platform:
                    sample['armed'] = self.platform.armed
                self.samples.write(json.dumps(sample) + '\n')
                self.samples.flush()
            return predicate()
        return super().wait(observed, timeout)

    def action(self, action_type, name, goal, result_field, timeout=90):
        self.phase = name.rsplit('/', 1)[-1]
        super().action(action_type, name, goal, result_field, timeout)
        if name.endswith('GoToBehavior'):
            self.phase = 'stopped_hold'
            end = time.monotonic() + 6
            self.wait(lambda: time.monotonic() >= end, 10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--display-port', type=int, default=8991)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = DisplayFlight(args.output, args.display_port)
    result = {'success': False}
    try:
        node.display('reset')
        node.display('start')
        result = node.run()
        node.display('finish')
        result['display_final'] = node.display('status')
    except Exception as exc:
        result['error'] = str(exc)
        result['recovery'] = node.recover()
    finally:
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        node.samples.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
