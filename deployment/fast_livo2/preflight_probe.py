#!/usr/bin/env python3
"""Require PX4 preflight readiness to remain true before starting the upper chain."""
import argparse
import json
import time

import rclpy
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--stable-seconds', type=float, default=2.0)
    args = parser.parse_args()
    if args.timeout <= 0 or not 0 < args.stable_seconds < args.timeout:
        parser.error('require 0 < stable-seconds < timeout')

    rclpy.init()
    node = Node('fast_livo2_px4_preflight_probe')
    samples = 0
    last_ready = False
    ready_since = None

    def receive(message):
        nonlocal samples, last_ready, ready_since
        samples += 1
        last_ready = bool(message.pre_flight_checks_pass)
        if last_ready:
            ready_since = ready_since or time.monotonic()
        else:
            ready_since = None

    subscription = node.create_subscription(
        VehicleStatus,
        '/drone0/fmu/out/vehicle_status_v4',
        receive,
        qos_profile_sensor_data)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if ready_since is not None and time.monotonic() - ready_since >= args.stable_seconds:
                print(json.dumps({
                    'ready': True,
                    'samples': samples,
                    'stable_seconds': time.monotonic() - ready_since,
                    'elapsed_seconds': time.monotonic() - started,
                }))
                return 0
        print(json.dumps({
            'ready': False,
            'samples': samples,
            'last_preflight_checks_pass': last_ready,
            'elapsed_seconds': time.monotonic() - started,
        }))
        return 1
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
