#!/usr/bin/env python3
"""Check real CAT-ORA plans on configured formation endpoints, without flight.

This fixed-input algorithm probe is not a sensor, execution, or flight test.
"""
import argparse
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import signal
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/"src/planner/framework"))
from uav_planner_framework.goal_assignment import match_goal_slots

import rclpy
from geometry_msgs.msg import Point
from mrs_formation_reshaping.srv import SrvGetReshapingTrajectories as Service
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    app = Path(__file__).resolve().parent
    sequence = yaml.safe_load((app / 'sequence.yaml').read_text())
    stages = sequence['stages']
    report = {'scope': 'real_algorithm_fixed_inputs_only', 'passed': False, 'stages': []}
    log = (args.output / 'algorithm.log').open('w')
    process = subprocess.Popen(['ros2', 'run', 'mrs_formation_reshaping', 'cat_ora_node'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('town05_verify_configured_plans')
    try:
        client = node.create_client(Service, '/mrs_formation_reshaping/get_reshaping_trajectories')
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError('CAT-ORA native service unavailable')
        current = stages[0]["targets"]
        for stage in stages[1:]:
            def enu(target):
                return [float(target['y']), float(target['x']), -float(target['z'])]
            vehicles = list(current)
            starts = [enu(t) for t in current.values()]
            goals = [enu(t) for t in stage['targets'].values()]
            request = Service.Request()
            request.initial_configurations = [Point(x=x, y=y, z=z) for x, y, z in starts]
            request.goal_configurations = [Point(x=x, y=y, z=z) for x, y, z in goals]
            request.trajectory_dt = .05
            request.max_velocity = .8
            request.max_acceleration = .5
            request.publish_visualization = False
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=25)
            if not future.done() or future.exception():
                raise RuntimeError('native planning timeout/error')
            response = future.result()
            if not response.success or len(response.trajectories) != 4:
                raise RuntimeError('native planning failed: ' + response.message)
            trajectories = [[[p.x, p.y, p.z] for p in t.waypoints] for t in response.trajectories]
            if not all(trajectories) or len({len(t) for t in trajectories}) != 1:
                raise RuntimeError('empty or unequal trajectory sample counts')
            binding = match_goal_slots(dict(zip(vehicles,[t[-1] for t in trajectories])),
                {key:enu(t) for key,t in stage['targets'].items()},policy='optimize')
            errors = [math.dist(t[-1],enu(stage['targets'][binding[v]])) for v,t in zip(vehicles,trajectories)]
            current = {v:{'x':t[-1][1],'y':t[-1][0],'z':-t[-1][2]} for v,t in zip(vehicles,trajectories)}
            start_errors = [math.dist(t[0], start) for t, start in zip(trajectories, starts)]
            separation = min(math.dist(a, b) for points in zip(*trajectories)
                             for a, b in itertools.combinations(points, 2))
            speed = max(math.dist(a, b)/.05 for t in trajectories for a, b in zip(t, t[1:]))
            acceleration = max(math.sqrt(sum((c[k]-2*b[k]+a[k])**2 for k in range(3)))/.05**2
                               for t in trajectories for a,b,c in zip(t,t[1:],t[2:]))
            finite = all(math.isfinite(v) for t in trajectories for p in t for v in p)
            inside = all(-9.00001 <= p[0] <= 9.00001 and -59.00001 <= p[1] <= -40.99999
                         and abs(p[2]-3.) <= .00001 for t in trajectories for p in t)
            passed = finite and inside and acceleration <= .50001 and max(errors) <= .001 and max(start_errors) <= .001 and separation >= 2 and speed <= .80001
            result = {'name': stage['name'], 'passed': passed, 'endpoint_errors_m': errors, 'vehicle_to_slot':binding,
                      'start_errors_m': start_errors, 'minimum_sampled_separation_m': separation,
                      'maximum_sampled_speed_m_s': speed, 'maximum_sampled_acceleration_m_s2': acceleration,
                      'all_points_finite_and_in_envelope': finite and inside, 'sample_count': len(trajectories[0])}
            report['stages'].append(result)
            (args.output / (stage['name'] + '-trajectories.json')).write_text(json.dumps(trajectories) + '\n')
            if not passed:
                raise RuntimeError('configured formation rejected: ' + json.dumps(result))
        report['passed'] = True
    except BaseException as exc:
        report['error'] = str(exc)
        raise
    finally:
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node()
        rclpy.try_shutdown()
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        except ProcessLookupError:
            pass
        log.close()
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
