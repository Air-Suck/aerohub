#!/usr/bin/env python3
"""Check native scene truth against unarmed PX4 attitude and magnetic input.

Run with the matching ProjectAirSim Python client installed. This never sends
motion commands or writes PX4 parameters. A running SITL container is required.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import re
import subprocess


def yaw_deg(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def angle_error(a, b):
    return abs((a-b+180) % 360-180)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container', required=True)
    parser.add_argument('--instance', type=int, default=20)
    parser.add_argument('--scene', default='ScenePlannerCourse')
    parser.add_argument('--vehicle', default='Drone1')
    parser.add_argument('--topic-port', type=int, default=9089)
    parser.add_argument('--service-port', type=int, default=9090)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--yaw-limit-deg', type=float, default=1.)
    args = parser.parse_args()
    if args.samples < 20:
        parser.error('at least 20 samples required')
    from projectairsim import ProjectAirSimClient
    args.output.mkdir(parents=True, exist_ok=True)

    def px4(command, *arguments):
        result = subprocess.run(['docker', 'exec', args.container,
            '/opt/px4-gazebo/bin/px4-'+command, '--instance', str(args.instance),
            *arguments], text=True, capture_output=True, timeout=45, check=True)
        return result.stdout

    def capture(topic, count):
        text = px4('listener', topic, '-n', str(count), '-r', '2')
        (args.output/(topic+'.txt')).write_text(text)
        return text

    def parameter(name):
        return float(px4('param', 'show', '-q', name).strip())

    client = ProjectAirSimClient('127.0.0.1', args.topic_port, args.service_port)
    client.connect_services()
    def truth():
        return client.request({'method':f'/Sim/{args.scene}/robots/{args.vehicle}/GetGroundTruthKinematics',
                               'params':{}, 'version':1.0})
    try:
        before = truth()
        with ThreadPoolExecutor(max_workers=3) as pool:
            jobs = {topic: pool.submit(capture, topic, count) for topic, count in
                    [('vehicle_attitude', args.samples), ('sensor_mag', args.samples), ('vehicle_status', 1)]}
            texts = {topic: job.result() for topic, job in jobs.items()}
        after = truth()
        parameters = {name:parameter(name) for name in ['EKF2_DECL_TYPE','EKF2_MAG_DECL','EKF2_MAG_TYPE']}
        estimator = capture('estimator_status', 1)
    finally:
        client.disconnect()
    (args.output/'native-truth.json').write_text(json.dumps({'before':before,'after':after},indent=2)+'\n')
    quaternions = [[float(v) for v in group.split(',')] for group in
                   re.findall(r'\bq: \[([^\]]+)\]', texts['vehicle_attitude'])]
    orientation = before['pose']['orientation']
    truth_yaw = yaw_deg([orientation[k] for k in ['w','x','y','z']])
    errors = [angle_error(yaw_deg(q), truth_yaw) for q in quaternions]
    status = texts['vehicle_status']
    armed = re.search(r'\barming_state: (\d+)', status)
    magnetic = {axis:[float(v) for v in re.findall(r'^\s*'+axis+r': ([-+\d.eE]+)', texts['sensor_mag'], re.M)]
                for axis in ['x','y','z']}
    means = [sum(magnetic[k])/len(magnetic[k]) if magnetic[k] else None for k in ['x','y','z']]
    field = json.loads(args.reference.read_text())['field_ned_gauss']
    # The configured reference is in NED. At this course's fixed yaw=0 spawn,
    # body FRD matches NED; reject moving/rotated cases rather than assume it.
    stationary = all(math.sqrt(sum(s['twist']['linear'][k]**2 for k in ['x','y','z'])) < .05
                     and math.sqrt(sum(s['twist']['angular'][k]**2 for k in ['x','y','z'])) < .01
                     for s in [before,after])
    native_identity = all(abs(before['pose']['orientation'][k]) < 1e-4 for k in ['x','y','z'])
    tests = {'twenty_attitude_samples':len(errors)>=args.samples,
             'unarmed':bool(armed and int(armed.group(1))==1),
             'stationary_native_truth':stationary,
             'native_identity_attitude':native_identity,
             'yaw_within_limit':bool(errors and max(errors)<=args.yaw_limit_deg),
             'twenty_mag_samples':all(len(v)>=args.samples for v in magnetic.values()),
             'field_matches_config':all(v is not None and abs(v-ref)<.01 for v,ref in zip(means,field)),
             'default_px4_declination_mode':parameters['EKF2_DECL_TYPE']==3}
    report = {'success':all(tests.values()),'checks':tests,'native_yaw_deg':truth_yaw,
              'yaw_error_deg_max':max(errors) if errors else None,'yaw_errors_deg':errors,
              'magnetic_field_body_gauss_mean':means,'reference_ned_gauss':field,
              'px4_parameters':parameters,'motion_requested':False,
              'scope':'Stationary ground alignment; airborne map and obstacle avoidance untested'}
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['success'] else 1


if __name__=='__main__':
    raise SystemExit(main())
