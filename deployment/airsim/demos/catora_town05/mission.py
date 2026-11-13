#!/usr/bin/env python3
"""Workstation-only client for predeployed CAT-ORA and four AS2 executors.

The compute host must already have the stage task YAMLs and a calibrated
fleet deployment YAML. No algorithms, adapters, or AS2 processes start here.
"""
import argparse
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
APP = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'simulator/airsim/sensors'))
from uav_as2_sensor_adapter import rpc as airsim_rpc
sys.modules['airsim_rpc'] = airsim_rpc
spec = importlib.util.spec_from_file_location('catora_verified_mission',
    ROOT / 'deployment/airsim/demos/catora_open_field/mission.py')
verified = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verified)
verified.Rpc = airsim_rpc.PersistentRpc

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from as2_msgs.action import Takeoff, Land
from as2_msgs.msg import ControlMode
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.msg import BehaviorResult
from uav_planner_interfaces.srv import ControlPlanner
import yaml


class Town05Mission(verified.Mission):
    def __init__(self, args):
        os.environ.update(CATORA_APP_ROOT=str(APP), CATORA_OUTPUT=str(args.output),
                          CATORA_EXECUTION_BACKEND='follow_path', CATORA_MIXED='0',
                          CATORA_NODE_NAME='catora_town05_workstation')
        super().__init__()
        self.args = args
        self.sequence = yaml.safe_load((APP / 'sequence.yaml').read_text())
        self.stages = []
        self.assignments = {}
        self.subscriptions.append(self.node.create_subscription(String, '/planner/goal_assignment',
            lambda message:self.assignments.__setitem__(json.loads(message.data)['task_id'],json.loads(message.data)),
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.flight_started = False
        self.interrupted = False
        self.controller = self.node.create_client(ControlPlanner, '/planner/control')
        self.clients.append(self.controller)
        self.actions = []
        for vehicle in self.vehicles:
            for kind, suffix in ((Takeoff, 'TakeoffBehavior'), (Land, 'LandBehavior'),
                                 (FollowTimedPath, 'follow_timed_path')):
                self.actions.append(ActionClient(self.node, kind, f'/{vehicle}/{suffix}'))
        self.clients.extend(self.actions)

    def close(self):
        self.rpc.close()
        super().close()

    def start(self, *_):
        raise RuntimeError('Workstation must not start compute services')

    def screenshot(self, name):
        response = self.rpc.call('simGetImages', [{'camera_name': 'swarm_observer',
            'image_type': 0, 'pixels_as_float': False, 'compress': True}], '', True)[0]
        (self.output / (name + '.png')).write_bytes(bytes(response['image_data_uint8']))

    def wait(self, predicate, timeout=45):
        def checked():
            if self.interrupted:
                raise InterruptedError('mission stop requested')
            if self.flight_started and not self.phase.startswith('emergency'):
                now = self.node.get_clock().now().nanoseconds / 1e9
                for vehicle, pose in self.local_poses.items():
                    stamp = pose.header.stamp.sec + pose.header.stamp.nanosec / 1e9
                    if not -.1 <= now - stamp <= .5:
                        raise RuntimeError('stale/future local pose: ' + vehicle)
            return predicate()
        return super().wait(checked, timeout)

    def record(self):
        original = self.phase
        if self.flight_started and original in ('planning', 'holding_goal'):
            self.phase = 'planning'
        try:
            super().record()
            if self.flight_started and not original.startswith('emergency') and min(self.min_separation, self.min_truth_separation) < 2.:
                raise RuntimeError('observed separation below 2m')
        finally:
            self.phase = original

    def hold_stage(self, stage):
        self.phase = 'holding_goal'
        started = None
        deadline = time.monotonic() + 20
        while True:
            self.wait(lambda: True)
            self.executor.spin_once(timeout_sec=.05)
            self.collision_check()
            positions = self.fleet_positions()
            truth = self.truth_positions()
            errors = {v: math.dist(positions[v], [stage['targets'][v][a] for a in ('x', 'y', 'z')]) for v in self.vehicles}
            truth_errors = {v: math.dist(truth[v], [stage['targets'][v][a] for a in ('x', 'y', 'z')]) for v in self.vehicles}
            with (self.output/'hold-errors.jsonl').open('a') as stream:
                stream.write(json.dumps({'stage':stage['name'],'sim_time':self.node.get_clock().now().nanoseconds/1e9,'state_errors_m':errors,'truth_errors_m':truth_errors})+'\n')
            within = max(errors.values()) <= .6 and max(truth_errors.values()) <= .75
            now = self.node.get_clock().now().nanoseconds / 1e9
            if not within:
                started = None
            elif started is None:
                started = now
            elif now - started >= stage['hold_duration_s']:
                self.stages.append({'name': stage['name'], 'task_id': self.task_id,
                    'endpoint_error_m': errors, 'truth_error_m': truth_errors,
                    'held_sim_seconds': now - started,
                    'goal_assignment':stage.get('goal_assignment'),
                    'requested_slots':stage.get('requested_slots',stage['targets'])})
                (self.output / 'stages.json').write_text(json.dumps(self.stages, indent=2) + '\n')
                self.screenshot(stage['name'])
                return
            if time.monotonic() >= deadline:
                raise TimeoutError('formation hold failed: ' + stage['name'])

    def run(self):
        self.phase = 'ground_preflight'
        self.wait(lambda: len(self.local_poses) == len(self.modes) == len(self.platforms) == 4
                  and self.node.get_clock().now().nanoseconds > 0
                  and self.controller.service_is_ready()
                  and all(a.server_is_ready() for a in self.actions), 90)
        if any(m.flag_armed for m in self.modes.values()):
            raise RuntimeError('all four platforms must initially be disarmed')
        settled = time.monotonic()
        self.wait(lambda: time.monotonic()-settled >= 15., 20)
        if any(m.flag_armed for m in self.modes.values()):
            raise RuntimeError('ground registration requires disarmed aircraft')
        self.screenshot('ground')
        if self.args.calibrate_ground:
            if not self.args.preflight_only:
                raise RuntimeError('calibration is a separate disarmed deployment step')
            self.align_origins()
            self.args.local_deployment = self.output/'deployment.yaml'
        # The compute-side calibration must match this local copy. Do not silently
        # recalibrate only the workstation while the Planner uses another frame.
        calibrated = yaml.safe_load(self.args.local_deployment.read_text())
        if set(calibrated) != set(self.vehicles):
            raise RuntimeError('calibrated deployment fleet mismatch')
        self.deployment = calibrated
        for vehicle, actual in self.truth_positions().items():
            if math.dist(actual, self.fleet_positions()[vehicle]) > .75:
                raise RuntimeError('compute deployment frame does not match ground truth: ' + vehicle)
        (self.output / 'deployment.yaml').write_text(yaml.safe_dump(self.deployment, sort_keys=False))
        for index, vehicle in enumerate(self.vehicles):
            self.collision_baseline[vehicle] = self.rpc.call('simGetCollisionInfo', f'Drone{index}')['time_stamp']
        if self.args.preflight_only:
            started = time.monotonic()
            self.wait(lambda: time.monotonic()-started >= self.args.observe_seconds, self.args.observe_seconds+5)
            self.screenshot('ground-observed')
            (self.output / 'preflight.json').write_text(json.dumps({'passed': True, 'armed': False,
                'actions_ready': 12, 'planner_ready': True, 'compute_processes_started': 0}) + '\n')
            return
        self.flight_started = True
        (self.output/'arming-intent.json').write_text(json.dumps({'time':time.time(),'armed_state_unknown_until_feedback':True})+'\n')
        self.phase = 'takeoff'
        for vehicle in self.vehicles:
            self.service(vehicle, 'set_arming_state', True, 400, lambda v=vehicle:self.modes[v].flag_armed)
            self.service(vehicle, 'set_offboard_mode', True, 176, lambda v=vehicle:self.modes[v].flag_control_offboard_enabled)
            self.action(vehicle, Takeoff, 'TakeoffBehavior',
                        Takeoff.Goal(takeoff_height=3., takeoff_speed=.6), 'takeoff_success')
        for index, vehicle in enumerate(self.vehicles):
            self.collision_baseline[vehicle] = self.rpc.call('simGetCollisionInfo', f'Drone{index}')['time_stamp']
        self.hold_stage(self.sequence['stages'][0])
        for stage in self.sequence['stages'][1:]:
            self.phase = 'planning'
            self.task_id = None
            self.status = None
            self.bridge_state = {}
            self.trajectory = None
            self.reference_counts = {v: 0 for v in self.vehicles}
            request = ControlPlanner.Request(command=ControlPlanner.Request.START,
                task_config_path=str(self.args.compute_root / stage['task_config']),
                deployment_config_path=str(self.args.compute_deployment))
            response = self.planner_call(self.controller, request)
            # Never retry ambiguous START: the compute service may already be flying.
            if response is None or not response.accepted:
                raise RuntimeError('Planner START ambiguous/rejected; stop before retry')
            self.task_id = response.result.task_id
            self.status = response.result
            self.wait(lambda:self.status.overall_status in
                (BehaviorResult.SUCCEEDED, BehaviorResult.FAILED, BehaviorResult.CANCELED), 120)
            if self.status.overall_status != BehaviorResult.SUCCEEDED:
                raise RuntimeError('Planner failed: ' + self.status.failure_reason)
            self.wait_as2_stop()
            if not all(self.reference_counts.values()):
                raise RuntimeError('missing per-vehicle AS2 reference evidence')
            self.wait(lambda:self.task_id in self.assignments,5)
            audit=self.assignments[self.task_id]
            if audit.get('interface_version')!=1 or audit.get('frame')!='LOCAL_NED':
                raise RuntimeError('invalid goal assignment metadata')
            bindings=audit['bindings']
            if set(bindings)!=set(self.vehicles) or {item['slot_id'] for item in bindings.values()}!=set(stage['targets']):
                raise RuntimeError('goal assignment is not one-to-one')
            for item in bindings.values():
                requested=stage['targets'][item['slot_id']]
                if math.dist([item['target'][a] for a in ('x','y','z')],[requested[a] for a in ('x','y','z')])>.001:
                    raise RuntimeError('goal assignment changed a requested slot')
            actual_stage=copy.deepcopy(stage)
            actual_stage['requested_slots']=stage['targets']
            actual_stage['targets']={v:item['target'] for v,item in bindings.items()}
            actual_stage['goal_assignment']=audit
            self.hold_stage(actual_stage)
        if min(self.min_separation, self.min_truth_separation) < 2:
            raise RuntimeError('observed separation below 2m')
        self.phase = 'landing'
        for vehicle in self.vehicles:
            self.action(vehicle, Land, 'LandBehavior', Land.Goal(land_speed=.5), 'land_success')
            self.wait(lambda v=vehicle:not self.modes[v].flag_armed)
        self.flight_started = False
        (self.output / 'result.json').write_text(json.dumps({'passed': True,
            'landed_and_disarmed': True, 'stages': self.stages,
            'min_separation_m': self.min_separation, 'min_truth_separation_m': self.min_truth_separation,
            'workstation_compute_processes_started': 0}, indent=2) + '\n')

    def recover(self):
        self.interrupted = False
        self.phase = 'emergency_stop'
        report = {'landed_and_disarmed': False, 'errors': []}
        try:
            result = self.planner_call(self.controller, ControlPlanner.Request(command=ControlPlanner.Request.CANCEL), 5)
            report['cancel_accepted'] = result is not None and result.accepted
        except Exception as exc:
            report['errors'].append('cancel: ' + str(exc))
        try:
            self.controller_mode(ControlMode.HOVER)
        except Exception as exc:
            report['errors'].append('hover: ' + str(exc))
        self.phase = 'emergency_landing'
        # Cancellation/HOVER errors must not skip independent Land attempts.
        for vehicle in self.vehicles:
            if vehicle not in self.modes or not self.modes[vehicle].flag_armed:
                continue
            try:
                self.action(vehicle, Land, 'LandBehavior', Land.Goal(land_speed=.5), 'land_success')
                self.wait(lambda v=vehicle:not self.modes[v].flag_armed, 45)
            except Exception as exc:
                report['errors'].append(vehicle + ': ' + str(exc))
        report['landed_and_disarmed'] = all(v in self.modes and not self.modes[v].flag_armed for v in self.vehicles)
        (self.output / 'failure-stop.json').write_text(json.dumps(report, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compute-root', type=Path, required=True)
    parser.add_argument('--compute-deployment', type=Path, required=True)
    parser.add_argument('--local-deployment', type=Path, required=True)
    parser.add_argument('--preflight-only', action='store_true')
    parser.add_argument('--calibrate-ground', action='store_true')
    parser.add_argument('--observe-seconds', type=float, default=30.)
    args = parser.parse_args()
    if not 0 <= args.observe_seconds <= 300:
        parser.error('observe-seconds must be between 0 and 300')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    mission = Town05Mission(args)
    signal.signal(signal.SIGTERM, lambda *_:setattr(mission, 'interrupted', True))
    signal.signal(signal.SIGINT, lambda *_:setattr(mission, 'interrupted', True))
    try:
        mission.run()
    except BaseException as exc:
        (args.output / 'failure.json').write_text(json.dumps({'phase': mission.phase, 'error': str(exc)}) + '\n')
        if any(m.flag_armed for m in mission.modes.values()):
            mission.recover()
        raise
    finally:
        mission.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
