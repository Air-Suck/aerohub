#!/usr/bin/env python3
"""Bounded state-chain diagnostic; optional Offboard request, never arms/takes off."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleOdometry, VehicleControlMode, VehicleStatus
from as2_msgs.msg import PlatformInfo
from std_srvs.srv import SetBool
from std_msgs.msg import String


class StateProbe(Node):
    def __init__(self, args):
        super().__init__('catora_state_chain_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.stream = (args.output/'trace.jsonl').open('w')
        self.counts = defaultdict(int)
        self.unique = defaultdict(set)
        self.last = {}
        self.gaps = defaultdict(float)
        self.ages = defaultdict(list)
        self.modes = {}
        self.platforms = {}
        self.subscriptions_owned = []
        self.events = []
        self.initial = time.monotonic()
        self.pose_violations = []
        self.offset_client = None
        self.offset_future = None
        self.restore_future = None
        self.test_requested = False
        self.restored = False
        self.create_subscription(Clock, '/clock', lambda m:self.observe('/clock', m), qos_profile_sensor_data)
        for vehicle in args.vehicles:
            topics = [(VehicleOdometry, f'/{vehicle}/fmu/out/vehicle_odometry'),
                      (VehicleControlMode, f'/{vehicle}/fmu/out/vehicle_control_mode'),
                      (Odometry, f'/{vehicle}/sensor_measurements/odom'),
                      (PoseStamped, f'/{vehicle}/self_localization/pose'),
                      (PoseStamped, f'/{vehicle}/catora/local_pose'),
                      (PlatformInfo, f'/{vehicle}/platform/info')]
            for kind, topic in topics:
                self.subscriptions_owned.append(self.create_subscription(kind, topic,
                    lambda m,t=topic:self.observe(t,m), qos_profile_sensor_data))
        self.create_subscription(String, '/planner/catora_input_state',
                                 lambda m:self.event('adapter_diagnostics', data=json.loads(m.data)), 10)
        if args.offboard_vehicle:
            self.offset_client = self.create_client(SetBool, f'/{args.offboard_vehicle}/set_offboard_mode')

    def event(self, kind, **values):
        row = {'kind':kind, 'elapsed_s':time.monotonic()-self.initial,
               'wall_ns':time.time_ns(), 'sim_ns':self.get_clock().now().nanoseconds, **values}
        self.stream.write(json.dumps(row)+'\n')
        self.events.append(row)

    def observe(self, topic, message):
        receipt = time.monotonic()
        sim = self.get_clock().now().nanoseconds
        previous = self.last.get(topic)
        if previous:
            self.gaps[topic] = max(self.gaps[topic], receipt-previous['receipt'])
        stamp = None
        if hasattr(message, 'header'):
            stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        elif hasattr(message, 'clock'):
            stamp = message.clock.sec*10**9+message.clock.nanosec
        elif hasattr(message, 'timestamp'):
            stamp = int(message.timestamp)*1000
        self.counts[topic] += 1
        self.unique[topic].add(stamp)
        self.last[topic] = {'stamp':stamp, 'receipt':receipt}
        row = {'kind':'sample','topic':topic,'elapsed_s':receipt-self.initial,
               'wall_ns':time.time_ns(),'sim_ns':sim,'stamp_ns':stamp}
        for field in ('timestamp_sample','flag_armed','flag_control_offboard_enabled','armed','offboard'):
            if hasattr(message,field):row[field] = getattr(message,field)
        if hasattr(message,'header'):row['frame_id'] = message.header.frame_id
        if isinstance(message, VehicleControlMode):self.modes[topic.split('/')[1]] = message
        if isinstance(message, PlatformInfo):
            self.platforms[topic.split('/')[1]] = message
            row['platform_state'] = message.status.state
        if hasattr(message,'header') and stamp:
            row['age_s'] = (sim-stamp)/1e9
            self.ages[topic].append(row['age_s'])
        self.stream.write(json.dumps(row)+'\n')

    def check_pose_age(self):
        if time.monotonic()-self.initial < self.args.warmup:
            return
        now = self.get_clock().now().nanoseconds
        for vehicle in self.args.vehicles:
            topic=f'/{vehicle}/catora/local_pose'
            last=self.last.get(topic)
            age=None if last is None else (now-last['stamp'])/1e9
            if age is None or not -.1 <= age <= .5:
                self.pose_violations.append({'vehicle':vehicle,'age_s':age,
                                            'elapsed_s':time.monotonic()-self.initial})

    def test_offboard(self):
        if not self.offset_client:
            return
        elapsed=time.monotonic()-self.initial
        if elapsed >= self.args.warmup+3 and not self.test_requested:
            if len(self.modes)!=len(self.args.vehicles) or any(m.flag_armed for m in self.modes.values()):
                raise RuntimeError('offboard-only probe requires all observed aircraft disarmed')
            if not self.offset_client.service_is_ready():
                raise RuntimeError('Offboard service unavailable')
            self.event('offboard_request', vehicle=self.args.offboard_vehicle, enabled=True)
            self.offset_future=self.offset_client.call_async(SetBool.Request(data=True))
            self.test_requested=True
        if self.offset_future is not None and self.offset_future.done():
            self.event('offboard_response', enabled=True, success=self.offset_future.result().success)
            self.offset_future=None
        if self.test_requested and elapsed >= self.args.warmup+8 and not self.restored:
            self.event('offboard_request', vehicle=self.args.offboard_vehicle, enabled=False)
            self.restore_future=self.offset_client.call_async(SetBool.Request(data=False))
            self.restored=True
        if self.restore_future is not None and self.restore_future.done():
            self.event('offboard_response', enabled=False, success=self.restore_future.result().success)
            self.restore_future=None
        if any(m.flag_armed for m in self.modes.values()):
            raise RuntimeError('unexpected armed feedback; probe never sends arm')

    def report(self):
        topics={}
        for topic,count in self.counts.items():
            values=self.ages[topic]
            topics[topic]={'count':count,'unique_stamps':len(self.unique[topic]),
                'max_receipt_gap_s':self.gaps[topic],
                'age_min_s':min(values,default=None),'age_max_s':max(values,default=None),
                'publishers':[{'node':p.node_name,'namespace':p.node_namespace,'type':p.topic_type}
                              for p in self.get_publishers_info_by_topic(topic)]}
        passed=not self.pose_violations and all(
            self.counts[f'/{v}/catora/local_pose']>5 for v in self.args.vehicles)
        return {'passed':passed,'ground_only':True,'arming_commands_sent':False,
                'offboard_vehicle':self.args.offboard_vehicle,'topics':topics,
                'message_fields':{k.__name__:k.get_fields_and_field_types() for k in (VehicleOdometry,VehicleControlMode,VehicleStatus)},
                'pose_age_violations':self.pose_violations,'events':self.events,
                'armed_feedback':{v:m.flag_armed for v,m in self.modes.items()}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--vehicles',nargs='+',default=['drone0','drone1','drone2','drone3'])
    parser.add_argument('--warmup',type=float,default=15.)
    parser.add_argument('--duration',type=float,default=30.)
    parser.add_argument('--offboard-vehicle',default='')
    args=parser.parse_args()
    if args.duration<=args.warmup+10 or args.duration>180:
        parser.error('duration must exceed warmup+10 and be <=180')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=StateProbe(args)
    result=None
    try:
        while time.monotonic()-node.initial<args.duration:
            rclpy.spin_once(node,timeout_sec=.01)
            node.check_pose_age()
            node.test_offboard()
        result=node.report()
        (args.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({'passed':result['passed'],'violations':len(result['pose_age_violations']),
                          'topics':{t:v['count'] for t,v in result['topics'].items()}}),flush=True)
    finally:
        if node.test_requested and not node.restored and node.offset_client.service_is_ready():
            future=node.offset_client.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        node.stream.close();node.destroy_node();rclpy.try_shutdown()
    if result is None or not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
