"""Read-only ROS observer for point-cloud stage ages. Never arms or commands motion."""
import argparse
import collections
import json
import math
from pathlib import Path
import time


def distribution(values):
    values=sorted(values)
    if not values:return None
    return {'count':len(values),'min_ms':values[0]*1000,
            'p50_ms':values[len(values)//2]*1000,
            'p95_ms':values[min(len(values)-1,int(.95*len(values)))]*1000,
            'max_ms':values[-1]*1000}


def summarize(events):
    grouped=collections.defaultdict(list);by_stamp=collections.defaultdict(dict)
    for e in events:
        grouped[e['stage']].append(e)
        by_stamp[e['stamp_ns']].setdefault(e['stage'],e)
    stages={stage:{'age_sim':distribution([(e['observer_sim_ns']-e['stamp_ns'])/1e9 for e in entries if e['observer_sim_ns']>0 and e['observer_sim_ns']>=e['stamp_ns']]),
                   'uninitialized_clock_records':sum(e['observer_sim_ns']<=0 for e in entries),
                   'future_stamp_records':sum(0<e['observer_sim_ns']<e['stamp_ns'] for e in entries),
                   'unique_scans':len({e['stamp_ns']for e in entries}),
                   'repeated_stamp_records':len(entries)-len({e['stamp_ns']for e in entries})} for stage,entries in grouped.items()}
    pairs={}
    for left,right in [('raw','registered'),('registered','ipc_processed'),('raw','ipc_processed')]:
        matched=[x for x in by_stamp.values() if left in x and right in x]
        pairs[left+'_to_'+right]={'observer_receive_wall':distribution([(x[right]['wall_monotonic_ns']-x[left]['wall_monotonic_ns'])/1e9 for x in matched]),
                                'observer_receive_sim':distribution([(x[right]['observer_sim_ns']-x[left]['observer_sim_ns'])/1e9 for x in matched if x[right]['observer_sim_ns']>0 and x[left]['observer_sim_ns']>0]),
                                'count':len(matched)}
    report={'scope':'Passive ROS callbacks at one observer; does not modify publishers or perform flight',
            'complete_required_stages':all(stages.get(k,{}).get('age_sim') is not None for k in ('raw','registered')),
            'stage_counts':{k:len(v)for k,v in grouped.items()},'stage_ages':stages,'same_stamp_pairs':pairs,
            'limitations':['Callback receive differences include observer scheduling and transport; negative differences may reflect observer ordering, not time reversal.',
                           'ROS clock is quantized; individual age samples are estimates.',
                           'Unmatched or pre-discovery scans are not treated as measured stage latency.']}
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=25.)
    parser.add_argument('--namespace',default='drone0')
    parser.add_argument('--events',type=Path,help='Analyze saved events only; no ROS dependency or node')
    args=parser.parse_args()
    if not math.isfinite(args.duration) or args.duration<=0:parser.error('duration must be positive')
    args.output.mkdir(parents=True,exist_ok=False)
    if args.events:
        data=json.loads(args.events.read_text());report=summarize(data['clouds']);report['source_events']=str(args.events)
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
        if not report['complete_required_stages']:raise SystemExit('insufficient valid raw/registered clock samples')
        return
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2
    rclpy.init();node=rclpy.create_node('cloud_stage_age_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    events=[];clocks=[]
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    def clock(message):
        clocks.append({'wall_monotonic_ns':time.monotonic_ns(),'sim_ns':message.clock.sec*10**9+message.clock.nanosec})
    node.create_subscription(Clock,'/clock',clock,qos)
    def receive(stage,message):
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        events.append({'stage':stage,'stamp_ns':stamp,'observer_sim_ns':node.get_clock().now().nanoseconds,
                       'wall_monotonic_ns':time.monotonic_ns(),'points':message.width*message.height})
    for stage,topic in [('raw','sensor_measurements/Lidar/points'),('registered','navigation/points'),('ipc_processed','ipc/processed_points')]:
        node.create_subscription(PointCloud2,f'/{args.namespace}/{topic}',lambda m,s=stage:receive(s,m),qos)
    try:
        end=time.monotonic()+args.duration
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        report=summarize(events)
        (args.output/'events.json').write_text(json.dumps({'clocks':clocks,'clouds':events},indent=2)+'\n')
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
        if not report['complete_required_stages']:raise SystemExit('insufficient valid raw/registered clock samples')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
