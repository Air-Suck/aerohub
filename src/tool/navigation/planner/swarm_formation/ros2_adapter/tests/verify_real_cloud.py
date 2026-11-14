"""Read actual navigation inputs; inspect their consumption by Swarm's real GridMap.

Only the bridge/probe processes created here are paused or stopped. No commands
are sent to PX4, AS2, AirSim or an existing navigation publisher.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import yaml

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def points(message):
    fields={f.name:f.offset for f in message.fields}
    count=message.width*message.height
    for i in range(0,count,max(1,count//50)):
        yield tuple(struct.unpack_from('<f',message.data,i*message.point_step+fields[axis])[0] for axis in ('x','y','z'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle',default='drone0')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=Path(__file__).parents[1]/'native/plan_manage/config/humble.yaml')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('verify_real_swarm_cloud',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    data={name:{} for name in ('source','registered','source_odom','mapped_odom')}
    stats={'raw_lidar_messages':0,'inflated_map_messages':0,'inflated_map_points':0}
    health={'value':None}

    def receive(name,msg):
        data[name][stamp(msg)]=msg
        while len(data[name])>100:data[name].pop(next(iter(data[name])))

    def map_received(msg):
        if msg.width:
            stats['inflated_map_messages']+=1;stats['inflated_map_points']=msg.width*msg.height
            stats['map_frame']=msg.header.frame_id;stats['map_stamp_ns']=stamp(msg)

    subscriptions=[]
    for topic,kind,callback in (
        (f'/{args.vehicle}/navigation/points',PointCloud2,lambda m:receive('source',m)),
        (f'/{args.vehicle}/navigation/odom',Odometry,lambda m:receive('source_odom',m)),
        ('/swarm_input_validation/points',PointCloud2,lambda m:receive('registered',m)),
        ('/swarm_input_validation/odom',Odometry,lambda m:receive('mapped_odom',m)),
        ('/swarm_map_validation/grid_map/occupancy_inflate',PointCloud2,map_received),
        ('/swarm_map_validation/grid_map/input_healthy',Bool,lambda m:health.update(value=m.data)),
        (f'/{args.vehicle}/sensor_measurements/Lidar/points',PointCloud2,
         lambda m:stats.update(raw_lidar_messages=stats['raw_lidar_messages']+1)),
    ):
        subscriptions.append(node.create_subscription(kind,topic,callback,qos_profile_sensor_data))
    children=[];streams=[]

    def start(name,command):
        stream=(args.output/(name+'.log')).open('w');streams.append(stream)
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(process);return process

    def wait(predicate,timeout=30):
        deadline=time.monotonic()+timeout
        while not predicate():
            if any(p.poll() is not None for p in children):raise RuntimeError('validation child exited; inspect logs')
            if time.monotonic()>deadline:raise TimeoutError(f'real cloud validation timed out: counts={ {k:len(v) for k,v in data.items()} }, map={stats}')
            rclpy.spin_once(node,timeout_sec=.03)

    client=node.create_client(Trigger,'/swarm_map_validation/inspect')

    def inspect():
        future=client.call_async(Trigger.Request());wait(future.done,5)
        return json.loads(future.result().message)

    try:
        start_args=['--ros-args','-p','use_sim_time:=true']
        bridge=start('bridge',['ros2','run','plan_env','navigation_bridge_node']+start_args+[
            '-r','__ns:=/swarm_input_validation',
            '-p',f'source_frame:={args.vehicle}/odom','-p',f'body_frame:={args.vehicle}/base_link',
            '-p','map_frame:=swarm_map_validation','-p','origin_enu:=[1.0, 2.0, 1.0]',
            '-p','origin_yaw:=0.35','-p',f'cloud_topic:=/{args.vehicle}/navigation/points',
            '-p',f'odom_topic:=/{args.vehicle}/navigation/odom'])
        config=yaml.safe_load(args.config.read_text())
        config.update({'use_sim_time':True,'grid_map.frame_id':'swarm_map_validation',
                       'grid_map.input_timeout':1.0,'grid_map.ground_height':-2.0,
                       'grid_map.map_size_z':8.0,'grid_map.virtual_ceil_height':-1.0,
                       'grid_map.visualization_truncate_height':6.0})
        settings=args.output/'map-parameters.yaml'
        settings.write_text(yaml.safe_dump({'/**':{'ros__parameters':config}}))
        start('map',['ros2','run','plan_env','map_probe_node','--ros-args',
            '-r','__node:=swarm_map_validation','--params-file',str(settings),
            '-r','~/grid_map/cloud:=/swarm_input_validation/points',
            '-r','~/grid_map/odom:=/swarm_input_validation/odom'])
        wait(lambda:client.service_is_ready() and len(data['registered'])>=10 and stats['inflated_map_messages']>=5,60)
        report=None
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            report=inspect()
            n=report['sampled_input_points']
            if report['healthy'] and n>=30 and report['occupied_matches']/n>.8 and report['esdf_surface_matches']/n>.8:break
            rclpy.spin_once(node,timeout_sec=.1)
        assert report and report['healthy'] and report['sampled_input_points']>=30, report
        assert report['occupied_matches']/report['sampled_input_points']>.8, report
        assert report['esdf_surface_matches']/report['sampled_input_points']>.8, report
        matches=set(data['source'])&set(data['registered']);assert len(matches)>=5
        max_error=0.0;c=math.cos(.35);s=math.sin(.35)
        for at in sorted(matches)[-5:]:
            source=data['source'][at];mapped=data['registered'][at]
            assert source.header.frame_id==args.vehicle+'/odom'
            assert mapped.header.frame_id=='swarm_map_validation' and stamp(source)==stamp(mapped)
            assert source.fields==mapped.fields and source.width==mapped.width and source.point_step==mapped.point_step
            for p,q in zip(points(source),points(mapped)):
                expected=(1+c*p[0]-s*p[1],2+s*p[0]+c*p[1],1+p[2])
                max_error=max(max_error,math.dist(expected,q))
        assert max_error<1e-4,max_error
        odom_matches=set(data['source_odom'])&set(data['mapped_odom']);assert odom_matches
        at=max(odom_matches);source=data['source_odom'][at];mapped=data['mapped_odom'][at]
        assert source.twist==mapped.twist and mapped.child_frame_id==source.child_frame_id
        # Stop only our bridge: native map must declare missing real data, then recover.
        (args.output/'mapping.json').write_text(json.dumps(report,indent=2)+'\n')
        os.killpg(bridge.pid,signal.SIGSTOP)
        deadline=time.monotonic()+6
        detected=False
        while time.monotonic()<deadline:
            detected=not inspect()['healthy']
            if detected:break
            rclpy.spin_once(node,timeout_sec=.05)
        assert detected,'map did not detect input loss'
        wait(lambda:health['value'] is False,3)
        os.killpg(bridge.pid,signal.SIGCONT)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            recovered=inspect()['healthy']
            if recovered:break
            rclpy.spin_once(node,timeout_sec=.05)
        assert recovered,'map did not recover on fresh actual data'
        wait(lambda:health['value'] is True,3)
        assert stats['raw_lidar_messages']>0 and stats['map_stamp_ns']>0
        result=dict(source=f'/{args.vehicle}/navigation/points',qos='best_effort',
                    received_registered_messages=len(data['registered']),max_rigid_transform_error_m=max_error,
                    stamp_and_fields_preserved=True,body_twist_preserved=True,
                    native_map=report,input_loss_detected=True,input_recovery=True,
                    native_health_topic_verified=True,**stats)
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        for process in children:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGCONT);os.killpg(process.pid,signal.SIGTERM)
        for process in children:
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
