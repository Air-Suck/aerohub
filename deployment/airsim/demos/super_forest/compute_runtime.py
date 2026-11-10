"""Board capability startup and readiness; no mission, arming, or simulator RPC."""
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from as2_msgs.msg import PlatformInfo
from std_msgs.msg import String
import yaml


class ComputeRuntime(Node):
    def __init__(self):
        super().__init__('super_compute_runtime',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.root=Path(os.environ['SUPER_APP_ROOT']);self.out=Path(os.environ['SUPER_EVIDENCE_ROOT'])
        self.odom=None;self.platform=None;self.origin=None;self.children=[];self.counts=collections.Counter()
        self.adapter={};self.ready=False;self.error='';self.stopping=False
        self.publisher=self.create_publisher(String,'/drone0/super_runtime/state',10)
        self.create_subscription(Odometry,'/drone0/sensor_measurements/odom',lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
        self.create_subscription(PlatformInfo,'/drone0/platform/info',lambda m:setattr(self,'platform',m),qos_profile_sensor_data)
        self.create_subscription(String,'/drone0/super_adapter/state',lambda m:setattr(self,'adapter',json.loads(m.data)),10)
        for key,topic in (('cloud','/drone0/navigation/points'),('map','/rog_map/occ'),('inflated','/rog_map/inf_occ')):
            self.create_subscription(PointCloud2,topic,lambda m,k=key:self.counts.update([k]) if m.width*m.height else None,qos_profile_sensor_data)
        self.create_timer(.2,self.publish_state)

    def publish_state(self):
        state={'ready':self.ready,'error':self.error,'origin':self.origin,'counts':dict(self.counts),
               'role':'compute','pid':os.getpid(),'children':{name:p.pid for name,p,_ in self.children}}
        self.publisher.publish(String(data=json.dumps(state)))

    def start(self,name,command):
        stream=(self.out/(name+'.log')).open('w')
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.children.append((name,child,stream))

    def check(self):
        for name,p,_ in self.children:
            if p.poll() is not None:raise RuntimeError(name+' exited: '+str(p.returncode))
        if self.stopping:raise InterruptedError('Compute shutdown requested')

    def wait(self,predicate,timeout):
        end=time.monotonic()+timeout
        while not predicate():
            self.check()
            if time.monotonic()>end:raise TimeoutError('Compute readiness timed out')
            rclpy.spin_once(self,timeout_sec=.02)

    def prepare(self):
        self.start('navigation',['ros2','run','as2_navigation_inputs','navigation_inputs_node','--ros-args',
            '-r','__ns:=/drone0','--params-file',str(self.root/'navigation.yaml')])
        self.wait(lambda:self.odom is not None and self.platform is not None and self.counts['cloud']>=3 and self.get_clock().now().nanoseconds>0,90)
        if self.platform.armed:raise RuntimeError('Capability startup requires a disarmed platform')
        settled=self.get_clock().now().nanoseconds
        self.wait(lambda:(self.get_clock().now().nanoseconds-settled)/1e9>=10,30)
        p=self.odom.pose.pose.position;self.origin=[p.x,p.y,p.z]
        if self.odom.header.frame_id!='drone0/odom' or not all(math.isfinite(x) for x in self.origin):
            raise RuntimeError('Invalid compute registration origin')
        task=json.loads((self.root/'task.json').read_text())
        config=yaml.safe_load((self.root/'planner.yaml').read_text())
        historical=json.loads((self.root/'reference_origin.json').read_text())['task_to_odom_translation_m']
        config['rog_map']['fix_map_origin']=[a+b-c for a,b,c in zip(config['rog_map']['fix_map_origin'],self.origin,historical)]
        config['rog_map']['virtual_ground_height']=task['virtual_ground_height_m']+self.origin[2]
        config['rog_map']['virtual_ceil_height']=task['virtual_ceil_height_m']+self.origin[2]
        path=self.out/'planner.yaml';path.write_text(yaml.safe_dump(config))
        self.start('super',['ros2','run','super_planner','fsm_node','--ros-args','-p','use_sim_time:=true','-p','config_name:='+str(path)])
        self.start('adapter',['ros2','run','uav_super_adapter','super_timed_path_adapter','--ros-args','-r','__ns:=/drone0','-p','use_sim_time:=true'])
        self.wait(lambda:self.counts['map']>=3 and self.counts['inflated']>=3 and 'enabled' in self.adapter,60)
        if self.adapter['enabled']:raise RuntimeError('Adapter must be inactive at capability readiness')
        self.ready=True;self.publish_state()
        (self.out/'ready.json').write_text(json.dumps({'ready':True,'origin':self.origin,'pid':os.getpid(),
            'planner_config_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'counts':dict(self.counts)})+'\n')

    def run(self):
        try:
            self.prepare()
            while not self.stopping:
                self.check();rclpy.spin_once(self,timeout_sec=.1)
        except InterruptedError:pass
        except BaseException as exc:
            self.error=str(exc);self.ready=False;self.publish_state()
            (self.out/'runtime-error.json').write_text(json.dumps({'error':self.error})+'\n')
            raise
        finally:
            self.ready=False;self.publish_state()
            for _,p,_ in self.children:
                if p.poll() is None:
                    try:os.killpg(p.pid,signal.SIGTERM)
                    except ProcessLookupError:pass
            for name,p,stream in self.children:
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
                stream.close()
            (self.out/'stopped.json').write_text(json.dumps({'stopped':True,'children':{name:p.returncode for name,p,_ in self.children}})+'\n')


def main():
    rclpy.init();node=ComputeRuntime()
    def stop(*_):node.stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:node.run()
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
