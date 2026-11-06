"""Read-only timing observer; independent of the application's callback workload."""
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from uav_planner_interfaces.msg import FleetSetpoint

rclpy.init()
node=Node('catora_pipeline_observer')
executor=SingleThreadedExecutor();executor.add_node(node)
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
clocks=[];points=[]
def clock(message): clocks.append((time.monotonic(),message.clock.sec+message.clock.nanosec/1e9))
def point(message):
 stamp=message.sampled_at.sec+message.sampled_at.nanosec/1e9
 points.append((time.monotonic(),stamp,clocks[-1][1] if clocks else stamp))
subscriptions=[node.create_subscription(Clock,'/clock',clock,qos),node.create_subscription(FleetSetpoint,'/planner/setpoints',point,1)]
start=time.monotonic()
while not points and time.monotonic()-start<100:executor.spin_once(timeout_sec=.02)
if not points:raise RuntimeError('No planner samples during ground probe')
def counters():
 values={}
 for p in Path('/proc').glob('[0-9]*/stat'):
  try:
   content=p.read_text(); fields=content.rsplit(')',1)[1].split()
   values[p.parent.name]=(int(fields[11])+int(fields[12]),content.split('(',1)[1].rsplit(')',1)[0])
  except (OSError,ValueError):pass
 return values
initial=counters();cpu_start=[list(map(int,l.split()[1:9])) for l in Path('/proc/stat').read_text().splitlines() if l.startswith('cpu')]
clocks.clear();points.clear();start=time.monotonic()
while time.monotonic()-start<10:executor.spin_once(timeout_sec=.01)
elapsed=time.monotonic()-start
final=counters();cpu_end=[list(map(int,l.split()[1:9])) for l in Path('/proc/stat').read_text().splitlines() if l.startswith('cpu')]
top=sorted([{'pid':pid,'command':name,'cpu_percent':100*(ticks-initial[pid][0])/os.sysconf('SC_CLK_TCK')/elapsed}
             for pid,(ticks,name) in final.items() if pid in initial],key=lambda r:r['cpu_percent'],reverse=True)[:12]
steps=[b[1]-a[1] for a,b in zip(clocks,clocks[1:])]
ages=[c-b for _,b,c in points]
util=[]
for a,b in zip(cpu_start,cpu_end):
 total=sum(b)-sum(a); idle=(b[3]+b[4])-(a[3]+a[4]);util.append(100*(1-idle/total) if total else 0)
print(json.dumps({'elapsed':elapsed,'clock_receipt_hz':len(clocks)/elapsed,'setpoint_receipt_hz':len(points)/elapsed,
 'max_clock_step':max(steps,default=0),'max_sample_age':max(ages,default=0),'min_sample_age':min(ages,default=0),
 'cpu_total_and_cores_percent':util,'top_processes':top}),flush=True)
print(json.dumps({'tf_graph':{topic:{'publishers':node.count_publishers(topic),'subscribers':node.count_subscribers(topic)}
 for topic in ['/tf']+['/drone'+str(i)+'/tf' for i in range(4)]}}),flush=True)
executor.shutdown();node.destroy_node();rclpy.shutdown()
