"""Replay recorded algorithm inputs in an isolated ROS graph; no AS2/platform is launched."""
import argparse
import json
from pathlib import Path
import signal
import sqlite3
import subprocess
import time
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from uav_follow_reference_acro.msg import ControlFrame,ControlSession
p=argparse.ArgumentParser();p.add_argument('evidence',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--reference-speed',type=float,default=.25);p.add_argument('--allow-degraded',action='store_true');a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True)
bag=a.evidence/'rosbag';db=next(bag.glob('*.db3'))
connection=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
first=connection.execute('select min(timestamp) from messages').fetchone()[0]
active=None
for stamp,data in connection.execute("select m.timestamp,m.data from messages m join topics t on m.topic_id=t.id where t.name='/drone0/external_acro/session' order by m.timestamp"):
 if deserialize_message(data,ControlSession).active:active=stamp;break
if active is None:raise RuntimeError('no active session in bag')
offset=max(0,(active-first)/1e9-1)
hover=json.loads((a.evidence/'hover.json').read_text())['normalized_thrust']
rclpy.init();n=Node('ipc_replay_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
frames=[];frame_wall=[];timings=[]
n.create_subscription(ControlFrame,'/replay/control',lambda m:(frames.append(m),frame_wall.append(time.monotonic())),qos_profile_sensor_data)
def health(m):
 if m.data.startswith('solved wall_ms='):
  values=dict(x.split('=')for x in m.data.removeprefix('solved ').split(','));timings.append({'wall_ms':float(values['wall_ms']),'iterations':int(values['iterations']),'setup_ms':float(values.get('setup_ms',0)),'optimize_ms':float(values.get('optimize_ms',0)),'corridor_before':int(values.get('corridor_before',0)),'corridor_after':int(values.get('corridor_after',0))})
n.create_subscription(String,'/replay/health',health,10)
log=(a.output/'node.log').open('w')
node=subprocess.Popen([get_package_prefix('ipc_native_core')+'/lib/ipc_native_core/ipc_node','--ros-args','-r','__ns:=/drone0','-p','use_sim_time:=true','-p','hover_percentage:='+str(hover),'-p','reference_speed_mps:='+str(a.reference_speed),'-p','diagnostic_qp_path:='+str(a.output/'failed-qp.json'),'-r','navigation/points:=/replay/input_points','-r','external_acro/control:=/replay/control','-r','ipc/health:=/replay/health','-r','ipc/processed_points:=/replay/processed_points'],stdout=log,stderr=subprocess.STDOUT)
player=None
try:
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
 player=subprocess.Popen(['ros2','bag','play',str(bag),'--start-offset',str(offset),'--topics','/clock','/drone0/navigation/odom','/drone0/ipc/processed_points','/drone0/external_acro/session','--remap','/drone0/ipc/processed_points:=/replay/input_points'],stdout=log,stderr=subprocess.STDOUT)
 end=time.monotonic()+300
 while player.poll() is None:
  if time.monotonic()>end:raise TimeoutError('replay timeout')
  if node.poll() is not None:raise RuntimeError('node exited')
  rclpy.spin_once(n,timeout_sec=.01)
 for _ in range(20):rclpy.spin_once(n,timeout_sec=.01)
 ms=sorted(x['wall_ms']for x in timings)
 result={'scope':'recorded postprocessed inputs only; no physical closed loop','player_code':player.returncode,'frames':len(frames),'valid_frames':sum(f.solver_ok for f in frames),'solve_count':len(timings),'max_valid_frame_gap_wall_s':max((b-a for a,b in zip([w for w,f in zip(frame_wall,frames)if f.solver_ok],[w for w,f in zip(frame_wall,frames)if f.solver_ok][1:])),default=0),'solve_p50_ms':ms[len(ms)//2] if ms else None,'solve_p95_ms':ms[min(len(ms)-1,int(.95*len(ms)))] if ms else None,'solve_max_ms':max(ms) if ms else None,'timings':timings}
 result['timing_gate_passed']=result['max_valid_frame_gap_wall_s']<=.1
 (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:v for k,v in result.items()if k!='timings'},indent=2))
 if not timings or not any(f.solver_ok for f in frames) or player.returncode:raise RuntimeError('replay did not exercise real solver')
 if not result['timing_gate_passed'] and not a.allow_degraded:raise RuntimeError('recorded-input timing gate failed; inspect rejected solves and counterfactual feedback limits')
finally:
 if player is not None and player.poll() is None:player.send_signal(signal.SIGINT);player.wait(timeout=5)
 node.send_signal(signal.SIGINT);node.wait(timeout=5);log.close();n.destroy_node();rclpy.shutdown()
