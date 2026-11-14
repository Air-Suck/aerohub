"""A late CAT-ORA RPC reply must not turn an expired state snapshot into a new plan."""
import json,os,signal,socketserver,subprocess,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from uav_planner_interfaces.msg import TimedTrajectorySegment
import yaml

root=Path(__file__).resolve().parent
count=0
class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        global count
        request=json.loads(self.rfile.readline());count+=1
        if count==1:time.sleep(1.3)
        self.wfile.write((json.dumps({'ok':True,'trajectories_enu':[[a,b] for a,b in zip(request['starts_enu'],request['goals_enu'])]})+'\n').encode())
class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address=True
    daemon_threads=True
server=Server(('127.0.0.1',28509),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
rclpy.init();node=Node('slow_proxy_contract_test');executor=SingleThreadedExecutor();executor.add_node(node)
config={'task':{'task_id':'slow-proxy'},'vehicles':['slowprobe'],
        'deployment':{'slowprobe':{'namespace':'/slowprobe','topics':{'pose':'/slowprobe/pose'},'px4_origin_in_fleet_ned':[0,0,0]}},
        'targets':{'slowprobe':{'x':2.,'y':1.,'z':-4.}},
        'planner':{'parameters':{'trajectory_dt':.1,'max_velocity':1.,'max_acceleration':1.,'valid_from_lead':.2}}}
path=root/'slow-proxy.yaml';path.write_text(yaml.safe_dump(config))
seen=[];publisher=node.create_publisher(PoseStamped,'/slowprobe/pose',1)
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscription=node.create_subscription(TimedTrajectorySegment,'/planner/trajectory',lambda m:seen.append((count,m)),qos)
def publish():
    m=PoseStamped();m.header.frame_id='slowprobe/odom';m.header.stamp=node.get_clock().now().to_msg();m.pose.orientation.w=1.;m.pose.position.z=4.;publisher.publish(m)
node.create_timer(.05,publish)
log=(root/'slow-proxy.log').open('w')
p=subprocess.Popen(['ros2','run','uav_cat_ora_adapter','cat_ora_adapter_node','--ros-args','-p','resolved_config:='+str(path),'-p','proxy_port:=28509'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    deadline=time.monotonic()+20
    while not seen and time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
    assert seen,'no plan after fresh retry'
    assert seen[0][0]>=2,'expired snapshot was published as a fresh plan'
    assert seen[0][1].task_id=='slow-proxy'
    result={'passed':True,'requests_before_first_trajectory':seen[0][0],'first_response_delay_seconds':1.3}
    (root/'slow-proxy-result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
finally:
    if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
    try:p.wait(timeout=4)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    try:os.killpg(p.pid,signal.SIGKILL)
    except ProcessLookupError:pass
    log.close();server.shutdown();server.server_close();executor.shutdown();node.destroy_node();rclpy.shutdown()
