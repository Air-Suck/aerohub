"""Real ROS adapter processes with isolated fake AS2 services; never arm a vehicle."""
import json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from as2_msgs.msg import ControlMode,ControllerInfo,TrajectorySetpoints
from as2_msgs.srv import SetControlMode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from uav_planner_interfaces.msg import FleetSetpoint,VehicleSetpoint,BehaviorResult
import yaml

root=Path(__file__).resolve().parent
rclpy.init();node=Node('cat_ora_adapter_contract_test');executor=SingleThreadedExecutor();executor.add_node(node)
vehicles=['adapterprobe'+str(i) for i in range(4)]
deployment={v:{'namespace':'/'+v,'topics':{'pose':'/'+v+'/catora/local_pose'},'px4_origin_in_fleet_ned':[0,0,0]} for v in vehicles}
(root/'deployment.yaml').write_text(yaml.safe_dump(deployment))
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
refs={v:0 for v in vehicles};poses={v:0 for v in vehicles};modes={v:7 for v in vehicles};requests=[]
reject=None;status=None;task_id='contract';state={};feed_odom=True;last_odom={};children=[];resources=[]
odom_pub={v:node.create_publisher(Odometry,'/'+v+'/sensor_measurements/odom',1) for v in vehicles}
info_pub={v:node.create_publisher(ControllerInfo,'/'+v+'/controller/info',1) for v in vehicles}
point_pub=node.create_publisher(FleetSetpoint,'/planner/setpoints',1)
status_pub=node.create_publisher(BehaviorResult,'/planner/status',1)

def pose(v,msg):poses[v]+=1

def reference(v,msg):
    assert msg.header.frame_id==v+'/odom' and len(msg.setpoints)==1
    assert math.isnan(msg.setpoints[0].twist.x) and math.isnan(msg.setpoints[0].acceleration.z)
    refs[v]+=1

def telemetry(msg):
    global state
    value=json.loads(msg.data)
    if value['task_id']!=task_id:return
    if os.environ.get('MULTI_OWNER')=='1':
        if state.get('task_id')!=task_id:state={'task_id':task_id}
        for key in ('mode_ready','published','stop_confirmed','stop_errors'):
            state.setdefault(key,{}).update(value.get(key,{}))
        state['stop_reason']=value.get('stop_reason')
    else:state=value

def service(v,request,response):
    requests.append((v,request.control_mode.control_mode))
    response.success=v!=reject
    if response.success:modes[v]=2 if request.control_mode.control_mode==ControlMode.HOVER else request.control_mode.control_mode
    return response

for v in vehicles:
    resources.append(node.create_subscription(PoseStamped,'/'+v+'/catora/local_pose',lambda m,v=v:pose(v,m),qos))
    resources.append(node.create_subscription(TrajectorySetpoints,'/'+v+'/motion_reference/trajectory',lambda m,v=v:reference(v,m),qos))
    resources.append(node.create_service(SetControlMode,'/'+v+'/controller/set_control_mode',lambda q,r,v=v:service(v,q,r)))
resources.append(node.create_subscription(String,'/planner/as2_bridge_state',telemetry,1))

def feed():
    now=node.get_clock().now().to_msg()
    for v in vehicles:
        info=ControllerInfo();info.input_control_mode.control_mode=modes[v];info.input_control_mode.yaw_mode=ControlMode.YAW_ANGLE;info_pub[v].publish(info)
        if feed_odom:
            m=Odometry();m.header.frame_id=v+'/odom';m.header.stamp=now;m.child_frame_id=v+'/base_link';m.pose.pose.orientation.w=1.;m.pose.pose.position.z=4.
            odom_pub[v].publish(m);last_odom[v]=m
    fleet=FleetSetpoint();fleet.task_id=task_id;fleet.sampled_at=now
    for v in vehicles:
        p=VehicleSetpoint();p.vehicle_id=v;p.position.x=2.;p.position.y=1.;p.position.z=-4.
        p.velocity.x=p.velocity.y=p.velocity.z=math.nan;p.acceleration.x=p.acceleration.y=p.acceleration.z=math.nan;p.yaw=math.nan
        fleet.setpoints.append(p)
    point_pub.publish(fleet)
    if status is not None:
        m=BehaviorResult();m.task_id=task_id;m.overall_status=status;status_pub.publish(m)

resources.append(node.create_timer(.04,feed))
def wait(predicate,timeout=20):
    until=time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>until:raise TimeoutError('contract condition timed out: '+str(state))
        executor.spin_once(timeout_sec=.02)
def spin(seconds):
    until=time.monotonic()+seconds;wait(lambda:time.monotonic()>=until,seconds+2)
def start(name,command):
    log=(root/(name+'.log')).open('w');p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);log.close();children.append(p);return p
def output(name):
    path=root/(name+'.yaml');path.write_text(yaml.safe_dump({'task':{'task_id':task_id},'vehicles':vehicles,'deployment':deployment}))
    command=['ros2','run','uav_cat_ora_adapter','cat_ora_as2_adapter_node','--ros-args','-p','resolved_config:='+str(path)]
    if os.environ.get('MULTI_OWNER')=='1':
        return [start(name+'-'+v,command+['-p','vehicle_id:='+v,'-r','__node:=owner_'+v]) for v in vehicles]
    return start(name,command)
def stop(p):
    if isinstance(p,list):
        for child in p:stop(child)
        return
    if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    try:os.killpg(p.pid,signal.SIGKILL)
    except ProcessLookupError:pass

result={}
try:
    start('input',['ros2','run','uav_cat_ora_adapter','cat_ora_state_adapter_node','--ros-args','-p','deployment_config:='+str(root/'deployment.yaml')])
    p=output('output');wait(lambda:all(x>=5 for x in refs.values()) and all(poses.values()))
    result['fresh_input_and_pva']=True
    if os.environ.get('MULTI_OWNER')=='1':
        for v in vehicles:
            publishers=node.get_publishers_info_by_topic('/'+v+'/motion_reference/trajectory')
            assert len(publishers)==1 and publishers[0].node_name=='owner_'+v,publishers
        result['one_owner_per_vehicle']=True
    feed_odom=False;spin(.4);before=poses[vehicles[0]]
    for kind in ('frame','old','future','duplicate','nan','quaternion'):
        m=Odometry();m.header.frame_id=vehicles[0]+'/odom';m.header.stamp=node.get_clock().now().to_msg();m.pose.pose.orientation.w=1.
        if kind=='frame':m.header.frame_id='earth'
        elif kind=='old':m.header.stamp.sec-=2
        elif kind=='future':m.header.stamp.sec+=1
        elif kind=='duplicate':m=last_odom[vehicles[0]]
        elif kind=='nan':m.pose.pose.position.x=math.nan
        elif kind=='quaternion':m.pose.pose.orientation.w=0.
        odom_pub[vehicles[0]].publish(m);spin(.15)
    assert poses[vehicles[0]]==before,(before,poses)
    feed_odom=True;result['invalid_state_rejected']=True
    status=BehaviorResult.PAUSED;wait(lambda:all(state.get('stop_confirmed',{}).values()) and len(state.get('stop_confirmed',{}))==4)
    spin(.2);before=dict(refs)
    for v in vehicles:modes[v]=7
    spin(.4);assert refs==before
    for v in vehicles:modes[v]=2
    spin(.2);status=BehaviorResult.RUNNING;spin(.4);assert refs==before
    # Model a fresh explicit task request to AS2 before resumed references can flow.
    clients=[node.create_client(SetControlMode,'/'+v+'/controller/set_control_mode') for v in vehicles]
    futures=[]
    for client in clients:
        wait(client.service_is_ready);q=SetControlMode.Request();q.control_mode.control_mode=7;q.control_mode.yaw_mode=1;futures.append(client.call_async(q))
    wait(lambda:all(f.done() for f in futures));wait(lambda:all(refs[v]>before[v] for v in vehicles))
    result['pause_and_explicit_resume']=True
    status=BehaviorResult.CANCELED;wait(lambda:state.get('stop_reason')==BehaviorResult.CANCELED and all(state.get('stop_confirmed',{}).values()))
    spin(.2);before=dict(refs)
    for v in vehicles:modes[v]=7
    spin(.4);assert refs==before
    # A completed HOVER remains confirmed when the task subsequently enters LAND/SPEED.
    for v in vehicles:modes[v]=4
    spin(8.2)
    assert not state.get('stop_errors') and all(state.get('stop_confirmed',{}).values()),state
    result['cancel_calls_as2_and_latches']=True
    result['landing_does_not_invalidate_completed_stop']=True
    stop(p);task_id='rejected-stop';status=None;state={};refs={v:0 for v in vehicles}
    for v in vehicles:modes[v]=7
    p=output('rejection');wait(lambda:all(refs.values()))
    reject=vehicles[2];status=BehaviorResult.FAILED
    wait(lambda:reject in state.get('stop_errors',{}))
    assert not state['stop_confirmed'][reject]
    result['as2_stop_rejection_reported']=True
    result['as2_requests']=requests
    result['passed']=True
finally:
    for p in reversed(children):stop(p)
    (root/'adapter-contract.json').write_text(json.dumps(result,indent=2)+'\n')
    executor.shutdown();node.destroy_node();rclpy.shutdown()
print(json.dumps(result,indent=2))
