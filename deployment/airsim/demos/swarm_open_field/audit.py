"""Read-only DDS evidence: native trajectories, PVA segments, actual AS2 references."""
import json,math,os,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from as2_msgs.msg import TrajectorySetpoints
from traj_utils.msg import PolyTraj
from uav_planner_interfaces.msg import TimedTrajectorySegment


def main():
    rclpy.init();n=Node('swarm_readonly_audit',parameter_overrides=[Parameter('use_sim_time',value=True)])
    vehicles=tuple(os.environ['AEROHUB_SWARM_VEHICLES'].split(','))
    if vehicles!=tuple('drone'+str(i) for i in range(len(vehicles))):raise ValueError('native drone_id requires drone0..droneN-1 order')
    out=Path('/results/dds-audit.json');counts={v:{'native_polynomials':0,'as2_references':0,'finite_pva_references':0,'broadcasts':0} for v in vehicles}
    state={'vehicles':counts,'segments':0,'pva_segments':0,'bad_references':0,'bad_segments':0};subscriptions=[];running=True
    def reference(msg,v):
        counts[v]['as2_references']+=1
        if len(msg.setpoints)!=1:state['bad_references']+=1;return
        p=msg.setpoints[0]
        if msg.header.frame_id!=v+'/odom' or not all(math.isfinite(x) for x in (p.position.x,p.position.y,p.position.z,p.yaw_angle)):state['bad_references']+=1
        if all(math.isfinite(x) for x in (p.twist.x,p.twist.y,p.twist.z,p.acceleration.x,p.acceleration.y,p.acceleration.z)):counts[v]['finite_pva_references']+=1
    def polynomial(msg):
        v='drone'+str(msg.drone_id)
        if v in counts:
            counts[v]['native_polynomials']+=1
            t=msg.duration[-1];k=(len(msg.duration)-1)*(msg.order+1)
            counts[v]['last_native_end_enu']=[sum(c[k+msg.order-p]*t**p for p in range(msg.order+1)) for c in (msg.coef_x,msg.coef_y,msg.coef_z)]
    def broadcast(msg):
        v='drone'+str(msg.drone_id)
        if v in counts:counts[v]['broadcasts']+=1
    def segment(msg):
        state['segments']+=1
        if len(msg.vehicle_trajectories)!=len(vehicles):state['bad_segments']+=1
        elif all(t.valid_fields&3==3 for t in msg.vehicle_trajectories):state['pva_segments']+=1
    for i,v in enumerate(counts):
        subscriptions.append(n.create_subscription(TrajectorySetpoints,f'/{v}/motion_reference/trajectory',lambda msg,v=v:reference(msg,v),qos_profile_sensor_data))
        subscriptions.append(n.create_subscription(PolyTraj,f'/drone_{i}_planning/trajectory',polynomial,10))
    subscriptions.append(n.create_subscription(PolyTraj,'/swarm_formation/broadcast',broadcast,100))
    subscriptions.append(n.create_subscription(TimedTrajectorySegment,'/planner/trajectory',segment,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stop)
    start=time.monotonic()
    try:
        while running and time.monotonic()-start<240:
            rclpy.spin_once(n,timeout_sec=.1)
            state['wall_seconds']=time.monotonic()-start;out.write_text(json.dumps(state,indent=2)+'\n')
    finally:
        n.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
