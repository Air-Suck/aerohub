"""Run the existing LeUAV forest8 task through AS2; no PX4 control bridge."""
import copy,json,math,time,traceback
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.msg import ControlMode,ControllerInfo,TrajectorySetpoints,PlatformStatus
from as2_msgs.action import Takeoff,Land
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleControlMode,TrajectorySetpoint
from std_srvs.srv import SetBool
from mars_quadrotor_msgs.msg import PositionCommand
from uav_as2_setpoint_adapter.super_mission import SuperMission
from uav_as2_sensor_adapter.rpc import Rpc

class ForestMission(SuperMission):
 def __init__(self):
  super().__init__();self.task=json.loads(Path('/evidence/source-task.json').read_text())
  self.origin=json.loads(Path('/evidence/origin.json').read_text())['task_to_odom_translation_m']
  self.rpc=Rpc('127.0.0.1',41472,.3);self.phase='preflight';self.failure=None
  self.wall_received={};self.latest={};self.counts={};self.speed=0.;self.max_speed=0.;self.max_ref_speed=0.
  self.flag_counts={};self.clock_last=None;self.clock_wall=time.monotonic();self.active=False
  self.route_active=False;self.started_sim=None;self.leg_started=None;self.collision_stamp=None
  self.last_collision_poll=0.;self.last_log=0.;self.last_flush=0.;self.bound_violations=0
  self.samples=open('/evidence/flight.jsonl','w');self.result={'task_source':'LeUAV profiles/super.forest8-ab.yaml','success':False,'source_task_unmodified':True,'task_to_odom_translation_m':self.origin}
  self.keep=[]
  types=[('odom','/drone0/sensor_measurements/odom',Odometry),('mode','/drone0/fmu/out/vehicle_control_mode',VehicleControlMode),('cloud','/drone0/navigation/points',PointCloud2),('map','/rog_map/occ',PointCloud2),('inflated','/rog_map/inf_occ',PointCloud2),('clock','/clock',Clock),('controller','/drone0/controller/info',ControllerInfo),('super','/planning/pos_cmd',PositionCommand),('reference','/drone0/motion_reference/trajectory',TrajectorySetpoints),('actuator','/drone0/actuator_command/trajectory',TrajectorySetpoints),('px4','/drone0/fmu/in/trajectory_setpoint',TrajectorySetpoint)]
  for k,t,c in types:self.keep.append(self.create_subscription(c,t,lambda m,k=k:self.observe(k,m),qos_profile_sensor_data))
  self.create_timer(.1,self.check)
 def now(self):return self.get_clock().now().nanoseconds*1e-9
 def local_position(self):
  p=self.pose.pose.position
  return [p.x-self.origin[0],p.y-self.origin[1],p.z-self.origin[2]]
 def observe(self,k,m):
  wall=time.monotonic();self.wall_received[k]=wall;self.latest[k]=m;self.counts[k]=self.counts.get(k,0)+1
  row={'topic':k,'wall':wall,'sim':self.now(),'phase':self.phase}
  if k=='clock':
   stamp=m.clock.sec+m.clock.nanosec*1e-9
   if self.clock_last is not None and stamp<self.clock_last and self.active:self.failure='simulation clock reset'
   if stamp!=self.clock_last:self.clock_wall=wall
   self.clock_last=stamp
   return
  if k=='odom':
   p=m.pose.pose.position;v=m.twist.twist.linear;row.update(p=[p.x,p.y,p.z],v=[v.x,v.y,v.z])
   self.speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z);self.max_speed=max(self.max_speed,self.speed)
  elif k in ('reference','actuator'):
   row['stamp']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
   if m.setpoints:
    p=m.setpoints[0];row.update(p=[p.position.x,p.position.y,p.position.z],v=[p.twist.x,p.twist.y,p.twist.z],a=[p.acceleration.x,p.acceleration.y,p.acceleration.z],yaw=p.yaw_angle)
    if k=='reference':self.max_ref_speed=max(self.max_ref_speed,math.sqrt(sum(x*x for x in row['v'])))
  elif k=='super':
   row.update(flag=m.trajectory_flag,p=[m.position.x,m.position.y,m.position.z],v=[m.velocity.x,m.velocity.y,m.velocity.z]);self.flag_counts[str(m.trajectory_flag)]=self.flag_counts.get(str(m.trajectory_flag),0)+1
  elif k=='px4':row.update(p=[float(v) for v in m.position],v=[float(v) for v in m.velocity],a=[float(v) for v in m.acceleration],yaw=float(m.yaw),timestamp=m.timestamp)
  elif k=='mode':row.update(armed=m.flag_armed,offboard=m.flag_control_offboard_enabled)
  elif k=='controller':row['mode']=m.input_control_mode.control_mode
  elif k in ('cloud','map','inflated'):return
  self.samples.write(json.dumps(row)+'\n')
 def check(self):
  wall=time.monotonic()
  if wall-self.last_flush>1:self.samples.flush();self.last_flush=wall
  if not self.active:return
  if wall-self.wall_received.get('odom',0)>.5 or wall-self.clock_wall>.5 or wall-self.wall_received.get('controller',0)>1.5:self.failure=self.failure or 'state/clock timeout'
  if self.started_sim is not None and self.now()-self.started_sim>self.task['mission_timeout_s']:self.failure=self.failure or 'mission timeout'
  if self.route_active:
   if self.now()-self.leg_started>self.task['leg_timeout_s']:self.failure=self.failure or 'waypoint timeout'
   if self.controller is not None and self.controller.input_control_mode.control_mode!=ControlMode.TRAJECTORY:self.failure=self.failure or 'AS2 trajectory mode lost (failsafe or handoff failure)'
   p=self.local_position()
   if any(v<a or v>b for v,a,b in zip(p,self.task['bounds_enu_min_m'],self.task['bounds_enu_max_m'])):self.bound_violations+=1
   if wall-self.last_collision_poll>=.2:
    try:
     c=self.rpc.call('simGetCollisionInfo','PX4');self.last_collision_poll=wall
     if c['has_collided'] and c['time_stamp']!=self.collision_stamp:self.result['collision']=c;self.failure='new AirSim flight collision'
    except Exception as exc:self.failure='collision observation failed: '+str(exc)
  if wall-self.last_log>=5:
   self.last_log=wall
   print(json.dumps({'phase':self.phase,'task_enu':self.local_position(),'speed_m_s':self.speed,'max_speed_m_s':self.max_speed,'distance_m':math.dist(self.local_position(),self.task['waypoints_enu_m'][-1]),'failure':self.failure}),flush=True)
 def healthy(self):
  if self.failure:raise RuntimeError(self.failure)
 def spin_until(self,predicate,timeout):
  def check():self.healthy();return predicate()
  self.wait(check,timeout)
 def set_phase(self,p):self.phase=p;print('PHASE '+p,flush=True);Path('/evidence/phase').write_text(p)
 def run_forest(self):
  takeoff_started=False
  try:
   self.wait(lambda:self.pose is not None and self.platform is not None and all(self.counts.get(k,0)>=3 for k in ('odom','mode','clock','controller','cloud','map','inflated')),60.)
   assert self.pose.header.frame_id=='drone0/odom'
   for k in ('cloud','map','inflated'):assert self.latest[k].width*self.latest[k].height>0,k
   self.service(SetBool,'super_adapter/enable',SetBool.Request(data=False))
   self.started_sim=self.now();self.set_phase('arming_offboard')
   self.service(SetBool,'set_arming_state',SetBool.Request(data=True));self.spin_until(lambda:self.latest['mode'].flag_armed,10.)
   if not self.latest['mode'].flag_control_offboard_enabled:self.service(SetBool,'set_offboard_mode',SetBool.Request(data=True))
   self.spin_until(lambda:self.latest['mode'].flag_control_offboard_enabled,10.)
   self.spin_until(lambda:time.monotonic()-self.wall_received.get('odom',0)<.2,5.)
   self.active=True;self.set_phase('takeoff');takeoff_started=True
   self.action(Takeoff,'TakeoffBehavior',Takeoff.Goal(takeoff_height=float(self.task['takeoff_height_m']),takeoff_speed=.5),'takeoff_success')
   self.spin_until(lambda:self.platform.status.state==PlatformStatus.FLYING,5.)
   self.set_phase('hold_after_takeoff');hold=self.now()
   self.spin_until(lambda:self.now()-hold>=self.task['hold_after_takeoff_s'],20.)
   self.collision_stamp=self.rpc.call('simGetCollisionInfo','PX4')['time_stamp']
   self.mode(ControlMode.TRAJECTORY)
   self.wait(lambda:self.controller is not None and self.controller.input_control_mode.control_mode==ControlMode.TRAJECTORY,.3)
   self.service(SetBool,'super_adapter/enable',SetBool.Request(data=True))
   self.wait(lambda:self.goal_pub.get_subscription_count()>0,.1)
   goal=copy.deepcopy(self.pose);goal.header.stamp=self.get_clock().now().to_msg()
   target=[x+o for x,o in zip(self.task['waypoints_enu_m'][-1],self.origin)]
   goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=target
   self.result['target_odom_m']=target;self.goal_pub.publish(goal)
   self.leg_started=self.now();self.route_active=True;self.set_phase('route');stable=None
   def arrived():
    nonlocal stable
    distance=math.dist(self.local_position(),self.task['waypoints_enu_m'][-1])
    if distance<=self.task['goal_tolerance_m'] and self.speed<.3:
     if stable is None:stable=self.now()
    else:stable=None
    return stable is not None and self.now()-stable>=2.
   self.spin_until(arrived,self.task['leg_timeout_s']*3+10)
   self.result['arrival_task_enu_m']=self.local_position();self.result['arrival_error_m']=math.dist(self.local_position(),self.task['waypoints_enu_m'][-1]);self.result['arrived']=True
  except BaseException as exc:
   self.result['reason']=str(exc);self.result['exception']=type(exc).__name__;traceback.print_exc()
  finally:
   self.route_active=False;self.active=False
   if takeoff_started:
    try:
     self.set_phase('hover');self.mode(ControlMode.HOVER)
     self.service(SetBool,'super_adapter/enable',SetBool.Request(data=False))
     self.set_phase('landing');self.action(Land,'LandBehavior',Land.Goal(land_speed=.5),'land_success')
     self.wait(lambda:not self.latest['mode'].flag_armed,15.)
     self.result['landed_and_disarmed']=True
    except BaseException as exc:
     self.result['recovery_error']=repr(exc);traceback.print_exc();self.set_phase('RECOVERY_REQUIRED')
   self.result.update(success=bool(self.result.get('arrived') and self.result.get('landed_and_disarmed') and 'reason' not in self.result),max_speed_m_s=self.max_speed,max_reference_speed_m_s=self.max_ref_speed,super_trajectory_flags=self.flag_counts,counts=self.counts,bounds_observation_count=self.bound_violations,final_task_enu_m=self.local_position() if self.pose else None)
   self.samples.flush();Path('/evidence/result.json').write_text(json.dumps(self.result,indent=2));print(json.dumps(self.result,indent=2),flush=True)
   if self.result.get('landed_and_disarmed'):self.set_phase('complete' if self.result['success'] else 'failed_recovered')

rclpy.init();n=ForestMission()
try:n.run_forest()
finally:n.samples.close();n.destroy_node();rclpy.shutdown()
