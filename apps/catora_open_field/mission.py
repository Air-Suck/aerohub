"""A real four-aircraft AirSim/PX4/AS2 CAT-ORA application, with flight evidence."""
import itertools
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos_profile_sensor_data = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
from as2_msgs.action import Takeoff, Land
from as2_msgs.msg import PlatformInfo, PlatformStatus, ControlMode, ControllerInfo, TrajectorySetpoints
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommandAck, VehicleControlMode
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from uav_planner_interfaces.msg import BehaviorResult, FleetSetpoint, TimedTrajectorySegment
from uav_planner_interfaces.srv import ControlPlanner
import yaml
from airsim_rpc import Rpc


class VerifiedStreamLoss(RuntimeError):
    """Raised only after the injected output pause caused actual AS2 HOVER feedback."""


class Mission:
    def __init__(self):
        self.root = Path(os.environ.get('CATORA_APP_ROOT', '/opt/catora_app'))
        self.output = Path(os.environ.get('CATORA_OUTPUT', '/results'))
        self.output.mkdir(parents=True, exist_ok=True)
        self.task = yaml.safe_load((self.root/'task.yaml').read_text())
        self.deployment = yaml.safe_load((self.root/'deployment.yaml').read_text())
        self.spawns = {v:list(d['px4_origin_in_fleet_ned']) for v,d in self.deployment.items()}
        self.vehicles = list(self.deployment)
        self.rpc = Rpc(os.environ.get('AIRSIM_HOST', '127.0.0.1'), int(os.environ.get('AIRSIM_PORT', '42471')), timeout=5)
        self.node = rclpy.create_node('catora_open_field_app', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.platforms, self.modes, self.acks = {}, {}, {}
        self.controllers = {}
        self.local_poses = {}
        self.trajectory_mode_claimed = False
        self.mode_lost_at = None
        self.reference_counts = {}
        self.mode_clients = {}
        self.status = None
        self.task_id = None
        self.phase = 'initializing'
        self.children = []
        self.subscriptions = []
        self.clients = []
        self.trajectory = None
        self.bridge_state = {}
        self.bridge_fragments = {}
        self.link_metrics = {}
        self.input_state = {}
        self.clock_samples = []
        self.setpoint_samples = []
        self.setpoints = None
        self.collision_baseline = {}
        self.min_separation = float('inf')
        self.min_truth_separation = float('inf')
        self.max_localization_disagreement = 0.0
        self.max_tracking_error = 0.0
        self.next_collision_check = 0.0
        self.samples = (self.output/'flight.jsonl').open('w')
        for vehicle in self.vehicles:
            self.reference_counts[vehicle] = 0
            topic=self.pose_topic(vehicle)
            self.deployment[vehicle]['topics']['pose']=topic
            self.subscriptions.append(self.node.create_subscription(PoseStamped,topic,
                lambda msg,v=vehicle:self.on_local_pose(v,msg),qos_profile_sensor_data))
            self.mode_clients[vehicle] = self.node.create_client(SetControlMode, f'/{vehicle}/controller/set_control_mode')
            self.subscriptions.append(self.node.create_subscription(ControllerInfo, f'/{vehicle}/controller/info',
                lambda msg,v=vehicle:self.controllers.__setitem__(v,msg), qos_profile_sensor_data))
            self.subscriptions.append(self.node.create_subscription(TrajectorySetpoints, f'/{vehicle}/motion_reference/trajectory',
                lambda msg,v=vehicle:self.on_reference(v,msg), qos_profile_sensor_data))
            for suffix, kind, store in (
                ('platform/info', PlatformInfo, self.platforms),
                ('fmu/out/vehicle_control_mode', VehicleControlMode, self.modes)):
                self.subscriptions.append(self.node.create_subscription(
                    kind, f'/{vehicle}/{suffix}', lambda msg,v=vehicle,s=store: s.__setitem__(v,msg), qos_profile_sensor_data))
            self.subscriptions.append(self.node.create_subscription(VehicleCommandAck, f'/{vehicle}/fmu/out/vehicle_command_ack_v{VehicleCommandAck.MESSAGE_VERSION}',
                lambda msg,v=vehicle: self.acks.__setitem__((v,msg.command),msg.result), qos_profile_sensor_data))
        self.subscriptions.append(self.node.create_subscription(String, '/planner/as2_bridge_state', self.on_bridge_state,10))
        self.subscriptions.append(self.node.create_subscription(String,'/planner/catora_input_state',self.on_input_state,10))
        self.subscriptions.append(self.node.create_subscription(BehaviorResult, '/planner/status', self.on_status,10))
        self.subscriptions.append(self.node.create_subscription(TimedTrajectorySegment, '/planner/trajectory', self.on_trajectory,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.subscriptions.append(self.node.create_subscription(FleetSetpoint, '/planner/setpoints', self.on_setpoint,1))
        self.subscriptions.append(self.node.create_subscription(Clock,'/clock',lambda msg:self.clock_samples.append((time.monotonic(),msg.clock.sec+msg.clock.nanosec/1e9)),qos_profile_sensor_data))
        self.node.create_timer(.1,self.record)

    def pose_topic(self, vehicle):
        return f'/{vehicle}/catora/local_pose'

    def ping_topic(self, vehicle):
        return f'/{vehicle}/catora/ping'

    def on_setpoint(self, message):
        self.setpoints=message
        stamp=message.sampled_at.sec+message.sampled_at.nanosec/1e9
        self.setpoint_samples.append((time.monotonic(),stamp,self.node.get_clock().now().nanoseconds/1e9))

    def on_bridge_state(self, message):
        state=json.loads(message.data)
        if state['task_id']!=self.task_id:return
        if os.environ.get('CATORA_MIXED')=='1':
            for vehicle in state['mode_ready']:self.bridge_fragments[vehicle]=state
            self.bridge_state={'task_id':self.task_id,'prepared':len(self.bridge_fragments)==len(self.vehicles) and all(s['prepared'] for s in self.bridge_fragments.values()),
                'stop_requested':any(s.get('stop_requested',False) for s in self.bridge_fragments.values()),
                'receipt_by_vehicle':{v:s.get('receipt',{}) for v,s in self.bridge_fragments.items()},
                'compute_by_vehicle':{v:s.get('compute_id','') for v,s in self.bridge_fragments.items()}}
            for key in ('mode_ready','published','stop_confirmed','stop_errors'):
                self.bridge_state[key]={v:value for s in self.bridge_fragments.values() for v,value in s.get(key,{}).items()}
        else:self.bridge_state=state
        self.samples.write(json.dumps({'sim_time':self.node.get_clock().now().nanoseconds/1e9,'bridge_state':state})+'\n')

    def on_input_state(self,message):
        state=json.loads(message.data)
        for key,value in state.items():self.input_state.setdefault(key,{}).update(value)

    def on_local_pose(self, vehicle, message):
        if message.header.frame_id != vehicle+'/odom':
            raise RuntimeError('unexpected local odometry frame for '+vehicle+': '+message.header.frame_id)
        self.local_poses[vehicle]=message

    def on_reference(self, vehicle, message):
        if self.phase != 'planning': return
        if message.header.frame_id != vehicle+'/odom' or len(message.setpoints)!=1:
            raise RuntimeError('invalid AS2 trajectory reference for '+vehicle)
        point=message.setpoints[0]
        if not all(math.isfinite(value) for value in (point.position.x,point.position.y,point.position.z,point.yaw_angle)):
            raise RuntimeError('nonfinite AS2 trajectory reference for '+vehicle)
        self.reference_counts[vehicle]+=1

    def controller_mode(self, mode):
        self.wait(lambda:all(client.service_is_ready() for client in self.mode_clients.values()))
        futures=[]
        for client in self.mode_clients.values():
            request=SetControlMode.Request()
            request.control_mode.control_mode=mode
            request.control_mode.yaw_mode=ControlMode.YAW_ANGLE
            futures.append(client.call_async(request))
        self.wait(lambda:all(future.done() for future in futures),10)
        if not all(future.result().success for future in futures):
            raise RuntimeError('controller mode request failed')
        expected={mode} if mode!=ControlMode.HOVER else {ControlMode.HOVER,ControlMode.POSITION}
        self.wait(lambda:all(v in self.controllers and self.controllers[v].input_control_mode.control_mode in expected
                            for v in self.vehicles),5)
        self.trajectory_mode_claimed=mode==ControlMode.TRAJECTORY
        print('All AS2 controllers switched to mode',mode,flush=True)

    def snapshot_processes(self):
        if not os.environ.get('CATORA_APP_ROOT'): return
        processes = []
        expected = ('CATORA_APP_ROOT='+str(self.root)).encode()
        for path in Path('/proc').glob('[0-9]*'):
            try:
                if expected not in (path/'environ').read_bytes().split(b'\0'): continue
                fields = dict(line.split(':',1) for line in (path/'status').read_text().splitlines() if ':' in line)
                processes.append({'pid':int(path.name), 'command':(path/'cmdline').read_bytes().replace(b'\0',b' ').decode(),
                                  'rss_kib':int(fields.get('VmRSS','0 kB').split()[0])})
            except (OSError, ValueError): continue
        (self.output/'processes.json').write_text(json.dumps({'machine':os.uname().machine,'processes':processes},indent=2)+'\n')

    def on_status(self,message):
        if self.task_id and message.task_id==self.task_id:
            self.status=message

    def on_trajectory(self,message):
        self.trajectory=message

    def start(self,name,command):
        stream=(self.output/(name+'.log')).open('w')
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.children.append((name,process,stream))

    def wait(self,predicate,timeout=45):
        deadline=time.monotonic()+timeout
        while not predicate():
            for name,process,_ in self.children:
                if self.phase in ('emergency_stop','emergency_landing') and name!='as2':continue
                if process.poll() is not None: raise RuntimeError(f'{name} exited: {process.returncode}')
            if time.monotonic()>deadline: raise TimeoutError(f'timed out during {self.phase}')
            self.executor.spin_once(timeout_sec=.05)

    def fleet_positions(self):
        result={}
        for vehicle,msg in self.local_poses.items():
            p=msg.pose.position
            origin=self.deployment[vehicle]['px4_origin_in_fleet_ned']
            result[vehicle]=[origin[0]+p.y,origin[1]+p.x,origin[2]-p.z]
        return result

    def truth_positions(self):
        result={}
        for index,vehicle in enumerate(self.vehicles):
            p=self.rpc.call('simGetVehiclePose',f'Drone{index}')['position']
            result[vehicle]=[self.spawns[vehicle][i]+p[axis] for i,axis in enumerate(('x_val','y_val','z_val'))]
        return result

    def align_origins(self):
        truth=self.truth_positions()
        for vehicle,actual in truth.items():
            p=self.local_poses[vehicle].pose.position
            # fleet NED = local NED + fixed origin. GPS-based AS2 local z may be nonzero on the ground.
            self.deployment[vehicle]['px4_origin_in_fleet_ned']=[actual[0]-p.y,actual[1]-p.x,actual[2]+p.z]
        (self.output/'deployment.yaml').write_text(yaml.safe_dump(self.deployment,sort_keys=False))
        print('Aligned fleet origins from stationary AirSim/AS2 observations',flush=True)

    def record(self):
        positions=self.fleet_positions()
        if len(positions)!=len(self.vehicles):return
        if self.phase=='planning':
            lost=self.trajectory_mode_claimed and self.status and self.status.overall_status==BehaviorResult.RUNNING and not self.bridge_state.get('stop_requested',False) and any(self.controllers[v].input_control_mode.control_mode!=ControlMode.TRAJECTORY for v in self.vehicles)
            if lost:
                if self.mode_lost_at is None:self.mode_lost_at=time.monotonic()
                # Status and controller feedback use different DDS topics; allow their
                # callbacks to catch up before classifying a terminal HOVER as failure.
                if time.monotonic()-self.mode_lost_at>.3:raise RuntimeError('AS2 trajectory mode lost during planning')
            else:self.mode_lost_at=None
            if time.monotonic()>=self.next_collision_check:
                self.collision_check()
                truth=self.truth_positions()
                for a,b in itertools.combinations(truth.values(),2):self.min_truth_separation=min(self.min_truth_separation,math.dist(a,b))
                for vehicle in self.vehicles:self.max_localization_disagreement=max(self.max_localization_disagreement,math.dist(truth[vehicle],positions[vehicle]))
                self.samples.write(json.dumps({'sim_time':self.node.get_clock().now().nanoseconds/1e9,'phase':self.phase,'airsim_truth_ned':truth})+'\n')
                self.next_collision_check=time.monotonic()+1.0
            for a,b in itertools.combinations(positions.values(),2):self.min_separation=min(self.min_separation,math.dist(a,b))
            if self.setpoints is not None:
                for item in self.setpoints.setpoints:
                    o=self.deployment[item.vehicle_id]['px4_origin_in_fleet_ned'];p=item.position
                    desired=[o[0]+p.x,o[1]+p.y,o[2]+p.z]
                    self.max_tracking_error=max(self.max_tracking_error,math.dist(positions[item.vehicle_id],desired))
        self.samples.write(json.dumps({'sim_time':self.node.get_clock().now().nanoseconds/1e9,'unix_time':time.time(),'phase':self.phase,
            'trajectory_mode_claimed':self.trajectory_mode_claimed,'positions_ned':positions,
            'desired_positions_ned':{item.vehicle_id:[self.deployment[item.vehicle_id]['px4_origin_in_fleet_ned'][i]+value for i,value in enumerate((item.position.x,item.position.y,item.position.z))] for item in self.setpoints.setpoints} if self.setpoints is not None else {}})+'\n')

    def service(self,vehicle,name,value,command,feedback):
        print(self.phase,vehicle,name,flush=True)
        client=self.node.create_client(SetBool,f'/{vehicle}/{name}')
        self.clients.append(client)
        self.wait(client.service_is_ready)
        self.acks.pop((vehicle,command),None)
        future=client.call_async(SetBool.Request(data=value))
        self.wait(future.done)
        if not future.result().success:raise RuntimeError(f'{vehicle} rejected {name}')
        self.wait(lambda:(vehicle,command) in self.acks)
        if self.acks[(vehicle,command)]!=VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError(f'{vehicle} PX4 rejected {name}: {self.acks[(vehicle,command)]}')
        self.wait(feedback)

    def action(self,vehicle,kind,name,goal,success):
        print(self.phase,vehicle,name,flush=True)
        client=ActionClient(self.node,kind,f'/{vehicle}/{name}')
        self.clients.append(client)
        self.wait(client.server_is_ready)
        future=client.send_goal_async(goal)
        self.wait(future.done)
        handle=future.result()
        if not handle.accepted:raise RuntimeError(f'{vehicle} rejected {name}')
        result=handle.get_result_async()
        try:self.wait(result.done,90)
        except Exception:
            handle.cancel_goal_async()
            raise
        if not getattr(result.result().result,success):raise RuntimeError(f'{vehicle} failed {name}')

    def screenshot(self,name):
        response=self.rpc.call('simGetImages',[{'camera_name':'survey','image_type':0,'pixels_as_float':False,'compress':True}],'',True)[0]
        (self.output/(name+'.png')).write_bytes(bytes(response['image_data_uint8']))

    def collision_check(self):
        for index,vehicle in enumerate(self.vehicles):
            msg=self.rpc.call('simGetCollisionInfo',f'Drone{index}')
            if msg['has_collided'] and msg['time_stamp']>self.collision_baseline[vehicle]:
                raise RuntimeError(f'{vehicle} AirSim collision: {msg["object_name"]}')

    def run(self):
        if os.environ.get('CATORA_METRICS_SCRIPT'):
            self.start('metrics',['python3',os.environ['CATORA_METRICS_SCRIPT']])
        self.start('as2',['python3',os.environ.get('AS2_LAUNCHER', '/opt/companion_as2/run.py')])
        self.start('input-adapter',['ros2','run','uav_cat_ora_adapter','cat_ora_state_adapter_node','--ros-args',
            '-p','deployment_config:='+os.environ.get('CATORA_INPUT_DEPLOYMENT',str(self.root/'deployment.yaml')),'-p','use_sim_time:=true'])
        self.start('cat-ora',['ros2','run','mrs_formation_reshaping','cat_ora_node','--ros-args','-p','use_sim_time:=true'])
        self.start('proxy',['ros2','run','mrs_formation_reshaping','ros2_proxy.py','--ros-args','-p','use_sim_time:=true'])
        self.phase='waiting_for_localization'
        self.wait(lambda:len(self.local_poses)==len(self.platforms)==len(self.modes)==len(self.vehicles),90)
        self.screenshot('ground')
        settle=time.monotonic()+15
        self.wait(lambda:time.monotonic()>=settle)
        self.align_origins()
        if os.environ.get('CATORA_MIXED')=='1':self.measure_links()
        if os.environ.get('CATORA_PREFLIGHT_ONLY')=='1':
            self.phase='clock_probe'
            self.clock_samples.clear()
            started=time.monotonic()
            self.wait(lambda:time.monotonic()-started>=12,20)
            samples=self.clock_samples
            gaps=[b[0]-a[0] for a,b in zip(samples,samples[1:])]
            steps=[b[1]-a[1] for a,b in zip(samples,samples[1:]) if b[1]!=a[1]]
            result={'samples':len(samples),'wall_seconds':time.monotonic()-started,
                    'receipt_hz':len(samples)/(time.monotonic()-started),
                    'max_wall_gap':max(gaps,default=0),'sim_updates':len(steps),
                    'max_sim_step':max(steps,default=0),'sim_steps':steps[:50]}
            (self.output/'clock-probe.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result),flush=True)
        if os.environ.get('CATORA_PREFLIGHT_ONLY')!='1':
            self.phase='takeoff'
            for vehicle in self.vehicles:
                ground_z=self.local_poses[vehicle].pose.position.z
                self.service(vehicle,'set_arming_state',True,400,lambda v=vehicle:self.modes[v].flag_armed)
                self.service(vehicle,'set_offboard_mode',True,176,lambda v=vehicle:self.modes[v].flag_control_offboard_enabled)
                self.action(vehicle,Takeoff,'TakeoffBehavior',Takeoff.Goal(takeoff_height=3.0,takeoff_speed=.6),'takeoff_success')
                self.wait(lambda v=vehicle,z=ground_z:self.platforms[v].status.state==PlatformStatus.FLYING and self.local_poses[v].pose.position.z-z>2.5)
            self.screenshot('airborne')
        for index,vehicle in enumerate(self.vehicles):self.collision_baseline[vehicle]=self.rpc.call('simGetCollisionInfo',f'Drone{index}')['time_stamp']
        planner_command=['ros2','run','uav_planner_framework','planner_framework_node','--ros-args',
            '-p','runtime_startup_timeout:='+os.environ.get('PLANNER_STARTUP_TIMEOUT', '0.0'),
            '-p','registry_root:='+os.environ.get('PLANNER_REGISTRY', '/opt/catora_app_ws/src/planner_framework/registry'),'-p','resolved_dir:='+str(self.output/'resolved'),'-p','use_sim_time:=true']
        if os.environ.get('CATORA_LOCAL_VEHICLE'):planner_command+=['-p','output_vehicle_id:='+os.environ['CATORA_LOCAL_VEHICLE']]
        self.start('planner',planner_command)
        client=self.node.create_client(ControlPlanner,'/planner/control');self.clients.append(client)
        self.wait(client.service_is_ready)
        self.phase='preflight_pipeline' if os.environ.get('CATORA_PREFLIGHT_ONLY')=='1' else 'planning'
        begin=time.monotonic()
        clock_deadline=time.monotonic()+20
        while True:
            future=client.call_async(ControlPlanner.Request(command=ControlPlanner.Request.START,
                task_config_path=str(self.root/'task.yaml'),deployment_config_path=str(self.output/'deployment.yaml')))
            self.wait(future.done)
            response=future.result()
            if response.accepted: break
            if response.message!='CLOCK_NOT_READY' or time.monotonic()>=clock_deadline:
                raise RuntimeError(response.message)
            print('Waiting for Planner first /clock sample',flush=True)
            settle=time.monotonic()+.2
            self.wait(lambda:time.monotonic()>=settle)
        self.task_id=response.result.task_id
        self.status=response.result
        # Wait for a real algorithm output and the output bridge before taking control.
        self.wait(lambda:self.trajectory is not None and self.setpoints is not None and self.bridge_state.get('prepared',False) and
                  all(any(info.node_name.startswith(('cat_ora_as2_','swarm_as2_')) for info in self.node.get_publishers_info_by_topic('/'+v+'/motion_reference/trajectory')) for v in self.vehicles),35)
        if os.environ.get('CATORA_PREFLIGHT_ONLY')=='1':
            self.setpoint_samples.clear()
            self.clock_samples.clear()
            started=time.monotonic()
            # Leave enough overlap for an independently discovered timing observer.
            self.wait(lambda:time.monotonic()-started>=20,25)
            samples=self.setpoint_samples
            ages=[s[2]-s[1] for s in samples]
            result={'wall_seconds':time.monotonic()-started,'setpoint_receipt_hz':len(samples)/(time.monotonic()-started),
                    'clock_receipt_hz':len(self.clock_samples)/(time.monotonic()-started),
                    'max_sample_age':max(ages,default=0),'min_sample_age':min(ages,default=0),
                    'bridge_state':self.bridge_state}
            (self.output/'pipeline-probe.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result),flush=True)
            return
        self.snapshot_processes()
        self.controller_mode(ControlMode.TRAJECTORY)
        scenario=os.environ.get('CATORA_SCENARIO','normal')
        if scenario!='normal':
            self.run_stop_scenario(client,scenario)
            return
        wall_limit=float(os.environ.get('PLANNER_APP_WALL_TIMEOUT','95'))
        while self.status.overall_status not in (BehaviorResult.SUCCEEDED,BehaviorResult.FAILED,BehaviorResult.CANCELED):
            self.wait(lambda:self.status.overall_status in (BehaviorResult.SUCCEEDED,BehaviorResult.FAILED,BehaviorResult.CANCELED) or time.monotonic()-begin>wall_limit,wall_limit+5)
            self.collision_check()
            if time.monotonic()-begin>wall_limit:raise TimeoutError('planner application timed out')
        if self.status.overall_status!=BehaviorResult.SUCCEEDED:raise RuntimeError(f'planner failed: {self.status.failure_reason}')
        if not all(self.reference_counts.values()):
            raise RuntimeError('not all AS2 reference streams were observed')
        self.trajectory_mode_claimed=False
        self.phase='holding_goal'
        self.wait_as2_stop()
        errors={v:math.dist(p,[self.task['targets'][v][axis] for axis in ('x','y','z')]) for v,p in self.fleet_positions().items()}
        truth_errors={v:math.dist(p,[self.task['targets'][v][axis] for axis in ('x','y','z')]) for v,p in self.truth_positions().items()}
        if max(errors.values())>.6:raise RuntimeError(f'endpoint error: {errors}')
        if max(truth_errors.values())>.75:raise RuntimeError(f'AirSim truth endpoint error: {truth_errors}')
        if self.min_separation<2.0:raise RuntimeError(f'insufficient observed separation: {self.min_separation}')
        if self.min_truth_separation<2.0:raise RuntimeError(f'insufficient AirSim truth separation: {self.min_truth_separation}')
        self.screenshot('goals')
        result={'task_id':self.task_id,'planner_status':'SUCCEEDED','endpoint_error_m':errors,
            'airsim_truth_endpoint_error_m':truth_errors,'min_truth_separation_m':self.min_truth_separation,
            'max_localization_disagreement_m':self.max_localization_disagreement,
            'min_observed_separation_m':self.min_separation,'max_sampled_tracking_error_m':self.max_tracking_error,
            'planner_wall_seconds':time.monotonic()-begin,'planning_collisions':False,
            'as2_trajectory_reference_counts':self.reference_counts,
            'position_source':os.environ.get('PLANNER_STATE_SOURCE','CAT-ORA input adapter checked AS2 local odometry; same frame as trajectory execution'),
            'runtime_topology':os.environ.get('CATORA_TOPOLOGY', 'PC containers'),
            'execution_host':os.uname().nodename,'execution_arch':os.uname().machine}
        if os.environ.get('CATORA_MIXED')=='1':
            result['mixed_deployment']=True
            result['communication_rtt']=self.link_metrics
            result['per_vehicle_execution']=self.bridge_state
        self.phase='landing'
        for vehicle in self.vehicles:
            self.action(vehicle,Land,'LandBehavior',Land.Goal(land_speed=.5),'land_success')
            self.wait(lambda v=vehicle:not self.modes[v].flag_armed)
        result['landed_and_disarmed']=True
        self.screenshot('landed')
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)

    def wait_as2_stop(self):
        def done():
            if self.bridge_state.get('stop_errors'):
                raise RuntimeError('AS2 stop failed: '+str(self.bridge_state['stop_errors']))
            return all(self.bridge_state.get('stop_confirmed',{}).get(v,False) for v in self.vehicles)
        self.wait(done,12)
        print('AS2 HOVER confirmed for all vehicles',flush=True)

    def measure_links(self):
        import statistics
        self.phase='communication_probe'
        for vehicle in self.vehicles:
            client=self.node.create_client(Trigger,self.ping_topic(vehicle));self.clients.append(client)
            started=time.monotonic();self.wait(client.service_is_ready,20)
            discovery=time.monotonic()-started;values=[]
            for _ in range(20):
                started=time.monotonic();future=client.call_async(Trigger.Request());self.wait(future.done,3)
                if not future.result().success:raise RuntimeError('input adapter not ready: '+vehicle)
                values.append((time.monotonic()-started)*1000)
            ordered=sorted(values)
            self.link_metrics[vehicle]={'samples':len(values),'discovery_seconds':discovery,'min_ms':min(values),'median_ms':statistics.median(values),'p95_ms':ordered[18],'max_ms':max(values)}
        (self.output/'communication-rtt.json').write_text(json.dumps(self.link_metrics,indent=2)+'\n')
        print('Compute-node ROS service round trips verified',flush=True)

    def run_stop_scenario(self,client,scenario):
        self.wait(lambda:all(n>=3 for n in self.reference_counts.values()),5)
        self.phase='stop_validation'
        self.trajectory_mode_claimed=False
        stopped_pid=None
        started=time.monotonic()
        try:
            if scenario=='stream-loss':
                tag=('CATORA_APP_ROOT='+str(self.root)).encode()
                matches=[]
                for p in Path('/proc').glob('[0-9]*'):
                    try:
                        if tag in (p/'environ').read_bytes().split(b'\0') and b'/lib/uav_cat_ora_adapter/cat_ora_as2_adapter_node' in (p/'cmdline').read_bytes():matches.append(int(p.name))
                    except OSError:pass
                if len(matches)!=1:raise RuntimeError('ambiguous output process: '+str(matches))
                stopped_pid=matches[0];os.kill(stopped_pid,signal.SIGSTOP)
                self.wait(lambda:all(self.controllers[v].input_control_mode.control_mode in (ControlMode.HOVER,ControlMode.POSITION) for v in self.vehicles),4)
                self.loss_observed_seconds=time.monotonic()-started
                # Exercise the same stop-before-cleanup path as an unexpected failure.
                raise VerifiedStreamLoss('output paused; AS2 stream-loss HOVER observed')
            future=client.call_async(ControlPlanner.Request(command=ControlPlanner.Request.CANCEL))
            self.wait(future.done,5)
            if not future.result().accepted:raise RuntimeError('Planner rejected cancel')
        finally:
            if stopped_pid is not None:os.kill(stopped_pid,signal.SIGCONT)
        self.wait(lambda:self.status.overall_status==BehaviorResult.CANCELED,5)
        self.wait_as2_stop()
        stop_seconds=time.monotonic()-started
        self.collision_check()
        self.screenshot('stopped')
        self.phase='landing'
        for vehicle in self.vehicles:
            self.action(vehicle,Land,'LandBehavior',Land.Goal(land_speed=.5),'land_success')
            self.wait(lambda v=vehicle:not self.modes[v].flag_armed)
        self.screenshot('landed')
        result={'scenario':scenario,'planner_status':'CANCELED','as2_hover_confirmed':True,
                'stop_wall_seconds':stop_seconds,'stop_state':self.bridge_state,
                'landed_and_disarmed':True,'execution_arch':os.uname().machine,
                'as2_trajectory_reference_counts':self.reference_counts}
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)

    def stop_after_failure(self):
        # This runs before terminating any planner/adapter/AS2 process.
        report={'requested':True,'hover_confirmed':False,'landed_and_disarmed':False}
        self.trajectory_mode_claimed=False
        self.phase='emergency_stop'
        try:
            self.controller_mode(ControlMode.HOVER)
            report['hover_confirmed']=True
            for client in self.clients:
                if getattr(client,'srv_name','')=='/planner/control' and client.service_is_ready():
                    try:
                        future=client.call_async(ControlPlanner.Request(command=ControlPlanner.Request.CANCEL))
                        self.wait(future.done,3)
                        report['planner_cancel_accepted']=future.result().accepted
                    except Exception as exc:report['planner_cancel_error']=str(exc)
            self.phase='emergency_landing'
            for vehicle in self.vehicles:
                if vehicle in self.modes and self.modes[vehicle].flag_armed:
                    self.action(vehicle,Land,'LandBehavior',Land.Goal(land_speed=.5),'land_success')
                    self.wait(lambda v=vehicle:not self.modes[v].flag_armed)
            report['landed_and_disarmed']=all(v in self.modes and not self.modes[v].flag_armed for v in self.vehicles)
        except Exception as exc:report['error']=str(exc)
        (self.output/'failure-stop.json').write_text(json.dumps(report,indent=2)+'\n')
        return report

    def close(self):
        for _,process,_ in reversed(self.children):
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        for _,process,stream in reversed(self.children):
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
            # ros2 run may exit before a child that handles SIGTERM itself.
            try: os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError: pass
            stream.close()
        self.samples.close()
        self.executor.shutdown(timeout_sec=2)
        self.node.destroy_node()


def main():
    rclpy.init()
    mission=Mission()
    def interrupted(*_):
        raise InterruptedError('mission interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    try:mission.run()
    except Exception as error:
        expected=isinstance(error,VerifiedStreamLoss)
        (mission.output/('fault-event.json' if expected else 'failure.json')).write_text(json.dumps({'phase':mission.phase,'error':str(error),'reference_counts':mission.reference_counts,
            'trajectory_received':mission.trajectory is not None,'setpoints_received':mission.setpoints is not None,
            'planner_status':None if mission.status is None else mission.status.overall_status,
            'controller_modes':{v:m.input_control_mode.control_mode for v,m in mission.controllers.items()},
            'local_frames':{v:m.header.frame_id for v,m in mission.local_poses.items()},'bridge_state':mission.bridge_state,'input_state':mission.input_state},indent=2)+'\n')
        report=mission.stop_after_failure() if any(m.flag_armed for m in mission.modes.values()) else {}
        if expected and report.get('hover_confirmed') and report.get('landed_and_disarmed') and report.get('planner_cancel_accepted'):
            result={'scenario':'stream-loss','planner_status':'CANCELED','as2_hover_confirmed':True,
                    'failsafe_observed_seconds':mission.loss_observed_seconds,'failure_stop':report,
                    'landed_and_disarmed':True,'execution_arch':os.uname().machine}
            (mission.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result),flush=True)
        else:raise
    finally:
        mission.close()
        rclpy.try_shutdown()


if __name__=='__main__':main()
