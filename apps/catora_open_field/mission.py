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
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import Takeoff, Land
from as2_msgs.msg import PlatformInfo, PlatformStatus
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommandAck, VehicleControlMode
from std_srvs.srv import SetBool
from uav_planner_interfaces.msg import BehaviorResult, FleetSetpoint, TimedTrajectorySegment
from uav_planner_interfaces.srv import ControlPlanner
import yaml
from airsim_rpc import Rpc


class Mission:
    def __init__(self):
        self.root = Path('/opt/catora_app')
        self.output = Path('/results')
        self.output.mkdir(parents=True, exist_ok=True)
        self.task = yaml.safe_load((self.root/'task.yaml').read_text())
        self.deployment = yaml.safe_load((self.root/'deployment.yaml').read_text())
        self.spawns = {v:list(d['px4_origin_in_fleet_ned']) for v,d in self.deployment.items()}
        self.vehicles = list(self.deployment)
        self.rpc = Rpc('127.0.0.1', 42471, timeout=5)
        self.node = rclpy.create_node('catora_open_field_app', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.poses, self.platforms, self.modes, self.acks = {}, {}, {}, {}
        self.status = None
        self.task_id = None
        self.phase = 'initializing'
        self.children = []
        self.subscriptions = []
        self.clients = []
        self.trajectory = None
        self.setpoints = None
        self.collision_baseline = {}
        self.min_separation = float('inf')
        self.min_truth_separation = float('inf')
        self.max_localization_disagreement = 0.0
        self.max_tracking_error = 0.0
        self.next_collision_check = 0.0
        self.samples = (self.output/'flight.jsonl').open('w')
        for vehicle in self.vehicles:
            for suffix, kind, store in (
                ('self_localization/pose', PoseStamped, self.poses),
                ('platform/info', PlatformInfo, self.platforms),
                ('fmu/out/vehicle_control_mode', VehicleControlMode, self.modes)):
                self.subscriptions.append(self.node.create_subscription(
                    kind, f'/{vehicle}/{suffix}', lambda msg,v=vehicle,s=store: s.__setitem__(v,msg), qos_profile_sensor_data))
            self.subscriptions.append(self.node.create_subscription(VehicleCommandAck, f'/{vehicle}/fmu/out/vehicle_command_ack_v{VehicleCommandAck.MESSAGE_VERSION}',
                lambda msg,v=vehicle: self.acks.__setitem__((v,msg.command),msg.result), qos_profile_sensor_data))
        self.subscriptions.append(self.node.create_subscription(BehaviorResult, '/planner/status', self.on_status,10))
        self.subscriptions.append(self.node.create_subscription(TimedTrajectorySegment, '/planner/trajectory', self.on_trajectory,10))
        self.subscriptions.append(self.node.create_subscription(FleetSetpoint, '/planner/setpoints', lambda msg: setattr(self,'setpoints',msg),10))
        self.node.create_timer(.1,self.record)

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
                if process.poll() is not None: raise RuntimeError(f'{name} exited: {process.returncode}')
            if time.monotonic()>deadline: raise TimeoutError(f'timed out during {self.phase}')
            rclpy.spin_once(self.node,timeout_sec=.05)

    def fleet_positions(self):
        result={}
        for vehicle,msg in self.poses.items():
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
            p=self.poses[vehicle].pose.position
            # fleet NED = local NED + fixed origin. GPS-based AS2 local z may be nonzero on the ground.
            self.deployment[vehicle]['px4_origin_in_fleet_ned']=[actual[0]-p.y,actual[1]-p.x,actual[2]+p.z]
        (self.output/'deployment.yaml').write_text(yaml.safe_dump(self.deployment,sort_keys=False))
        print('Aligned fleet origins from stationary AirSim/AS2 observations',flush=True)

    def record(self):
        positions=self.fleet_positions()
        if len(positions)!=4:return
        if self.phase=='planning':
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
        self.samples.write(json.dumps({'sim_time':self.node.get_clock().now().nanoseconds/1e9,'phase':self.phase,'positions_ned':positions})+'\n')

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
        self.start('as2',['python3','/opt/companion_as2/run.py'])
        self.start('cat-ora',['ros2','run','mrs_formation_reshaping','cat_ora_node','--ros-args','-p','use_sim_time:=true'])
        self.start('proxy',['ros2','run','mrs_formation_reshaping','ros2_proxy.py','--ros-args','-p','use_sim_time:=true'])
        self.phase='waiting_for_localization'
        self.wait(lambda:len(self.poses)==len(self.platforms)==len(self.modes)==4,90)
        settle=time.monotonic()+15
        self.wait(lambda:time.monotonic()>=settle)
        self.align_origins()
        self.screenshot('ground')
        self.phase='takeoff'
        for vehicle in self.vehicles:
            self.service(vehicle,'set_arming_state',True,400,lambda v=vehicle:self.modes[v].flag_armed)
            self.service(vehicle,'set_offboard_mode',True,176,lambda v=vehicle:self.modes[v].flag_control_offboard_enabled)
            self.action(vehicle,Takeoff,'TakeoffBehavior',Takeoff.Goal(takeoff_height=3.0,takeoff_speed=.6),'takeoff_success')
            self.wait(lambda v=vehicle:self.platforms[v].status.state==PlatformStatus.FLYING and self.poses[v].pose.position.z>2.5)
        self.screenshot('airborne')
        for index,vehicle in enumerate(self.vehicles):self.collision_baseline[vehicle]=self.rpc.call('simGetCollisionInfo',f'Drone{index}')['time_stamp']
        self.start('planner',['ros2','run','uav_planner_framework','planner_framework_node','--ros-args',
            '-p','registry_root:=/opt/catora_app_ws/src/planner_framework/registry','-p','resolved_dir:=/results/resolved','-p','use_sim_time:=true'])
        client=self.node.create_client(ControlPlanner,'/planner/control');self.clients.append(client)
        self.wait(client.service_is_ready)
        self.phase='planning'
        begin=time.monotonic()
        future=client.call_async(ControlPlanner.Request(command=ControlPlanner.Request.START,
            task_config_path=str(self.root/'task.yaml'),deployment_config_path=str(self.output/'deployment.yaml')))
        self.wait(future.done)
        response=future.result()
        if not response.accepted:raise RuntimeError(response.message)
        self.task_id=response.result.task_id
        self.status=response.result
        while self.status.overall_status not in (BehaviorResult.SUCCEEDED,BehaviorResult.FAILED,BehaviorResult.CANCELED):
            self.wait(lambda:self.status.overall_status in (BehaviorResult.SUCCEEDED,BehaviorResult.FAILED,BehaviorResult.CANCELED) or time.monotonic()-begin>95,100)
            self.collision_check()
            if time.monotonic()-begin>95:raise TimeoutError('planner application timed out')
        if self.status.overall_status!=BehaviorResult.SUCCEEDED:raise RuntimeError(f'planner failed: {self.status.failure_reason}')
        errors={v:math.dist(p,[self.task['targets'][v][axis] for axis in ('x','y','z')]) for v,p in self.fleet_positions().items()}
        truth_errors={v:math.dist(p,[self.task['targets'][v][axis] for axis in ('x','y','z')]) for v,p in self.truth_positions().items()}
        if max(errors.values())>.6:raise RuntimeError(f'endpoint error: {errors}')
        if max(truth_errors.values())>.75:raise RuntimeError(f'AirSim truth endpoint error: {truth_errors}')
        if self.min_separation<2.0:raise RuntimeError(f'insufficient observed separation: {self.min_separation}')
        self.screenshot('goals')
        result={'task_id':self.task_id,'planner_status':'SUCCEEDED','endpoint_error_m':errors,
            'airsim_truth_endpoint_error_m':truth_errors,'min_truth_separation_m':self.min_truth_separation,
            'max_localization_disagreement_m':self.max_localization_disagreement,
            'min_observed_separation_m':self.min_separation,'max_sampled_tracking_error_m':self.max_tracking_error,
            'planner_wall_seconds':time.monotonic()-begin,'planning_collisions':False}
        self.phase='landing'
        for vehicle in self.vehicles:
            self.action(vehicle,Land,'LandBehavior',Land.Goal(land_speed=.5),'land_success')
            self.wait(lambda v=vehicle:not self.modes[v].flag_armed)
        result['landed_and_disarmed']=True
        self.screenshot('landed')
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)

    def close(self):
        for _,process,_ in reversed(self.children):
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        for _,process,stream in reversed(self.children):
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
            stream.close()
        self.samples.close()
        self.node.destroy_node()


def main():
    rclpy.init()
    mission=Mission()
    try:mission.run()
    except Exception as error:
        (mission.output/'failure.json').write_text(json.dumps({'phase':mission.phase,'error':str(error)},indent=2)+'\n')
        raise
    finally:
        mission.close()
        rclpy.try_shutdown()


if __name__=='__main__':main()
