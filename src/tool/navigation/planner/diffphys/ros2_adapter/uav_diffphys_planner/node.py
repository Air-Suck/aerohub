"""ROS policy orchestration; only the Behavior publishes Motion Reference."""
import copy
from collections import deque
import json
import math
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped,TwistStamped
from std_msgs.msg import Bool,String
from uav_follow_reference_acro.msg import ControlFrame,ControlSession
from uav_inference_interfaces.srv import InferDiffphys
from .core import observation,decode
from .camera import sample_high_speed

PROFILES = {
    'swarm': dict(max_speed=2.5, gravity=9.80665,
                  version='sha256:72e0ec5ad1c7a2163ce10506c4210bc9439fab8e6b0cb83114002bedd5582628;',
                  heading_clipped=False, vertical_correction=.01, margin=.15),
    'high_speed': dict(max_speed=13., gravity=9.81,
                       version='sha256:dd9ffeafd12e44dfd0e402fb880b52d110f414530faddb16e6c9b3dbd336bb64;high-speed-hfov91-crop82-nearest36x48-pool3-state10-explicit-h-v2',
                       heading_clipped=True, vertical_correction=0., margin=.2),
}

def stamp(h):return h.stamp.sec+h.stamp.nanosec/1e9

def vec(v):return np.array([v.x,v.y,v.z])

def rot(q):return Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()


class Planner(Node):
    def __init__(self):
        super().__init__('diffphys_planner')
        self.declare_parameter('use_sim_time',True) if not self.has_parameter('use_sim_time') else None
        self.last_error_wall=0.
        self.profile=str(self.declare_parameter('policy_profile','swarm').value)
        if self.profile not in PROFILES:raise ValueError('policy_profile must be swarm or high_speed')
        profile=PROFILES[self.profile]
        self.speed=float(self.declare_parameter('target_speed',2.5).value)
        self.goal_lookahead=float(self.declare_parameter('goal_lookahead_m',0.).value)
        self.command_tilt=float(self.declare_parameter('max_command_tilt_rad',0.).value)
        if not math.isfinite(self.command_tilt) or not 0<=self.command_tilt<math.pi/2:raise ValueError('invalid command tilt limit')
        if not math.isfinite(self.goal_lookahead) or self.goal_lookahead<0:raise ValueError('invalid goal lookahead')
        if not math.isfinite(self.speed) or not 0<self.speed<=profile['max_speed']:
            raise ValueError(f"target_speed must be in (0,{profile['max_speed']}] for {self.profile}")
        self.camera_x=float(self.declare_parameter('camera_x_m',.25).value)
        self.age=.2;self.skew=.05;self.solve_age=.2
        self.pose_history=deque(maxlen=250);self.twists=deque(maxlen=250)
        self.depth=self.info=self.session=None
        self.h=None;self.forward=None;self.pending=None;self.command=None;self.sequence=0
        self.last_image=None;self.last_request=-math.inf;self.last_clock=None;self.hover=None
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.client=self.create_client(InferDiffphys,'inference/diffphys/infer')
        self.output=self.create_publisher(ControlFrame,'external_acro/control',qos_profile_sensor_data)
        self.ready=self.create_publisher(Bool,'diffphys/ready',1)
        self.status=self.create_publisher(String,'diffphys/status',10)
        self.create_subscription(PoseStamped,'self_localization/pose',self.pose_history.append,qos_profile_sensor_data)
        self.create_subscription(TwistStamped,'self_localization/twist',self.twists.append,qos_profile_sensor_data)
        camera_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE) if self.profile=='high_speed' else qos_profile_sensor_data
        self.create_subscription(Image,'sensor_measurements/diffphys_front/depth_planar/image',self.on_depth,camera_qos)
        self.create_subscription(CameraInfo,'sensor_measurements/diffphys_front/depth_planar/camera_info',lambda m:setattr(self,'info',m),camera_qos)
        self.create_subscription(ControlSession,'external_acro/session',self.on_session,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.log=Path('/evidence/policy.jsonl').open('w')
        self.create_timer(.01,self.tick)

    def on_session(self,msg):
        if not self.session or (msg.session_id,msg.revision)!=(self.session.session_id,self.session.revision):
            self.h=None;self.forward=None;self.command=None;self.last_image=None;self.sequence=0
        self.session=msg
        if not msg.active:self.command=None

    def event(self,kind,**values):
        value=dict(kind=kind,wall=time.monotonic(),**values)
        self.log.write(json.dumps(value)+'\n');self.log.flush()
        self.status.publish(String(data=json.dumps(value)))

    def on_depth(self,image):
        self.depth=image
        if self.profile=='high_speed':
            self.event('image_received',stamp=stamp(image.header),stamp_ns=image.header.stamp.sec*10**9+image.header.stamp.nanosec,age=self.get_clock().now().nanoseconds/1e9-stamp(image.header),bytes=len(image.data))

    def snapshot(self,now):
        if not self.depth or not self.info or not self.pose_history:raise ValueError('inputs unavailable')
        image=self.depth;info=self.info
        if not 0<=now-stamp(image.header)<=self.age:raise ValueError('stale image')
        width,height,fov,pitch=(640,480,91,20) if self.profile=='high_speed' else (48,36,79,10)
        if (image.height,image.width)!=(height,width) or info.width!=width or info.height!=height:raise ValueError('camera dimensions')
        expected=width/(2*math.tan(math.radians(fov)/2))
        if abs(info.k[0]-expected)>1e-3 or abs(info.k[4]-expected)>1e-3:raise ValueError('camera calibration')
        if info.header.frame_id!=image.header.frame_id or any(abs(v)>1e-8 for v in info.d):raise ValueError('camera frame/distortion')
        t=self.buffer.lookup_transform('drone0/base_link',image.header.frame_id,Time()).transform
        if np.linalg.norm(vec(t.translation)-[self.camera_x,0,.1])>1e-4:raise ValueError('camera lever arm')
        expected_rotation=Rotation.from_euler('y',-pitch,degrees=True).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        if np.max(np.abs(rot(t.rotation)-expected_rotation))>1e-4:raise ValueError('camera rotation')
        pose=min(self.pose_history,key=lambda p:abs(stamp(p.header)-stamp(image.header)))
        if abs(stamp(pose.header)-stamp(image.header))>self.skew:raise ValueError('no paired pose')
        twist=next((v for v in reversed(self.twists) if v.header.stamp==pose.header.stamp),None)
        if twist is None:raise ValueError('no same-stamp twist')
        latest=self.pose_history[-1]
        if not 0<=now-stamp(latest.header)<=self.age:raise ValueError('stale current state')
        rotation=rot(pose.pose.orientation)
        velocity=vec(twist.twist.linear)
        if twist.header.frame_id=='drone0/base_link':velocity=rotation@velocity
        elif twist.header.frame_id!=pose.header.frame_id:raise ValueError('unknown velocity frame')
        return image,pose,rotation,velocity

    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9
        if self.last_clock is not None and now<self.last_clock:
            self.command=None;self.h=None;self.forward=None;self.event('clock_reversed');return
        self.last_clock=now
        if self.hover is None and Path('/evidence/hover.json').exists():
            self.hover=json.loads(Path('/evidence/hover.json').read_text())['normalized_thrust']
        try:
            snapshot=self.snapshot(now)
            healthy=bool(self.hover and self.client.service_is_ready())
        except Exception as error:
            snapshot=None;healthy=False
            if time.monotonic()-self.last_error_wall>1.:
                self.last_error_wall=time.monotonic();self.event('input_wait',error=str(error))
        self.ready.publish(Bool(data=healthy))
        if self.pending and self.pending['future'].done():
            old=self.pending;self.pending=None
            self.event('inference_finished',request_id=old['request'].request_id,wall_elapsed=time.monotonic()-old['wall'])
            try:result=old['future'].result()
            except Exception as error:
                self.event('service_exception',error=str(error));return
            context=(self.session and self.session.active and old['key']==(self.session.session_id,self.session.revision))
            timely=(healthy and 0<=now-old['started']<=self.solve_age and 0<=now-stamp(old['request'].depth.header)<=self.age and stamp(self.session.header)<=old['started']<=now if context else False)
            if not result or not result.success or not context or not timely:
                self.event('inference_rejected',error=result.error_code if result else 'NO_RESULT');return
            try:
                relative=old['goal']-old['position']
                if PROFILES[self.profile]['heading_clipped']:
                    heading=old['level']@np.asarray(old['request'].state[3:6])
                else:heading=relative
                q,force=decode(result.action,old['level'],old['rotation'],relative,old['local_vz'],self.hover,
                               gravity=PROFILES[self.profile]['gravity'],heading_vector=heading,
                               vertical_correction=PROFILES[self.profile]['vertical_correction'],
                               previous_forward=(old['level'][:,0] if self.forward is None else self.forward)
                                   if self.profile=='high_speed' else None,max_command_tilt_rad=self.command_tilt)
                if result.request_id!=old['request'].request_id or result.state_header!=old['request'].state_header or result.depth_header!=old['request'].depth.header:raise ValueError('response identity mismatch')
                if result.device!='cuda' or not result.model_version.startswith(PROFILES[self.profile]['version']) or len(result.hidden_state)!=192 or not np.isfinite(result.hidden_state).all():raise ValueError('CUDA/model-profile/hidden-state contract')
                self.h=list(result.hidden_state)
                if self.profile=='high_speed':self.forward=Rotation.from_quat(q).as_matrix()[:,0]
                self.command=(q,force,now,old['key'],old['request'].state_header.frame_id)
                self.event('policy',action=list(map(float,result.action)),force=force,q=q.tolist(),model=result.model_version,device=result.device,depth_stamp=stamp(old['request'].depth.header),latency=now-old['started'])
            except ValueError as error:self.event('decode_rejected',error=str(error));self.command=None
        if not self.session or not self.session.active or not healthy:return
        session=self.session
        if session.control_mode!=ControlFrame.ATTITUDE:return
        if self.command:
            q,force,solved,key,frame=self.command
            if 0<=now-solved<=self.solve_age and key==(session.session_id,session.revision) and solved>=stamp(session.header):
                msg=ControlFrame();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='drone0/base_link'
                msg.last_successful_solve=rclpy.time.Time(seconds=solved).to_msg()
                msg.session_id=key[0];msg.revision=key[1];self.sequence+=1;msg.sequence=self.sequence
                msg.control_mode=ControlFrame.ATTITUDE;msg.attitude_frame_id=frame
                msg.attitude.x,msg.attitude.y,msg.attitude.z,msg.attitude.w=map(float,q);msg.thrust_newtons=force;msg.solver_ok=True
                self.output.publish(msg)
        if self.pending or now-self.last_request<1/15:return
        image,pose,rotation,velocity=snapshot
        image_key=(image.header.stamp.sec,image.header.stamp.nanosec)
        if image_key==self.last_image:return
        if session.target.header.frame_id!=pose.header.frame_id:return
        goal=vec(session.target.point);position=vec(pose.pose.position)
        state,level=observation(position,rotation,velocity,goal,self.speed,
            margin=PROFILES[self.profile]['margin'],full_attitude=self.profile=='high_speed',
            goal_lookahead_m=self.goal_lookahead)
        if np.any(np.abs(level@state[3:6])>vec(session.max_speed)+1e-6):return
        request=InferDiffphys.Request();request.request_id=session.session_id+'-'+str(image.header.stamp.sec)+'-'+str(image.header.stamp.nanosec)
        try:sampled=sample_high_speed(image) if self.profile=='high_speed' else copy.deepcopy(image)
        except ValueError as error:
            self.command=None;self.event('invalid_depth',error=str(error));return
        request.state_header=copy.deepcopy(pose.header);request.depth=sampled;request.state=list(map(float,state));request.hidden_state=self.h or []
        if not Path('/evidence/first-observation.json').exists():
            Path('/evidence/first-source-depth.bin').write_bytes(bytes(image.data))
            Path('/evidence/first-source-camera.json').write_text(json.dumps(dict(width=image.width,height=image.height,step=image.step,encoding=image.encoding,is_bigendian=bool(image.is_bigendian),stamp=stamp(image.header),frame=image.header.frame_id,k=list(self.info.k),profile=self.profile)))
            from uav_inference.diffphys_contract import request_payload
            Path('/evidence/first-observation.json').write_text(json.dumps(request_payload(request,round(now*1e9),self.age,self.skew)))
        future=self.client.call_async(request)
        self.event('inference_sent',request_id=request.request_id)
        self.pending=dict(future=future,wall=time.monotonic(),key=(session.session_id,session.revision),started=now,request=request,level=level,rotation=rotation,goal=goal,position=position,local_vz=float(velocity@level[:,2]))
        self.last_request=now;self.last_image=image_key


def main():
    rclpy.init();node=Planner()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.log.close();node.destroy_node();rclpy.try_shutdown()
