"""ROS-only DiffPhys course task, with native Takeoff/FollowReference/Land."""
import json,math,time,signal
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import Takeoff,FollowReference
from as2_msgs.msg import YawMode,ControlMode
from std_msgs.msg import Bool,String
from std_srvs.srv import SetBool
from yopo_mission import Mission

class DiffphysMission(Mission):
    def __init__(self):
        super().__init__(Path('/evidence'),[0.,120.,3.],0)
        self.source_ready=False;self.health={};self.action_handle=None
        self.controller=None
        from as2_msgs.msg import ControllerInfo
        self.create_subscription(ControllerInfo,'/drone0/controller/info',lambda m:setattr(self,'controller',m),qos_profile_sensor_data)
        self.create_subscription(Bool,'/drone0/diffphys/ready',lambda m:setattr(self,'source_ready',m.data),1)
        self.create_subscription(String,'/drone0/diffphys/status',self.on_health,qos_profile_sensor_data)
        self.status={};self.result['planner']='diffphys'
        self.result.pop('arrived_5m',None)
        self.result.pop('stopped',None)
    def on_health(self,msg):
        self.health=json.loads(msg.data)
    def graph_ready(self):
        return self.clock_samples>=25 and all(self.count_publishers(t)==1 for t in ['/clock','/drone0/self_localization/pose','/drone0/self_localization/twist'])
    def mode_is_position(self):
        return self.controller is not None and self.controller.input_control_mode.control_mode==ControlMode.POSITION
    def navigate(self):
        self.wait(lambda:self.platform is not None and self.pose is not None and self.graph_ready(),60)
        if self.platform.armed:raise RuntimeError('requires unarmed platform')
        origin=[self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z]
        self.result.update(origin=origin,target=[origin[0],origin[1]+120,origin[2]+3])
        from as2_msgs.srv import SetControlMode
        request=SetControlMode.Request();request.control_mode.control_mode=ControlMode.ATTITUDE;request.control_mode.yaw_mode=ControlMode.YAW_ANGLE
        self.service(SetControlMode,'/drone0/controller/set_control_mode',request)
        self.service(SetBool,'/drone0/set_arming_state',SetBool.Request(data=True))
        self.result['motion_requested']=True
        self.wait(lambda:self.platform.armed)
        self.service(SetBool,'/drone0/set_offboard_mode',SetBool.Request(data=True));self.wait(lambda:self.platform.offboard)
        self.action(Takeoff,'/drone0/TakeoffBehavior',Takeoff.Goal(takeoff_height=3.,takeoff_speed=.5))
        until=time.monotonic()+3;self.wait(lambda:time.monotonic()>until)
        Path('/evidence/need-hover').touch()
        self.wait(lambda:Path('/evidence/hover.json').exists(),30)
        self.wait(lambda:self.source_ready,30)
        goal=FollowReference.Goal();goal.target_pose.header.frame_id='earth'
        # DiffPhys uses earth, so transform measured task goal from odom explicitly.
        from geometry_msgs.msg import PoseStamped
        target=PoseStamped();target.header.frame_id='drone0/odom';target.pose.orientation.w=1.
        target.pose.position.x,target.pose.position.y,target.pose.position.z=self.result['target']
        actual=self.tf.transform(target,'earth')
        goal.target_pose.point.x=actual.pose.position.x;goal.target_pose.point.y=actual.pose.position.y;goal.target_pose.point.z=actual.pose.position.z
        q=self.pose.pose.orientation
        goal.yaw.mode=YawMode.FIXED_YAW;goal.yaw.angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        goal.max_speed_x=goal.max_speed_y=goal.max_speed_z=5.
        cli=ActionClient(self,FollowReference,'/drone0/FollowReferenceBehavior');self.task_clients.append(cli)
        self.wait(cli.server_is_ready);future=cli.send_goal_async(goal);self.wait(future.done);self.action_handle=future.result()
        if not self.action_handle.accepted:raise RuntimeError('DiffPhys FollowReference rejected')
        execution=self.action_handle.get_result_async()
        self.hud('start');self.hud_started=True
        stable=None;start=self.get_clock().now().nanoseconds/1e9
        while True:
            rclpy.spin_once(self,timeout_sec=.02)
            if execution.done():raise RuntimeError('DiffPhys execution ended: '+str(execution.result()))
            now=self.get_clock().now().nanoseconds/1e9
            p=self.pose.pose.position;local=[p.x-origin[0],p.y-origin[1],p.z-origin[2]]
            v=self.twist.twist.linear;speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            error=math.dist(local,[0,120,3])
            self.samples.write(json.dumps(dict(sim=now,position=local,speed=speed,goal_error=error,health=self.health))+'\n');self.samples.flush()
            if not -6<=local[0]<=6 or not -2<=local[1]<=123:raise RuntimeError('course horizontal boundary')
            q=self.pose.pose.orientation
            tilt=math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y))))
            if tilt>.6:raise RuntimeError('DiffPhys tilt exceeds 0.6rad')
            if now-start>300:raise TimeoutError('DiffPhys course timeout')
            if error<=6.:break
        # Follow the existing DiffPhys terminal contract: cancel policy, then native GoTo.
        canceled=self.action_handle.cancel_goal_async();self.wait(canceled.done,5)
        if not canceled.result().goals_canceling:raise RuntimeError('policy cancellation rejected')
        self.wait(execution.done,10);self.action_handle=None
        self.wait(lambda:self.mode_is_position(),10)
        from as2_msgs.action import GoToWaypoint
        terminal=GoToWaypoint.Goal();terminal.target_pose.header.frame_id='drone0/odom'
        terminal.target_pose.point.x,terminal.target_pose.point.y,terminal.target_pose.point.z=self.result['target']
        terminal.yaw.mode=YawMode.FIXED_YAW;terminal.yaw.angle=goal.yaw.angle
        terminal.max_speed=2.5
        client=ActionClient(self,GoToWaypoint,'/drone0/GoToBehavior');self.task_clients.append(client)
        self.wait(client.server_is_ready);future=client.send_goal_async(terminal);self.wait(future.done)
        self.action_handle=future.result()
        if not self.action_handle.accepted:raise RuntimeError('terminal GoTo rejected')
        finished=self.action_handle.get_result_async();stable=None;deadline=time.monotonic()+90
        while True:
            rclpy.spin_once(self,timeout_sec=.02)
            p=self.pose.pose.position;error=math.dist([p.x,p.y,p.z],self.result['target'])
            v=self.twist.twist.linear;speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            now=self.get_clock().now().nanoseconds/1e9
            if error<=1.5 and speed<=.3:
                if stable is None:stable=now
                if now-stable>=2 and finished.done():
                    if not finished.result().result.go_to_success:raise RuntimeError('terminal GoTo failed')
                    break
            else:stable=None
            if time.monotonic()>deadline:raise TimeoutError('terminal GoTo settling')
        self.action_handle=None
        self.result.update(terminal_behavior='GoToBehavior',goal_error_m=error,stop_speed_m_s=speed)
        self.result['arrived']=True
    def run(self):
        try:self.navigate()
        except Exception as exc:self.result['error']=str(exc)
        finally:
            if self.action_handle is not None:
                f=self.action_handle.cancel_goal_async()
                try:self.wait(f.done,5)
                except Exception as exc:self.result['cancel_error']=str(exc)
            if self.hud_started:self.hud('finish')
            from as2_msgs.action import Land
            try:
                if self.platform and self.platform.armed:
                    self.action(Land,'/drone0/LandBehavior',Land.Goal(land_speed=.5));self.result['land_success']=True
                    self.wait(lambda:not self.platform.armed,30)
            except Exception as exc:self.result['recovery_error']=str(exc)
            self.result['disarmed']=bool(self.platform and not self.platform.armed)
            self.result['success']=bool(self.result.get('arrived') and self.result['land_success'] and self.result['disarmed'] and not self.result.get('error'))
            Path('/evidence/result.json').write_text(json.dumps(self.result,indent=2))
            self.samples.close();self.events.close()

def main():
    rclpy.init();node=DiffphysMission()
    def stop(*_):raise RuntimeError('interrupted; recovering')
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:node.run()
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
