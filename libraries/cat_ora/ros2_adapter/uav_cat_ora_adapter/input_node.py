"""Expose checked local AS2 odometry for CAT-ORA and its task completion observer."""
import time
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml
from .state import checked_pose


class CatOraStateAdapter(Node):
    def __init__(self):
        super().__init__('cat_ora_state_adapter')
        self.declare_parameter('deployment_config','')
        deployment=yaml.safe_load(Path(self.get_parameter('deployment_config').value).read_text())
        self.stamps={};self.sent={};self.warned={};self._inputs=[];self._outputs={}
        self.received={v:0 for v in deployment};self.published=dict(self.received);self.rejected={}
        self.diagnostics=self.create_publisher(String,'planner/catora_input_state',1)
        self.create_timer(1.,self.publish_state)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        for vehicle,config in deployment.items():
            namespace=config['namespace'].strip('/')
            self._outputs[vehicle]=self.create_publisher(PoseStamped,f'/{namespace}/catora/local_pose',1)
            self._inputs.append(self.create_subscription(Odometry,f'/{namespace}/sensor_measurements/odom',
                lambda msg,v=vehicle,n=namespace:self.on_odometry(v,n,msg),qos))
            self.create_service(Trigger,f'/{namespace}/catora/ping',lambda request,response,v=vehicle:self.ping(v,response))

    def ping(self,vehicle,response):
        response.success=self.published[vehicle]>0
        response.message=json.dumps({'vehicle':vehicle,'published':self.published[vehicle]})
        return response

    def on_odometry(self,vehicle,namespace,message):
        self.received[vehicle]+=1
        try:
            stamp=checked_pose(message,namespace+'/odom',self.get_clock().now().nanoseconds/1e9,self.stamps.get(vehicle,0))
        except ValueError as exc:
            self.rejected[vehicle]=str(exc)
            now=time.monotonic()
            if str(exc)!='duplicate or out-of-order pose' and now-self.warned.get(vehicle,0)>2:
                self.get_logger().warning(vehicle+': '+str(exc));self.warned[vehicle]=now
            return
        self.stamps[vehicle]=stamp
        # Only planning state is throttled; AS2 control inputs retain their original rate.
        if stamp-self.sent.get(vehicle,0)<.05:return
        result=PoseStamped();result.header=message.header;result.pose=message.pose.pose
        self._outputs[vehicle].publish(result);self.sent[vehicle]=stamp
        self.published[vehicle]+=1

    def publish_state(self):
        message=String();message.data=json.dumps({'received':self.received,'published':self.published,'last_rejection':self.rejected})
        self.diagnostics.publish(message)


def main(args=None):
    rclpy.init(args=args);node=CatOraStateAdapter()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
