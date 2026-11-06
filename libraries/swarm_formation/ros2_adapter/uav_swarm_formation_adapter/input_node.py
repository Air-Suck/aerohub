"""Expose checked local navigation state and real input telemetry for Swarm tasks."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger


class SwarmInputAdapter(Node):
    def __init__(self):
        super().__init__('swarm_input_adapter')
        self.vehicle = self.declare_parameter('vehicle_id', 'drone0').value
        self.counts = {'odom': 0, 'cloud': 0}
        self.stamps = {}; self.last = {}; self.map_healthy = False
        self.publisher = self.create_publisher(PoseStamped, f'/{self.vehicle}/swarm/local_pose', qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(String, '/planner/swarm_input_state', 10)
        for kind, message in [('odom', Odometry), ('cloud', PointCloud2)]:
            self.create_subscription(message, f'/{self.vehicle}/navigation/'+('points' if kind=='cloud' else 'odom'),
                lambda msg,k=kind:self.receive(k,msg), qos_profile_sensor_data)
        self.create_subscription(Bool, f'/drone_{int(self.vehicle[5:])}_ego_planner_node/grid_map/input_healthy',
                                 self.on_health,10)
        self.create_service(Trigger, f'/{self.vehicle}/swarm/ping', self.ping)
        self.create_timer(.5,self.report)

    def on_health(self, msg):
        self.map_healthy = msg.data

    def healthy(self):
        return self.map_healthy and all(time.monotonic()-self.last.get(k,0)<1 for k in self.counts)

    def receive(self, kind, msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        if msg.header.frame_id!=self.vehicle+'/odom' or stamp<=0 or not -.05<=age<=1 or stamp<=self.stamps.get(kind,0):return
        if kind=='odom':
            p=msg.pose.pose.position; q=msg.pose.pose.orientation
            if not all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or abs(sum(x*x for x in (q.x,q.y,q.z,q.w))-1)>.05:return
            pose=PoseStamped();pose.header=msg.header;pose.pose=msg.pose.pose;self.publisher.publish(pose)
        elif not msg.width*msg.height or not msg.data:return
        self.counts[kind]+=1;self.stamps[kind]=stamp;self.last[kind]=time.monotonic()

    def ping(self, request, response):
        response.success=self.healthy();response.message=json.dumps(self.counts);return response

    def report(self):
        state={'vehicle':self.vehicle,'counts':self.counts,'stamps':self.stamps,'map_healthy':self.map_healthy,'healthy':self.healthy()}
        self.diagnostics.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args);node=SwarmInputAdapter()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
