"""Read-only DDS participant identity diagnostic, no motion interfaces."""
import json
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
name=os.environ.get('PROBE_NAME','guid_probe')
node=Node(name)
node.create_publisher(String,'/'+name,1)
deadline=time.monotonic()+3
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.05)
info=node.get_publishers_info_by_topic('/'+name)
print(json.dumps(dict(name=name,pid=os.getpid(),gids=[bytes(p.endpoint_gid).hex() for p in info])),flush=True)
node.destroy_node();rclpy.shutdown()
