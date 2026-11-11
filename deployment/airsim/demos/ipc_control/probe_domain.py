"""Refuse to start a flight test in an occupied ROS domain."""
import json
import time
import rclpy
rclpy.init()
node = rclpy.create_node('ipc_control_domain_probe')
until = time.monotonic()+3
while time.monotonic()<until:
    rclpy.spin_once(node,timeout_sec=.1)
others = [(name, ns) for name, ns in node.get_node_names_and_namespaces()
          if name != 'ipc_control_domain_probe']
print(json.dumps({'other_nodes':others}))
node.destroy_node(); rclpy.shutdown()
raise SystemExit(1 if others else 0)
