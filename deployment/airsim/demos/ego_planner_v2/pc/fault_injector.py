"""Test-only transparent DDS gates and exact-process fault injection.

AS2 navigation topics remain live. Only the adapter's drone0 input is remapped.
The process signal is restricted to this compute container's native drone0 node.
"""
import json
import os
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class Injector(Node):
    def __init__(self):
        super().__init__('acceptance_fault_injector')
        self.blocked = dict(odom=False, cloud=False)
        self.counts = {k: dict(received=0, forwarded=0, dropped=0) for k in self.blocked}
        self.events = []
        self.pubs = {}
        for key, kind, suffix in [('odom', Odometry, 'odom'), ('cloud', PointCloud2, 'points')]:
            self.pubs[key] = self.create_publisher(kind, '/acceptance/drone0/'+suffix, qos_profile_sensor_data)
            self.create_subscription(kind, '/drone0/navigation/'+suffix,
                lambda m,k=key:self.forward(k,m), qos_profile_sensor_data)
            self.create_service(SetBool, '/acceptance/block_'+key,
                lambda req,res,k=key:self.gate(k,req,res))
        self.create_service(Trigger, '/acceptance/exit_planner', self.kill)
        self.status = self.create_publisher(String, '/acceptance/injector_status', 10)
        self.create_timer(.1, self.report)

    def forward(self, key, msg):
        self.counts[key]['received'] += 1
        if self.blocked[key]:
            self.counts[key]['dropped'] += 1
        else:
            self.pubs[key].publish(msg)
            self.counts[key]['forwarded'] += 1

    def event(self, **values):
        row = dict(wall=time.time(), monotonic=time.monotonic(), **values)
        self.events.append(row)
        self.get_logger().info(json.dumps(row))

    def gate(self, key, req, res):
        self.blocked[key] = req.data
        self.event(kind=key, blocked=req.data, counts=self.counts[key].copy())
        res.success = True
        res.message = json.dumps(self.events[-1])
        return res

    def kill(self, req, res):
        candidates = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:
                argv = (entry/'cmdline').read_bytes().decode().split('\0')
                if (argv[0] == '/work/install/humble/ego_planner/lib/ego_planner/ego_planner_node'
                        and '__ns:=/drone0' in argv):
                    candidates.append((int(entry.name), argv))
            except (OSError, UnicodeError):pass
        if len(candidates) != 1:
            res.success = False;res.message = 'expected exactly one owned native drone0 process: '+repr(candidates)
            return res
        pid, argv = candidates[0]
        os.kill(pid, signal.SIGKILL)
        self.event(kind='planner_exit', pid=pid, argv=argv, signal='SIGKILL')
        res.success = True;res.message = json.dumps(self.events[-1])
        return res

    def report(self):
        self.status.publish(String(data=json.dumps(dict(blocked=self.blocked, counts=self.counts, events=self.events))))


if __name__ == '__main__':
    rclpy.init();node=Injector()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
