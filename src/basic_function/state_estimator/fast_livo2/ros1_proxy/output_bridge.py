#!/usr/bin/env python3
"""Bounded latest-sample TCP streams for odometry, cloud and optional bag clock."""
import base64
import json
import socket
import struct
import threading
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock

class Stream:
    def __init__(self, port):
        self.condition = threading.Condition()
        self.packet = None
        self.sequence = 0
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', port))
        self.server.listen(1)
        self.server.settimeout(1)
        threading.Thread(target=self.run, daemon=True).start()

    def push(self, obj):
        try:
            data = json.dumps(dict(version=1, **obj), allow_nan=False, separators=(',', ':')).encode()
        except (ValueError, TypeError):
            rospy.logwarn_throttle(5, 'Discard nonfinite/invalid FAST-LIVO2 output')
            return
        if len(data) > 32 * 1024 * 1024:
            rospy.logwarn_throttle(5, 'Cloud exceeds 32 MiB transport limit; reduce upstream publication size')
            return
        with self.condition:
            self.packet = struct.pack('!I', len(data)) + data
            self.sequence += 1
            self.condition.notify_all()

    def run(self):
        while not rospy.is_shutdown():
            try:
                conn, _ = self.server.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(2)
                # Never replay cached old state on a new connection.
                previous = self.sequence
                while not rospy.is_shutdown():
                    with self.condition:
                        self.condition.wait_for(lambda: self.sequence != previous, timeout=1)
                        if self.sequence == previous:
                            continue
                        packet, previous = self.packet, self.sequence
                    try:
                        conn.sendall(packet)
                    except OSError:
                        break
        self.server.close()

def vec(v):
    return [v.x, v.y, v.z]

def header(m):
    return dict(stamp=[m.header.stamp.secs, m.header.stamp.nsecs], frame=m.header.frame_id)

def main():
    rospy.init_node('fast_livo2_output_bridge')
    port = rospy.get_param('~port', 18761)
    odom_topic = rospy.get_param('~odom_topic', '/LIVO2/imu_propagate')
    odom, cloud, clock = [Stream(port+i) for i in range(3)]
    def on_odom(m):
        q = m.pose.pose.orientation
        odom.push(dict(kind='odom', **header(m), child=m.child_frame_id,
                      position=vec(m.pose.pose.position), orientation=[q.x,q.y,q.z,q.w],
                      linear=vec(m.twist.twist.linear), angular=vec(m.twist.twist.angular),
                      pose_covariance=list(m.pose.covariance), twist_covariance=list(m.twist.covariance)))
    def on_cloud(m):
        cloud.push(dict(kind='cloud', **header(m), height=m.height, width=m.width,
                       fields=[[f.name,f.offset,f.datatype,f.count] for f in m.fields],
                       is_bigendian=m.is_bigendian, point_step=m.point_step, row_step=m.row_step,
                       is_dense=m.is_dense, data=base64.b64encode(bytes(m.data)).decode()))
    rospy.Subscriber(odom_topic, Odometry, on_odom, queue_size=1)
    rospy.Subscriber('/cloud_registered', PointCloud2, on_cloud, queue_size=1, buff_size=32*1024*1024)
    rospy.Subscriber('/clock', Clock, lambda m: clock.push(dict(kind='clock', stamp=[m.clock.secs,m.clock.nsecs])), queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
