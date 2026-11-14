"""Publish Gazebo Harmonic simulation time received from the PX4 container."""

from __future__ import annotations

import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class GazeboClockBridge(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_clock_bridge")
        self.declare_parameter("port", 28502)
        self.publisher = self.create_publisher(Clock, "/clock", 10)
        self.latest: tuple[int, int] | None = None
        self.published: tuple[int, int] | None = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", int(self.get_parameter("port").value)))
        self.socket.settimeout(0.5)
        self.running = True
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        self.create_timer(0.01, self._publish)

    def _read(self) -> None:
        while self.running:
            try:
                payload, _address = self.socket.recvfrom(12)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(payload) == 12:
                self.latest = struct.unpack("!qI", payload)

    def _publish(self) -> None:
        if self.latest is None or self.latest == self.published:
            return
        message = Clock()
        message.clock.sec, message.clock.nanosec = self.latest
        self.publisher.publish(message)
        self.published = self.latest

    def destroy_node(self) -> bool:
        self.running = False
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboClockBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
