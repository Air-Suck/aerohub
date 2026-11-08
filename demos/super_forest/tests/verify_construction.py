"""Image-local smoke: construct actual ROS nodes without enabling or flying."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from mission import Mission
from uav_super_adapter.node import SuperAdapter

Path('/evidence').mkdir(exist_ok=True)
rclpy.init()
mission=adapter=None
try:
    mission=Mission()
    adapter=SuperAdapter()
    rclpy.spin_once(adapter,timeout_sec=.05)
    assert not adapter.enabled and adapter.client is None
    print('PASS: mission and adapter construct; no execution enabled')
finally:
    if adapter:adapter.destroy_node()
    if mission:mission.close()
    rclpy.try_shutdown()
