"""Shared-image message loading and workstation boundary; no motion requests."""
import rclpy
from rclpy.type_support import check_for_type_support
from as2_msgs.action import Takeoff,Land
from px4_msgs.msg import VehicleCommand,VehicleCommandAck
from uav_planner_interfaces.action import FollowTimedPath
from mars_quadrotor_msgs.msg import PolynomialTrajectory
from mission import Mission

for kind in (Takeoff,Land,FollowTimedPath,VehicleCommand,VehicleCommandAck,PolynomialTrajectory):
    check_for_type_support(kind)
rclpy.init();node=Mission()
try:
    assert node.four_part and not node.children and not node.result['motion_requested']
    try:node.start('forbidden',['false'])
    except RuntimeError:pass
    else:raise AssertionError('Workstation spawned a compute process')
    try:node.emergency_land()
    except RuntimeError:pass
    else:raise AssertionError('Workstation bypassed compute execution for recovery')
    print('PASS: shared image loads actual interfaces; workstation cannot spawn compute nodes')
finally:node.close();rclpy.try_shutdown()
