"""Accepted plans bind completion and audit to original vehicle identities."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
try:
    from uav_planner_framework.ros_node import PlannerFrameworkNode as PlannerNode
except ImportError:
    PlannerNode = None
from uav_planner_framework.behavior import BehaviorStatus
from uav_planner_framework.trajectory import TimedTrajectorySegment, VehicleTrajectory, TrajectoryPoint, Vector3

@unittest.skipIf(PlannerNode is None, 'requires ROS environment')
class AssignmentAcceptance(unittest.TestCase):
    def test_accepted_swap_binds_completion_and_rejects_mid_task_change(self):
        with tempfile.TemporaryDirectory() as directory:
            node = NS(lifecycle=NS(snapshot=NS(task_id='task',status=BehaviorStatus.RUNNING)),
                resolved={'planner':{'id':'cat_ora','parameters':{'goal_assignment':'optimize'}},
                          'targets':{'a':{'x':0.,'y':0.,'z':-3.},'b':{'x':4.,'y':0.,'z':-3.}}},
                goal_assignment=None, gateway=Mock(), completion=Mock(), execution=Mock(),
                assignment_publisher=Mock(), get_parameter=lambda name:NS(value=directory),
                _now=lambda:1., _fail=Mock())
            def segment(swapped):
                targets=(4.,0.) if swapped else (0.,4.)
                return TimedTrajectorySegment(1,'task',1.,2.,True,tuple(
                    VehicleTrajectory(v,frozenset(),(TrajectoryPoint(2.,Vector3(x,0.,-3.)),))
                    for v,x in zip(('a','b'),targets)))
            message=NS(task_id='task')
            with patch('uav_planner_framework.ros_node.segment_from_msg',return_value=segment(True)):
                PlannerNode._on_trajectory(node,message)
            node._fail.assert_not_called()
            self.assertEqual(4.,node.completion.targets['a'].x)
            audit=json.loads((Path(directory)/'task.assignment.json').read_text())
            self.assertEqual('b',audit['bindings']['a']['slot_id'])
            node.execution.submit.assert_called_once_with(message)
            with patch('uav_planner_framework.ros_node.segment_from_msg',return_value=segment(False)):
                PlannerNode._on_trajectory(node,message)
            node._fail.assert_called_once()
            node.execution.submit.assert_called_once()

    def test_rejected_plan_cannot_publish_assignment_or_rebind_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            node=NS(lifecycle=NS(snapshot=NS(task_id='task',status=BehaviorStatus.RUNNING)),
                resolved={'planner':{'id':'cat_ora','parameters':{'goal_assignment':'optimize'}},
                          'targets':{'a':{'x':0.,'y':0.,'z':-3.}}},
                goal_assignment=None,gateway=Mock(),completion=Mock(),execution=Mock(),
                assignment_publisher=Mock(),_now=lambda:1.,_fail=Mock())
            node.gateway.accept.return_value=False
            segment=TimedTrajectorySegment(1,'task',1.,2.,True,(
                VehicleTrajectory('a',frozenset(),(TrajectoryPoint(2.,Vector3(0.,0.,-3.)),)),))
            with patch('uav_planner_framework.ros_node.segment_from_msg',return_value=segment):
                PlannerNode._on_trajectory(node,NS(task_id='task'))
            self.assertIsNone(node.goal_assignment)
            node.execution.submit.assert_not_called()
            node.assignment_publisher.publish.assert_not_called()
