import unittest

from uav_swarm_formation_adapter.node import (
    formation_center,
    local_enu_to_fleet_enu,
    validate_trajectory_starts,
)


class ConversionTest(unittest.TestCase):
    def test_local_enu_to_fleet_enu(self):
        self.assertEqual(
            local_enu_to_fleet_enu([1.0, 2.0, 3.0], [-4.0, -10.0, 0.0]),
            [-9.0, -2.0, 3.0],
        )

    def test_center_from_configured_shape(self):
        config = {
            "vehicles": [f"drone{i}" for i in range(7)],
            "targets": {
                "drone0": {"x": 0.0, "y": -4.0, "z": -1.5},
                "drone1": {"x": -1.0, "y": -2.2679, "z": -1.5},
                "drone2": {"x": -2.0, "y": -4.0, "z": -1.5},
                "drone3": {"x": -1.0, "y": -5.7321, "z": -1.5},
                "drone4": {"x": 1.0, "y": -5.7321, "z": -1.5},
                "drone5": {"x": 2.0, "y": -4.0, "z": -1.5},
                "drone6": {"x": 1.0, "y": -2.2679, "z": -1.5},
            },
        }
        self.assertEqual(formation_center(config), (-4.0, 0.0, 1.5))

    def test_rejects_incompatible_shape(self):
        config = {
            "vehicles": [f"drone{i}" for i in range(7)],
            "targets": {vehicle: {"x": 0.0, "y": 0.0, "z": -1.0} for vehicle in [f"drone{i}" for i in range(7)]},
        }
        with self.assertRaises(ValueError):
            formation_center(config)

    def test_rejects_discontinuous_trajectory_start(self):
        with self.assertRaisesRegex(ValueError, "starts 5.000 m"):
            validate_trajectory_starts(
                ["drone0"],
                {"drone0": [0.0, 0.0, 1.0]},
                [[[3.0, 4.0, 1.0], [3.1, 4.0, 1.0]]],
                1.0,
            )

    def test_accepts_continuous_trajectory_start(self):
        validate_trajectory_starts(
            ["drone0"],
            {"drone0": [0.0, 0.0, 1.0]},
            [[[0.1, 0.0, 1.0], [0.2, 0.0, 1.0]]],
            1.0,
        )


class ThreeVehicleContractTest(unittest.TestCase):
    def test_runtime_output_timeout_only_fails_after_healthy_output(self):
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        from uav_swarm_formation_adapter.node import SwarmFormationAdapter
        node = Mock()
        node.output_started = False
        node.failure_sent = False
        node.unavailable_since = None
        node.config = {'task': {'task_id': 'runtime-loss'}}
        node.get_parameter.return_value.value = 3.0
        SwarmFormationAdapter._output_unavailable(node, 'startup unavailable')
        node.failure_publisher.publish.assert_not_called()
        node.output_started = True
        with patch('uav_swarm_formation_adapter.node.time.monotonic',
                   side_effect=(10.0, 12.9, 13.1)):
            SwarmFormationAdapter._output_unavailable(node, 'first loss')
            SwarmFormationAdapter._output_unavailable(node, 'still unavailable')
            node.failure_publisher.publish.assert_not_called()
            SwarmFormationAdapter._output_unavailable(node, 'timeout')
        failure = node.failure_publisher.publish.call_args.args[0]
        self.assertEqual(failure.task_id, 'runtime-loss')
        self.assertEqual(failure.reason, 'OUTPUT_UNAVAILABLE')
        self.assertTrue(node.failure_sent)
        self.assertTrue(node.final_sent)

    def test_three_vehicle_scaled_targets_share_one_anchor(self):
        config = {
            'vehicles': ['drone0', 'drone1', 'drone2'],
            'planner': {'parameters': {'formation_scale': 2.0}},
            'targets': {
                'drone0': {'x': 6.0, 'y': 0.0, 'z': -3.0},
                'drone1': {'x': 4.0, 'y': 3.4642, 'z': -3.0},
                'drone2': {'x': 2.0, 'y': 0.0, 'z': -3.0},
            },
        }
        self.assertEqual(formation_center(config), (0.0, 6.0, 3.0))
        config['targets']['drone2']['z'] = -.5
        with self.assertRaises(ValueError):
            formation_center(config)

    def test_response_sampling_time_survives_adapter_delay(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from uav_swarm_formation_adapter.node import SwarmFormationAdapter
        node = Mock()
        node.vehicles = ['drone0', 'drone1', 'drone2']
        node.config = {'task': {'task_id': 'sample-clock'},
                       'planner': {'parameters': {'sample_dt': .1, 'valid_from_lead': .2}}}
        node.get_clock.return_value.now.return_value = SimpleNamespace(nanoseconds=101_500_000_000)
        msg = SwarmFormationAdapter._message(node, [[[0,0,3],[0,.1,3]]] * 3, valid_from=101.2)
        self.assertEqual((msg.valid_from.sec, msg.valid_from.nanosec), (101,200_000_000))
        self.assertEqual((msg.generated_at.sec,msg.generated_at.nanosec), (101,500_000_000))
        self.assertEqual(len(msg.vehicle_trajectories),3)

    def test_live_goal_waits_for_every_owned_output_publisher(self):
        from unittest.mock import Mock, patch
        from uav_swarm_formation_adapter.node import SwarmFormationAdapter
        node = Mock()
        node.vehicles = ['drone0','drone1','drone2']
        node.poses = dict.fromkeys(node.vehicles, [0,0,3])
        node.final_sent = False
        node.config = {'planner': {'parameters': {'wait_for_output': True}},
                       'deployment': {v:{'namespace':v} for v in node.vehicles}}
        from types import SimpleNamespace
        native = SimpleNamespace(node_name='TakeoffBehavior')
        owned = SimpleNamespace(node_name='swarm_as2_adapter')
        node.get_publishers_info_by_topic.side_effect = [[native,owned],[native,owned],[native]]
        with patch('uav_swarm_formation_adapter.node.socket.create_connection') as connect:
            SwarmFormationAdapter._update(node)
            connect.assert_not_called()
        self.assertEqual(node.get_publishers_info_by_topic.call_count,3)

    def test_pva_fields_and_ned_axes_are_preserved(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from uav_swarm_formation_adapter.node import SwarmFormationAdapter
        node=Mock();node.vehicles=['drone0']
        node.config={'task':{'task_id':'pva'},'planner':{'parameters':{'sample_dt':.1,'valid_from_lead':.2}}}
        node.get_clock.return_value.now.return_value=SimpleNamespace(nanoseconds=100_000_000_000)
        msg=SwarmFormationAdapter._message(node,[[[1,2,3],[2,3,4]]],100.2,
            [[[4,5,6],[4,5,6]]],[[[7,8,9],[7,8,9]]])
        trajectory=msg.vehicle_trajectories[0];point=trajectory.points[0]
        self.assertEqual(trajectory.valid_fields,3)
        self.assertEqual((point.position.x,point.position.y,point.position.z),(2,1,-3))
        self.assertEqual((point.velocity.x,point.velocity.y,point.velocity.z),(5,4,-6))
        self.assertEqual((point.acceleration.x,point.acceleration.y,point.acceleration.z),(8,7,-9))
        with self.assertRaises(ValueError):
            SwarmFormationAdapter._message(node,[[[1,2,3],[2,3,4]]],100.2,[],None)

    def test_native_fleet_completion_marks_one_final_segment(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from uav_swarm_formation_adapter.node import SwarmFormationAdapter
        node=Mock();node.vehicles=['drone0']
        node.config={'task':{'task_id':'final'},'planner':{'parameters':{'sample_dt':.1,'valid_from_lead':.2}}}
        node.get_clock.return_value.now.return_value=SimpleNamespace(nanoseconds=100_000_000_000)
        message=SwarmFormationAdapter._message(node,[[[1,2,3],[1,2,3]]],100.2,
            [[[0,0,0],[0,0,0]]],[[[0,0,0],[0,0,0]]],True)
        self.assertTrue(message.is_final)


if __name__ == "__main__":
    unittest.main()
