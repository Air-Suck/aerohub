"""Configuration contracts only; these checks do not certify flight safety."""
import json
import math
from pathlib import Path
import unittest

import yaml

APP = Path(__file__).resolve().parents[1]
ROOT = APP.parents[3]


class DemoConfigurationTest(unittest.TestCase):
    def test_sequence_targets_and_spawn_frames_are_consistent(self):
        sequence = yaml.safe_load((APP / 'sequence.yaml').read_text())
        self.assertEqual(['horizontal_line', 'diamond', 'vertical_line', 'square'],
                         [stage['name'] for stage in sequence['stages']])
        settings = json.loads((ROOT / 'simulator/airsim/scenarios/catora_town05/settings.json').read_text())
        deployment = yaml.safe_load((APP / 'deployment.yaml').read_text())
        for i, stage in enumerate(sequence['stages']):
            targets = stage['targets']
            self.assertEqual(set(deployment), set(targets))
            for axis, center in [('x', -50), ('y', 0), ('z', -3)]:
                self.assertAlmostEqual(center, sum(t[axis] for t in targets.values())/4)
            values = list(targets.values())
            for a in range(4):
                for b in range(a):
                    self.assertGreaterEqual(math.dist(list(values[a].values()),
                                                      list(values[b].values())), 2)
            if i:
                task = yaml.safe_load((APP / stage['task_config']).read_text())
                self.assertEqual(targets, task['targets'])
            else:
                for name, target in targets.items():
                    vehicle = settings['Vehicles'][name.replace('drone', 'Drone')]
                    self.assertEqual([vehicle['X'], vehicle['Y'], vehicle['Z']],
                                     deployment[name]['px4_origin_in_fleet_ned'])
                    self.assertEqual([target['x'], target['y']], [vehicle['X'], vehicle['Y']])

    def test_exactly_one_board_and_two_center_roles(self):
        plan = yaml.safe_load((APP / 'deployment-plan.yaml').read_text())
        self.assertEqual(1, sum(n['host'] == 'robopi' for n in plan['compute_nodes'].values()))
        self.assertEqual(3, sum(n['host'] == 'pc' for n in plan['compute_nodes'].values()))
        self.assertEqual(1, plan['coordination']['center_instance_count'])
        self.assertEqual({'board_center', 'board_worker'},
                         set(plan['coordination']['validation_cases']))
        self.assertEqual('pc', plan['workstation']['host'])


if __name__ == '__main__':
    unittest.main()
