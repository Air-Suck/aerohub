import unittest
from uav_planner_framework.goal_assignment import match_goal_slots
from uav_planner_framework.registry import ConfigError


class GoalAssignmentTest(unittest.TestCase):
    def setUp(self):
        self.slots={'drone0':(0.,-8.,-3.),'drone1':(-8.,0.,-3.),'drone2':(8.,0.,-3.),'drone3':(0.,8.,-3.)}
        self.endpoints=dict(self.slots)
        self.endpoints['drone1'],self.endpoints['drone2']=self.endpoints['drone2'],self.endpoints['drone1']

    def test_fixed_default_rejects_permutation(self):
        with self.assertRaises(ConfigError):match_goal_slots(self.endpoints,self.slots)

    def test_optimized_binding_keeps_vehicle_ids_and_slot_permutation(self):
        self.assertEqual({'drone0':'drone0','drone1':'drone2','drone2':'drone1','drone3':'drone3'},
                         match_goal_slots(self.endpoints,self.slots,'optimize'))
        self.assertEqual((8.,0.,-3.),self.endpoints['drone1'])

    def test_duplicate_slot_is_rejected(self):
        self.endpoints['drone1']=self.endpoints['drone0']
        with self.assertRaises(ConfigError):match_goal_slots(self.endpoints,self.slots,'optimize')

    def test_unrequested_or_nonfinite_goal_is_rejected(self):
        for value in ((999.,0.,-3.),(float('nan'),0.,-3.)):
            self.endpoints['drone1']=value
            with self.assertRaises(ConfigError):match_goal_slots(self.endpoints,self.slots,'optimize')

    def test_unknown_policy_or_wrong_vehicle_set_is_rejected(self):
        with self.assertRaises(ConfigError):match_goal_slots(self.endpoints,self.slots,'ignore')
        self.endpoints.pop('drone0')
        with self.assertRaises(ConfigError):match_goal_slots(self.endpoints,self.slots,'optimize')

    def test_ambiguous_slots_are_rejected(self):
        slots={'a':(0.,0.,0.),'b':(.0001,0.,0.)}
        with self.assertRaises(ConfigError):match_goal_slots(slots,slots,'optimize')
