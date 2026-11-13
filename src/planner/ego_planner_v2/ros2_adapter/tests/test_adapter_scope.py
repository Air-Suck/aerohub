"""Offline ownership regression; does not require ROS or access a vehicle."""
import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'bridge'))
from uav_ego_planner_adapter.scope import AdapterScope


class ScopeTests(unittest.TestCase):
    def setUp(self):
        self.config={'vehicles':['alpha','beta'],'planner_ids':{'alpha':0,'beta':1},
            'deployment':{'alpha':{'namespace':'drone0','px4_origin_in_fleet_ned':[0,0,0]},
                          'beta':{'namespace':'drone1','px4_origin_in_fleet_ned':[4,0,0]}},
            'task':{'task_id':'shared'},'limits':{'velocity':.23}}

    def test_legacy_endpoints_and_fleet_preserved(self):
        s=AdapterScope(self.config)
        self.assertEqual(s.config,self.config)
        self.assertEqual(s.api('cancel'),'/ego/cancel')
        self.assertEqual(s.planner('trajectory'),'/planner/trajectory')
        self.assertEqual(s.native('beta','goal'),'/ego/beta/goal')

    def test_local_execution_and_readonly_peer_metadata(self):
        before=copy.deepcopy(self.config);s=AdapterScope(self.config,'beta')
        self.assertEqual(s.config['vehicles'],['beta'])
        self.assertEqual(s.config['planner_ids'],{'beta':1})
        self.assertEqual(set(s.config['deployment']),{'beta'})
        self.assertEqual(s.fleet['vehicles'],['alpha','beta'])
        s.config['deployment']['beta']['px4_origin_in_fleet_ned'][0]=99
        self.assertEqual(self.config,before)
        self.assertEqual(s.fleet,before)

    def test_names_do_not_depend_on_vehicle_label(self):
        s=AdapterScope(self.config,'beta')
        self.assertEqual(s.api('goal'),'/drone1/ego/goal')
        self.assertEqual(s.planner('failure'),'/drone1/planner/failure')
        self.assertEqual(s.native('beta','odom'),'/drone1/ego/native/odom')
        with self.assertRaises(ValueError):s.native('alpha','goal')

    def test_control_endpoints_are_disjoint(self):
        def endpoints(v):
            s=AdapterScope(self.config,v)
            return {s.api(k) for k in ('goal','cancel','status')} | {
                s.planner(k) for k in ('trajectory','failure')} | {
                s.native(v,k) for k in ('goal','odom','points')}
        self.assertFalse(endpoints('alpha') & endpoints('beta'))

    def test_unknown_selection_rejected(self):
        for value in ('drone0','unknown',None,False):
            with self.subTest(value=value),self.assertRaises(ValueError):AdapterScope(self.config,value)

    def test_duplicate_id_or_namespace_rejected(self):
        for key in ('id','namespace'):
            c=copy.deepcopy(self.config)
            if key=='id':c['planner_ids']['beta']=0
            else:c['deployment']['beta']['namespace']='drone0'
            with self.assertRaises(ValueError):AdapterScope(c,'alpha')

    def test_ambiguous_namespace_and_id_rejected(self):
        for ns in ('/drone0','drone0/child','drone-0',''):
            c=copy.deepcopy(self.config);c['deployment']['alpha']['namespace']=ns
            with self.assertRaises(ValueError):AdapterScope(c,'alpha')
        for identity in (-1,True,0.5):
            c=copy.deepcopy(self.config);c['planner_ids']['alpha']=identity
            with self.assertRaises(ValueError):AdapterScope(c,'alpha')

if __name__=='__main__':unittest.main()
