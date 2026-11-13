"""Report-contract regression; synthetic fixtures are not flight evidence."""
import json
import ast
import time
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from analyze import analyze


class StopContractTests(unittest.TestCase):
    def test_already_finished_action_may_reject_cancel_before_result_callback(self):
        # Exercise the actual Mission shutdown method without importing ROS.
        tree=ast.parse(Path(__file__).with_name('mission.py').read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='confirm_execution_stop')
        namespace={'time':time,'ControlMode':SimpleNamespace(POSITION=1)}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'mission-stop-method','exec'),namespace)
        class Future:
            def __init__(self,data):self.data=data;self.ready=False
            def done(self):return self.ready
            def result(self):return self.data
        finished=Future(SimpleNamespace(status=4))
        canceled=Future(SimpleNamespace(goals_canceling=[]))
        handle=SimpleNamespace(accepted=True,get_result_async=lambda:finished,cancel_goal_async=lambda:canceled)
        def wait(predicate,timeout):
            self.assertGreater(timeout,0)
            if hasattr(predicate,'__self__') and isinstance(predicate.__self__,Future):predicate.__self__.ready=True
            self.assertTrue(predicate())
        mission=SimpleNamespace(terminal_handle=handle,action_handle=None,wait=wait,delay=lambda duration:None,
            counts={'external_acro/control':0},result={},
            controller=SimpleNamespace(input_control_mode=SimpleNamespace(control_mode=1)))
        namespace['confirm_execution_stop'](mission)
        self.assertTrue(mission.result['execution_stop_confirmed'])
        # A rejected cancel without any terminal result must still fail.
        finished.ready=False;mission.result={}
        def never_finishes(predicate,timeout):
            if getattr(predicate,'__self__',None) is finished:raise TimeoutError('stopping')
            wait(predicate,timeout)
        mission.wait=never_finishes
        with self.assertRaises(TimeoutError):namespace['confirm_execution_stop'](mission)
        self.assertFalse(mission.result.get('execution_stop_confirmed',False))
        # Receiving a terminal Action result does not permit continued policy output.
        finished.ready=True;mission.wait=wait;mission.result={}
        def continued_output(duration):mission.counts['external_acro/control']+=1
        mission.delay=continued_output
        with self.assertRaisesRegex(RuntimeError,'policy output continued'):
            namespace['confirm_execution_stop'](mission)

    def report(self, **changes):
        result=dict(success=True,stop_contract_version=2,case='super-forest-route',
            goal_distance_m=.1,tilt_max_rad=.5,collision_end={'has_collided':False},
            landed_and_disarmed=True,terminal_settled=True,stop_wall_s=12.,
            arrival_wall_s=75.,arrival_wall_budget_s=300.,low_speed_dwell_sim_s=2.1,
            stop_speed_mps=.1,execution_stop_confirmed=True,execution_stop_wall_s=.4)
        result.update(changes)
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory)
            for name, data in [('result',result),('first-observation',{'depth':[[2.]*48 for _ in range(36)]}),
                ('cleanup',{'owned_resources_stopped':True})]:
                (out/(name+'.json')).write_text(json.dumps(data))
            (out/'policy.jsonl').write_text(json.dumps(dict(kind='policy',device='cuda',latency=.01))+'\n')
            return analyze(out)

    def test_physical_settling_over_ten_seconds_is_not_exit_ack_timeout(self):
        self.assertTrue(self.report()['success'])

    def test_historical_combined_timeout_is_not_relabelled(self):
        self.assertFalse(self.report(stop_contract_version=1)['criteria']['bounded_stop'])

    def test_each_stage_retains_its_own_deadline_and_dwell(self):
        for change in [dict(arrival_wall_s=301.),dict(low_speed_dwell_sim_s=1.9),
                       dict(execution_stop_wall_s=10.1),dict(execution_stop_confirmed=False),
                       dict(terminal_settled=False),dict(stop_speed_mps=.31),dict(goal_distance_m=1.6),
                       dict(expected_fault=True)]:
            with self.subTest(change=change):self.assertFalse(self.report(**change)['success'])


if __name__=='__main__':unittest.main()
