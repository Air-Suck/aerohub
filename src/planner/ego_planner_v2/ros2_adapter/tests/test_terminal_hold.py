"""Frozen observed short-tail regression and conservative rejection boundaries."""
import copy
import json
from pathlib import Path
import unittest
import numpy as np
from traj_utils.msg import PolyTraj
from uav_ego_planner_adapter.conversion import (Polynomial, segment_for, equivalent_final_tail,
    nanoseconds, set_ns, validate_terminal_hold)

DATA=json.loads((Path(__file__).parent/'data/short_final_tail.json').read_text())

def message(row):
    msg=PolyTraj(drone_id=1,order=5)
    set_ns(msg.start_time,row['start_ns'])
    for field in ('duration','coef_x','coef_y','coef_z'):setattr(msg,field,row[field])
    return msg

def hermite(left,right,t):
    # Independently solve the six endpoint constraints in normalized time.
    h=(nanoseconds(right.time_from_start)-nanoseconds(left.time_from_start))/1e9
    u=t/h
    p0=np.array([left.position.y,left.position.x,-left.position.z])
    p1=np.array([right.position.y,right.position.x,-right.position.z])
    v0=np.array([left.velocity.y,left.velocity.x,-left.velocity.z])
    v1=np.array([right.velocity.y,right.velocity.x,-right.velocity.z])
    a0=np.array([left.acceleration.y,left.acceleration.x,-left.acceleration.z])
    a1=np.array([right.acceleration.y,right.acceleration.x,-right.acceleration.z])
    c0=p0;c1=v0*h;c2=a0*h*h/2
    high=np.linalg.solve([[1,1,1],[3,4,5],[6,12,20]],
                        np.array([p1-c0-c1-c2,v1*h-c1-2*c2,a1*h*h-2*c2]))
    coeff=np.array([c0,c1,c2,*high]).T
    return [np.array([np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(c,d)) for c in coeff])/h**d for d in range(3)]

class TerminalHoldTests(unittest.TestCase):
    def setUp(self):
        self.msg=message(DATA['candidate']);self.source=Polynomial(self.msg,1,.2415,.3675)
        self.config={'vehicles':['drone1'],'planner':{'parameters':{'valid_from_lead':.4}},'task':{'task_id':'frozen-tail'}}
        self.goal={'drone1':DATA['goal']}
        self.now=self.source.end_ns-DATA['remaining_at_failure_ns']

    def segment(self,source=None,now=None,goal=None):
        return segment_for(self.config,{'drone1':source or self.source},goal or self.goal,self.now if now is None else now)

    def test_frozen_failure_is_non_equivalent_and_old_coverage_impossible(self):
        previous=Polynomial(message(DATA['previous']),1,.2415,.3675)
        self.assertFalse(equivalent_final_tail(previous,self.source,DATA['goal'],self.now,.0175))
        self.assertLess(self.source.end_ns-self.now,150_000_000)
        self.assertGreater(self.source.end_ns-self.now,50_000_000)

    def test_observed_tail_preserves_source_and_budgets(self):
        before=copy.deepcopy(self.source.coefficients)
        s=self.segment();start=nanoseconds(s.valid_from);points=s.vehicle_trajectories[0].points
        self.assertTrue(s.is_final)
        self.assertGreaterEqual(start,self.now+50_000_000)
        self.assertEqual(start+nanoseconds(points[-1].time_from_start),self.source.end_ns+100_000_000)
        self.assertGreaterEqual(nanoseconds(points[-1].time_from_start),100_000_000)
        self.assertIn(self.source.end_ns,[start+nanoseconds(p.time_from_start) for p in points])
        np.testing.assert_array_equal(self.source.coefficients,before)
        for left,right in zip(points,points[1:]):
            absolute_left=start+nanoseconds(left.time_from_start)
            h=(nanoseconds(right.time_from_start)-nanoseconds(left.time_from_start))/1e9
            for u in np.linspace(0,1,101):
                absolute=absolute_left+round(float(u*h)*1e9)
                expected=self.source.sample(min(absolute,self.source.end_ns))
                if absolute>self.source.end_ns:expected=[expected[0],np.zeros(3),np.zeros(3)]
                actual=hermite(left,right,u*h)
                for a,b in zip(actual,expected):self.assertLess(np.linalg.norm(a-b),1e-4)

    def test_dispatch_exhausted_still_rejects(self):
        for remaining in (50_999_999,50_000_000,49_000_000,0,-1):
            with self.subTest(remaining=remaining),self.assertRaises(ValueError):self.segment(now=self.source.end_ns-remaining)

    def test_short_but_dispatchable_boundary(self):
        # Even a 1 ms moving suffix remains on the original curve. The rest is
        # after the original endpoint; it does not rescale the suffix to 100 ms.
        s=self.segment(now=self.source.end_ns-51_000_000)
        self.assertEqual(nanoseconds(s.valid_from),self.source.end_ns-1_000_000)
        points=s.vehicle_trajectories[0].points
        for u in np.linspace(0,1,101):
            expected=self.source.sample(self.source.end_ns-1_000_000+round(float(u)*1_000_000))
            actual=hermite(points[0],points[1],float(u)*.001)
            for a,b in zip(actual,expected):self.assertLess(np.linalg.norm(a-b),1e-4)

    def test_nonfinal_and_wrong_goal_do_not_get_hold(self):
        with self.assertRaises(ValueError):self.segment(goal={'drone1':[2.,4.,2.]})
        msg=copy.deepcopy(self.msg);msg.coef_x[-2]+=.01
        source=Polynomial(msg,1,.2415,.3675)
        self.assertFalse(source.final(DATA['goal']))
        with self.assertRaises(ValueError):self.segment(source=source)

    def test_wire_derivatives_cannot_be_amplified_by_hold(self):
        msg=copy.deepcopy(self.msg);msg.coef_x[-2]+=0.00009
        source=Polynomial(msg,1,.2415,.3675)
        self.assertTrue(source.final(DATA['goal']))
        with self.assertRaisesRegex(ValueError,'not stationary'):validate_terminal_hold(source)

    def test_sufficient_coverage_does_not_change_existing_end(self):
        previous=Polynomial(message(DATA['previous']),1,.2415,.3675)
        s=self.segment(source=previous,now=previous.end_ns-300_000_000)
        self.assertEqual(nanoseconds(s.valid_from),previous.end_ns-100_000_000)
        self.assertEqual(nanoseconds(s.valid_from)+nanoseconds(s.vehicle_trajectories[0].points[-1].time_from_start),previous.end_ns)

    def test_observed_float32_knot_is_not_interpolated_across(self):
        row=json.loads((Path(__file__).parent/'data/short_tail_wire_knot.json').read_text())
        source=Polynomial(message(row['candidate']),1,.2415,.3675)
        s=self.segment(source=source,now=row['now_ns'])
        start=nanoseconds(s.valid_from);points=s.vehicle_trajectories[0].points
        last_start=source.start_ns+round(float(source.ends[-2])*1e9)
        self.assertEqual(start,last_start+1)
        self.assertGreaterEqual(start-row['now_ns'],50_000_000)
        self.assertEqual(len(points),3)
        h=(nanoseconds(points[1].time_from_start)-nanoseconds(points[0].time_from_start))/1e9
        for u in np.linspace(0,1,101):
            expected=source.sample(start+round(float(u*h)*1e9))
            actual=hermite(points[0],points[1],float(u*h))
            for a,b in zip(actual,expected):self.assertLess(np.linalg.norm(a-b),1e-4)

if __name__=='__main__':unittest.main()
