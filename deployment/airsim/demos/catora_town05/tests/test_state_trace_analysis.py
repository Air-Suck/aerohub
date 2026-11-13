"""Offline trace arithmetic checks, not ROS/flight validation."""
import importlib.util
from pathlib import Path
import unittest

APP = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('state_trace_analysis', APP/'analyze_state_trace.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def sample(topic, seconds, stamp):
    return {'kind':'sample','topic':topic,'elapsed_s':seconds,
            'wall_ns':int((100+seconds)*1e9),'stamp_ns':int(stamp*1e9)}


class TraceAnalysisTest(unittest.TestCase):
    def test_stall_is_measured_separately_from_uninterrupted_clock(self):
        rows=[sample('/clock',0,0), sample('/clock',.02,.02),
              sample('/pose',0,0), sample('/pose',1.2,1.2)]
        result=module.analyze(rows,warmup=0)['topics']
        self.assertAlmostEqual(.02,result['/clock']['max_receipt_gap_s'])
        self.assertAlmostEqual(1.2,result['/pose']['max_receipt_gap_s'])
        gap=result['/pose']['largest_gap_interval']
        self.assertEqual(100000000000,gap['start_wall_ns'])
        self.assertEqual(101200000000,gap['end_wall_ns'])
        self.assertAlmostEqual(1.2,gap['source_stamp_step_s'])

    def test_warmup_cutoff_does_not_change_event_timestamps(self):
        event={'kind':'offboard_request','elapsed_s':2.,'wall_ns':102000000000}
        rows=[sample('/pose',1,1),sample('/pose',2,2),sample('/pose',4,4),event]
        result=module.analyze(rows,warmup=3.)
        self.assertEqual(1,result['topics']['/pose']['count'])
        self.assertEqual([event],result['events'])
        self.assertEqual(0,result['topics']['/pose']['max_receipt_gap_s'])
