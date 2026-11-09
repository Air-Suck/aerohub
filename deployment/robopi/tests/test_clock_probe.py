import importlib.util
from pathlib import Path
import unittest

path=Path(__file__).resolve().parents[1]/'diagnostics/clock_probe.py'
spec=importlib.util.spec_from_file_location('clock_probe',path)
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


def row(at,offset=0.,board_step=0.,rtt=.02):
    ns=int(at*1e9);delta=int(offset*1e9);step=int(board_step*1e9)
    return {'pc_wall_mid_ns':ns,'pc_mono_mid_ns':ns,'board_wall_ns':ns+delta+step,
            'board_mono_ns':ns+delta,'offset_seconds':offset+board_step,
            'offset_lower_seconds':offset+board_step-rtt/2,
            'offset_upper_seconds':offset+board_step+rtt/2,'rtt_seconds':rtt}


class ClockProbeTest(unittest.TestCase):
    def test_wall_step_detected_independently_of_round_trip_offset(self):
        result=probe.assess([row(0),row(5,board_step=2.77)])
        self.assertTrue(result['wall_step_detected'])
        self.assertTrue(result['definite_offset_limit_violation'])

    def test_no_false_offset_verdict_when_uncertainty_spans_limit(self):
        result=probe.assess([row(0,offset=.5,rtt=.2),row(5,offset=.5,rtt=.2)])
        self.assertFalse(result['definite_offset_limit_violation'])
        self.assertFalse(result['wall_step_detected'])
        self.assertAlmostEqual(result['offset_change_uncertainty_seconds'],.2)

    def test_gradual_cross_host_rate_is_separate_from_wall_step(self):
        result=probe.assess([row(0),row(100,offset=.01)])
        self.assertFalse(result['wall_step_detected'])
        self.assertAlmostEqual(result['observed_rate_ppm'],100.)


if __name__=='__main__':unittest.main()
