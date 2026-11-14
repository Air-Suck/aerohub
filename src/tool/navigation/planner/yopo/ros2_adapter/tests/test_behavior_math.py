"""Compare emitted dense points through the actual Behavior C++ quintic sampler."""
from pathlib import Path
import subprocess
import tempfile
import unittest
import numpy as np
from uav_yopo_planner.core import Curve, wrap


class BehaviorMathTest(unittest.TestCase):
    def test_dense_pva_and_yaw_full_domain(self):
        root = Path(__file__).resolve().parents[7]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory)/'sample')
            subprocess.run(['g++', '-std=c++17', '-O2', '-I'+str(root/'src/behavior/follow_path/include'),
                            str(Path(__file__).with_name('behavior_math.cpp')), '-o', binary], check=True)
            curve = Curve.create(10., np.array([[1.,2.,3.],[.2,-.5,.7],[.1,.3,-.2]]),
                                 np.array([[7.,-2.,6.],[.3,.4,.1],[-.3,.1,.2]]),
                                 1.6666666666667, 3.13, np.array([-10., -.1, 3.]))
            queries = np.linspace(0, curve.duration, 337)
            lines = [str(len(curve.times))+' '+str(len(queries))]
            for t in curve.times:
                pva, yaw = curve.sample(curve.start+t)
                lines.append(' '.join(map(str, [t, *pva.reshape(-1), yaw])))
            lines.extend(map(str, queries))
            output = subprocess.check_output([binary], input='\n'.join(lines), text=True)
            actual = np.array([list(map(float,line.split())) for line in output.splitlines()])
            expected = np.array([np.r_[curve.sample(10.+t)[0].reshape(-1), curve.sample(10.+t)[1]] for t in queries])
            np.testing.assert_allclose(actual[:,:9], expected[:,:9], atol=1e-8, rtol=1e-7)
            np.testing.assert_allclose([wrap(v) for v in actual[:,9]-expected[:,9]], 0., atol=1e-10)


if __name__ == '__main__':
    unittest.main()
