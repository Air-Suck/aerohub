import os
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
UPSTREAM = ROOT / "src/planner/yopo/upstream/YOPO"
MODEL = ROOT / "src/inference/models/yopo"

if (UPSTREAM / "policy/state_transform.py").is_file():
    sys.path.insert(0, str(UPSTREAM))
    from config.config import cfg
    from policy.state_transform import StateTransform

sys.path.insert(0, str(MODEL))
from processing import YopoProcessing


@unittest.skipUnless((UPSTREAM / "policy/state_transform.py").is_file(), "YOPO upstream is not initialized")
class YopoHoumoContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg["train"] = False
        cfg["velocity"] = float(os.environ.get("YOPO_VELOCITY", "6.0"))
        cls.reference = StateTransform()
        cls.houmo = YopoProcessing.from_file(
            MODEL / "houmo/artifacts/yopo-runtime.npz", velocity=cfg["velocity"]
        )

    def test_observation_preparation_matches_upstream(self):
        state = np.array(
            [[1.2, -0.3, 0.2, 0.4, -0.2, 0.1, 10.0, -2.0, 0.5]],
            dtype=np.float32,
        )
        expected = self.reference.prepare_input_cpu(
            self.reference.normalize_obs_cpu(state)
        )
        actual = self.houmo.prepare_observation(state[0])
        np.testing.assert_allclose(actual, expected, rtol=1e-6, atol=1e-6)

    def test_candidate_decoding_matches_upstream(self):
        rng = np.random.default_rng(7)
        predictions = rng.uniform(-1.0, 1.0, (1, 9, 3, 5)).astype(np.float32)
        scores = rng.uniform(0.0, 2.0, (1, 3, 5)).astype(np.float32)
        flat = predictions.reshape(9, -1).T
        lattice_id = np.arange(len(flat) - 1, -1, -1)
        expected = self.reference.pred_to_endstate_cpu(flat, lattice_id)
        endstates, actual_scores, best = self.houmo.decode(predictions, scores)
        np.testing.assert_allclose(endstates, expected, rtol=1e-5, atol=1e-6)
        np.testing.assert_allclose(actual_scores, scores.reshape(-1))
        self.assertEqual(best, int(scores.reshape(-1).argmin()))


if __name__ == "__main__":
    unittest.main()
