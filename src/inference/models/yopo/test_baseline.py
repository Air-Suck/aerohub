"""Extract preprocessing from the frozen ROS entrypoint, without importing ROS."""
import ast
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
import cv2
import numpy as np
import torch
from adapter import Predictor, preprocess_depth

ROOT = Path(__file__).resolve().parents[4]


def original_preprocess(data, cfg):
    tree = ast.parse((ROOT/'src/planner/yopo/upstream/YOPO/test_yopo_ros.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'callback_depth')
    statements = []
    for stmt in method.body[2:]:
        if isinstance(stmt, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'time1' for t in stmt.targets):
            break
        statements.append(stmt)
    scope = dict(np=np, cv2=cv2, data=data,
                 self=SimpleNamespace(height=cfg['image_height'], width=cfg['image_width'], min_dis=.04, max_dis=20.))
    exec(compile(ast.Module(body=statements, type_ignores=[]), '<frozen ROS preprocessing>', 'exec'), scope)
    return scope['depth']


class BaselineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.predictor = Predictor()

    def test_reference_entrypoint_all_candidates(self):
        from scipy.spatial.transform import Rotation
        model = self.predictor.backend._model
        from config.config import cfg
        transform = model.state_transform
        rng = np.random.RandomState(43)
        for shape in [(96, 160), (48, 80), (7, 11)]:
            depth = rng.uniform(.01, 30, shape).astype(np.float32)
            depth.flat[:5] = [0., np.nan, np.inf, .02, 20.]
            data = SimpleNamespace(data=depth.tobytes(), height=shape[0], width=shape[1], encoding='32FC1')
            expected_depth = original_preprocess(data, cfg)
            np.testing.assert_array_equal(preprocess_depth(depth, 160, 96), expected_depth)
            rotation = Rotation.from_euler('ZYX', [.7, -.2, .3]).as_matrix()
            # Reference-mode nonzero velocity/acceleration, relative goal, nonidentity attitude.
            state_world = np.array([[1.3, -.4, .7], [.2, -.3, .4], [14., -9., 2.]])
            state = (state_world @ rotation).reshape(9).astype(np.float32)
            obs = transform.prepare_input_cpu(transform.normalize_obs_cpu(state[None]))
            with torch.inference_mode():
                prediction, score = model(torch.from_numpy(expected_depth).to(self.predictor.backend.device),
                                          torch.from_numpy(obs).to(self.predictor.backend.device))
            prediction = prediction.cpu().numpy().reshape(9, -1).T
            expected = transform.pred_to_endstate_cpu(prediction, np.arange(len(prediction)-1, -1, -1))
            # JSON transport represents invalid nonfinite pixels as zero (positive inf caps at 20m).
            transported = np.nan_to_num(depth, nan=0., posinf=20., neginf=0.)
            result = self.predictor.infer(dict(request_id='baseline', depth=transported.tolist(), state=state.tolist()))
            np.testing.assert_allclose(result['endstates'], expected, atol=1e-5, rtol=1e-4)
            np.testing.assert_allclose(result['scores'], score.cpu().numpy().reshape(-1), atol=1e-5, rtol=1e-4)
            self.assertEqual(result['best_index'], int(score.argmin()))


if __name__ == '__main__':
    unittest.main()
