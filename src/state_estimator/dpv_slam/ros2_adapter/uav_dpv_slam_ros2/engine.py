from dataclasses import dataclass

import numpy as np


def keyframe_stamp(input_stamps, keyframe_indices, keyframe_count):
    if keyframe_count < 1:
        raise ValueError("at least one keyframe is required")
    input_index = int(keyframe_indices[keyframe_count - 1])
    if not 0 <= input_index < len(input_stamps):
        raise ValueError("DPVO keyframe timestamp index is out of range")
    return int(input_stamps[input_index])


def seed_torch(torch_module, seed):
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("random seed must be a non-negative integer")
    torch_module.manual_seed(seed)
    return seed


def resolve_pose(index, keyframes, deltas, active=None):
    """Recover one input-frame pose using DPVO's terminate-time delta chain."""
    if index in keyframes:
        return keyframes[index]
    active = set() if active is None else active
    if index in active:
        raise ValueError("DPVO pose delta chain contains a cycle")
    if index not in deltas:
        raise ValueError("DPVO pose delta chain is incomplete")
    active.add(index)
    parent, transform = deltas[index]
    pose = transform * resolve_pose(int(parent), keyframes, deltas, active)
    active.remove(index)
    return pose


@dataclass(frozen=True)
class EngineResult:
    pose: object
    points: object
    initialized: bool
    frame_count: int
    stamp_ns: int


class DpvSlamEngine:
    def __init__(self, config_path, network_path, random_seed=1234):
        import torch
        from dpvo.config import cfg as default_cfg
        from dpvo.dpvo import DPVO

        seed_torch(torch, random_seed)
        self._torch = torch
        self._dpvo_type = DPVO
        self._config = default_cfg.clone()
        self._config.merge_from_file(config_path)
        self._config.LOOP_CLOSURE = True
        self._config.CLASSIC_LOOP_CLOSURE = False
        self._network_path = network_path
        self._slam = None
        self._input_stamps = []

    def process(self, stamp_ns, rgb, intrinsics):
        image = np.ascontiguousarray(rgb[..., ::-1])
        height, width = image.shape[:2]
        height, width = height - height % 16, width - width % 16
        image = image[:height, :width]
        tensor = self._torch.from_numpy(image).permute(2, 0, 1).cuda()
        camera = self._torch.as_tensor(intrinsics, dtype=self._torch.float32, device="cuda")
        if self._slam is None:
            self._slam = self._dpvo_type(
                self._config, self._network_path, ht=height, wd=width, viz=False)
        with self._torch.no_grad():
            self._input_stamps.append(int(stamp_ns))
            self._slam(float(stamp_ns) * 1.0e-9, tensor, camera)
        if not self._slam.is_initialized or self._slam.n < 1:
            return EngineResult(None, None, False, self._slam.counter, int(stamp_ns))
        from dpvo.lietorch import SE3
        keyframes = {
            int(self._slam.pg.tstamps_[index].item()):
            SE3(self._slam.pg.poses_[index])
            for index in range(self._slam.n)
        }
        current = resolve_pose(
            self._slam.counter - 1, keyframes, self._slam.pg.delta)
        pose = current.inv().data.detach().cpu().numpy()
        matrix = np.eye(4, dtype=np.float64)
        x, y, z, w = pose[3:7]
        matrix[:3, :3] = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        matrix[:3, 3] = pose[:3]
        points = self._slam.pg.points_[:self._slam.m].detach().cpu().numpy().copy()
        points = points[np.isfinite(points).all(axis=1)]
        return EngineResult(matrix, points, True, self._slam.counter, int(stamp_ns))
