import numpy as np
import pytest

from uav_dpv_slam_ros2.engine import keyframe_stamp, resolve_pose, seed_torch


def test_keyframe_stamp_preserves_original_input_time():
    inputs = [100, 200, 300, 400, 500]
    indices = np.array([0, 1, 4, 0, 0])
    assert keyframe_stamp(inputs, indices, 3) == 500


def test_keyframe_stamp_rejects_invalid_index():
    with pytest.raises(ValueError, match="out of range"):
        keyframe_stamp([100], np.array([3]), 1)


class Transform:
    def __init__(self, values):
        self.values = tuple(values)

    def __mul__(self, other):
        return Transform(self.values + other.values)


def test_resolve_pose_matches_dpvo_delta_chain_order():
    keyframes = {2: Transform(("keyframe-2",))}
    deltas = {
        3: (2, Transform(("delta-3",))),
        4: (3, Transform(("delta-4",))),
    }
    assert resolve_pose(4, keyframes, deltas).values == (
        "delta-4", "delta-3", "keyframe-2")


def test_resolve_pose_rejects_missing_or_cyclic_delta_chain():
    with pytest.raises(ValueError, match="incomplete"):
        resolve_pose(3, {2: Transform(("keyframe",))}, {})
    with pytest.raises(ValueError, match="cycle"):
        resolve_pose(3, {}, {3: (4, Transform(())), 4: (3, Transform(()))})


class FakeTorch:
    def __init__(self):
        self.seeds = []

    def manual_seed(self, value):
        self.seeds.append(value)


def test_seed_torch_uses_upstream_evaluation_seed_contract():
    torch = FakeTorch()
    assert seed_torch(torch, 1234) == 1234
    assert torch.seeds == [1234]

    for invalid in (-1, True, 1.5, "1234"):
        with pytest.raises(ValueError, match="non-negative integer"):
            seed_torch(torch, invalid)
