import numpy as np
import pytest

from uav_dpv_slam_ros2.engine import keyframe_stamp


def test_keyframe_stamp_preserves_original_input_time():
    inputs = [100, 200, 300, 400, 500]
    indices = np.array([0, 1, 4, 0, 0])
    assert keyframe_stamp(inputs, indices, 3) == 500


def test_keyframe_stamp_rejects_invalid_index():
    with pytest.raises(ValueError, match="out of range"):
        keyframe_stamp([100], np.array([3]), 1)
