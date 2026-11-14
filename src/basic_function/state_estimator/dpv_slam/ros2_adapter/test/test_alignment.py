import numpy as np
import pytest

from uav_dpv_slam_ros2.alignment import estimate_similarity


def test_similarity_recovers_metric_transform():
    source = np.array([
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0],
        [2.0, 1.0, 0.5], [3.0, 2.0, 1.0], [4.0, 2.0, 1.5]])
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    target = (2.5 * (rotation @ source.T)).T + [4.0, -2.0, 1.0]
    result = estimate_similarity(
        source, target, minimum_baseline=1.0,
        minimum_horizontal_baseline=1.0,
        minimum_horizontal_excitation=0.1)
    assert result.scale == pytest.approx(2.5)
    assert result.rotation == pytest.approx(rotation)
    assert result.translation == pytest.approx([4.0, -2.0, 1.0])
    assert result.rmse < 1.0e-10


def test_similarity_rejects_short_baseline():
    source = np.arange(15, dtype=float).reshape(5, 3) * 1.0e-4
    with pytest.raises(ValueError, match="baseline"):
        estimate_similarity(source, source, minimum_baseline=1.0)


def test_similarity_rejects_unobservable_horizontal_rotation():
    source = np.array([[0.0, 0.0, float(i)] for i in range(6)])
    with pytest.raises(ValueError, match="horizontal baseline"):
        estimate_similarity(
            source, source, minimum_baseline=1.0,
            minimum_horizontal_baseline=0.5)


def test_similarity_rejects_single_horizontal_direction():
    source = np.array([[float(i), 0.0, 0.1 * i] for i in range(6)])
    with pytest.raises(ValueError, match="horizontal excitation"):
        estimate_similarity(
            source, source, minimum_baseline=1.0,
            minimum_horizontal_baseline=1.0,
            minimum_horizontal_excitation=0.1)


def test_similarity_rejects_unreasonable_scale():
    source = np.array([[i, i % 2, i % 3] for i in range(5)], dtype=float)
    with pytest.raises(ValueError, match="scale"):
        estimate_similarity(source, source * 100.0, scale_bounds=(0.1, 10.0))
