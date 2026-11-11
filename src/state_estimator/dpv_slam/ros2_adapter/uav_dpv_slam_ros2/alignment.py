from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Similarity:
    scale: float
    rotation: np.ndarray
    translation: np.ndarray
    rmse: float

    def position(self, point):
        return self.scale * (self.rotation @ np.asarray(point)) + self.translation

    def pose(self, pose):
        result = np.eye(4, dtype=np.float64)
        result[:3, :3] = self.rotation @ pose[:3, :3]
        result[:3, 3] = self.position(pose[:3, 3])
        return result


def estimate_similarity(source, target, minimum_baseline=1.0,
                        minimum_horizontal_baseline=0.0,
                        minimum_horizontal_excitation=0.0,
                        scale_bounds=(0.05, 20.0)):
    """Fit target = scale * rotation * source + translation with Umeyama."""
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("paired positions must have shape Nx3")
    if source.shape[0] < 5 or not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError("at least five finite position pairs are required")
    baseline = np.max(np.linalg.norm(target - target[0], axis=1))
    if baseline < minimum_baseline:
        raise ValueError("metric alignment baseline is too short")
    horizontal_baseline = np.max(np.linalg.norm(
        target[:, :2] - target[0, :2], axis=1))
    if horizontal_baseline < minimum_horizontal_baseline:
        raise ValueError("metric alignment horizontal baseline is too short")
    horizontal_centered = target[:, :2] - target[:, :2].mean(axis=0)
    horizontal_singular = np.linalg.svd(
        horizontal_centered, compute_uv=False) / np.sqrt(target.shape[0])
    if horizontal_singular[-1] < minimum_horizontal_excitation:
        raise ValueError("metric alignment horizontal excitation is too weak")
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    variance = np.sum(source_centered * source_centered) / source.shape[0]
    if variance <= 1.0e-12:
        raise ValueError("visual trajectory has no usable translation")
    covariance = target_centered.T @ source_centered / source.shape[0]
    u, singular, vt = np.linalg.svd(covariance)
    sign = np.ones(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sign[-1] = -1
    rotation = u @ np.diag(sign) @ vt
    scale = float(np.sum(singular * sign) / variance)
    if not scale_bounds[0] <= scale <= scale_bounds[1]:
        raise ValueError("estimated monocular scale is outside configured bounds")
    translation = target_mean - scale * rotation @ source_mean
    fitted = (scale * (rotation @ source.T)).T + translation
    rmse = float(np.sqrt(np.mean(np.sum((fitted - target) ** 2, axis=1))))
    return Similarity(scale, rotation, translation, rmse)
