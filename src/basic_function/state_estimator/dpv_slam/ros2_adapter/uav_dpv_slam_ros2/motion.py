import math

import numpy as np


def matrix_from_pose(pose):
    q = np.array([
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w], dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm <= 0.0:
        raise ValueError("pose quaternion is invalid")
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return result


def quaternion_xyzw(rotation):
    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("rotation must be finite 3x3")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = [(rotation[2, 1] - rotation[1, 2]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s,
                (rotation[1, 0] - rotation[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(0.0, 1 + rotation[i, i] - rotation[j, j] - rotation[k, k])) * 2
        quat = np.zeros(4)
        quat[i] = 0.25 * s
        quat[j] = (rotation[j, i] + rotation[i, j]) / s
        quat[k] = (rotation[k, i] + rotation[i, k]) / s
        quat[3] = (rotation[k, j] - rotation[j, k]) / s
    quat = np.asarray(quat)
    return quat / np.linalg.norm(quat)
