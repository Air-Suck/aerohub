"""Official high-speed crop/nearest sampling, before lossless ROS transport.

Normalization is pointwise and commutes with this selection; the model owns
inverse-depth normalization and max pooling. Source calibration stays on ROS.
"""
import copy
import numpy as np


def sample_high_speed(image):
    if (image.height, image.width) != (480, 640) or image.encoding != '32FC1':
        raise ValueError('high-speed camera requires 640x480 32FC1 meters')
    if image.step < 640 * 4 or len(image.data) != 480 * image.step:
        raise ValueError('invalid source depth layout')
    raw = np.ndarray((480, 640), dtype='>f4' if image.is_bigendian else '<f4',
                     buffer=bytes(image.data), strides=(image.step, 4))
    if not np.isfinite(raw).all() or (raw < 0).any():
        raise ValueError('invalid source depth')
    # Exactly Python round and PyTorch interpolate(mode=nearest) indices.
    top = round((480 - 480 * .82) / 2)
    left = round((640 - 640 * .82) / 2)
    rows = top + np.arange(36) * (480 - 2 * top) // 36
    cols = left + np.arange(48) * (640 - 2 * left) // 48
    selected = raw[np.ix_(rows, cols)].astype('<f4')
    selected[selected == 0] = 24.
    output = copy.deepcopy(image)
    output.height, output.width, output.step = 36, 48, 48 * 4
    output.is_bigendian = False
    output.data = selected.tobytes()
    return output
