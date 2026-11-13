# SPDX-License-Identifier: GPL-3.0-or-later
"""Image and detection validation shared by YOLO Inference and its Tool.

No ROS node, tracker, depth, TF or flight-control imports belong in this module.
"""
import math

import numpy as np


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def array_image(msg):
    if msg.encoding not in ('rgb8', 'bgr8') or not 0 < msg.width <= 2048 or not 0 < msg.height <= 2048:
        raise ValueError('expected RGB8/BGR8 image up to 2048x2048')
    if msg.step < msg.width * 3 or len(msg.data) != msg.height * msg.step:
        raise ValueError('invalid image payload/stride')
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    image = raw[:, :msg.width * 3].copy().reshape(msg.height, msg.width, 3)
    return image[:, :, ::-1].copy() if msg.encoding == 'rgb8' else image


def validate_request(request):
    if not request.request_id or len(request.request_id) > 128:
        raise ValueError('invalid request ID')
    if not 0 <= request.generation < 2**64:
        raise ValueError('invalid generation')
    image = request.image
    if not image.header.frame_id or len(image.header.frame_id) > 256:
        raise ValueError('missing/invalid optical frame')
    if image.header.stamp.sec < 0 or not 0 <= image.header.stamp.nanosec < 1_000_000_000 or seconds(image.header.stamp) <= 0:
        raise ValueError('missing/invalid source acquisition timestamp')
    return array_image(image)


def validate_frame(frame, request):
    image = request.image
    if (not frame.valid or frame.header != image.header or frame.request_id != request.request_id or
            frame.generation != request.generation or (frame.width, frame.height) != (image.width, image.height) or
            not frame.model_version):
        raise ValueError('inference response identity/geometry mismatch')
    for detection in frame.detections:
        xy = detection.xyxy
        if (len(xy) != 4 or not all(math.isfinite(x) for x in xy) or
                not 0 <= xy[0] < xy[2] <= frame.width or not 0 <= xy[1] < xy[3] <= frame.height or
                not math.isfinite(detection.confidence) or not 0 <= detection.confidence <= 1 or not detection.category):
            raise ValueError('invalid detection')
        mask = detection.mask
        if (mask.header != image.header or mask.encoding != 'mono8' or
                (mask.width, mask.height, mask.step) != (image.width, image.height, image.width) or
                len(mask.data) != image.width * image.height or
                not np.isin(np.frombuffer(bytes(mask.data), np.uint8), [0, 1]).all()):
            raise ValueError('invalid instance mask source/geometry/pixels')
