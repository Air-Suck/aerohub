"""Strict transport of the official swarm profile; no TF or image resizing."""

import math
import struct

from .yopo_contract import stamp_ns


def request_payload(request, now_ns, max_age_s, max_skew_s):
    if not request.request_id or len(request.request_id) > 128:
        raise ValueError("request_id must contain 1..128 characters")
    image = request.depth
    if not image.header.frame_id or not request.state_header.frame_id:
        raise ValueError("depth and state frames are required")
    image_time, state_time = stamp_ns(image.header.stamp), stamp_ns(request.state_header.stamp)
    if any(stamp > now_ns or now_ns - stamp > max_age_s * 1e9 for stamp in (image_time, state_time)):
        raise ValueError("actual observation is stale or in the future")
    if abs(image_time - state_time) > max_skew_s * 1e9:
        raise ValueError("depth/state timestamps exceed synchronization tolerance")
    if (image.height, image.width) != (36, 48):
        raise ValueError("official swarm profile requires 36x48 depth; resizing is not calibration")
    formats = {"32FC1": ("f", 4, 1.), "16UC1": ("H", 2, 0.001)}
    if image.encoding not in formats:
        raise ValueError("depth requires 32FC1 meters or 16UC1 millimeters")
    fmt, size, scale = formats[image.encoding]
    if image.step < 48 * size or len(image.data) != image.step * 36:
        raise ValueError("invalid depth stride/data length")
    unpacker = struct.Struct((">" if image.is_bigendian else "<") + "48" + fmt)
    rows = [[value * scale for value in unpacker.unpack_from(image.data, row * image.step)] for row in range(36)]
    if any(not math.isfinite(value) or value <= 0 for row in rows for value in row):
        raise ValueError("depth must contain positive finite measurements")
    # Fixed-size ROS float arrays can expose NumPy scalars in generated Python types.
    state = [float(value) for value in request.state]
    hidden = [float(value) for value in request.hidden_state]
    if len(state) != 10 or len(hidden) not in (0, 192) or any(not math.isfinite(value) for value in state + hidden):
        raise ValueError("state requires 10 finite values; hidden state requires 0 or 192")
    return dict(request_id=request.request_id, depth=rows, state=state, hidden_state=hidden or None)


def validate_result(result, request_id):
    if result["request_id"] != request_id:
        raise ValueError("mismatched request_id")
    if len(result["action"]) != 6 or len(result["hidden_state"]) != 192:
        raise ValueError("invalid action/hidden dimensions")
    if any(not math.isfinite(value) for value in result["action"] + result["hidden_state"]):
        raise ValueError("nonfinite model output")
    if not result["model_version"] or result["device"] not in ("cpu", "cuda"):
        raise ValueError("missing model identity/device")
