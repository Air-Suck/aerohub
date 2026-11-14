"""ROS-independent conversion and validation, including row padding/endian handling."""

import math
import struct


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError("invalid observation timestamp")
    return stamp.sec * 1000000000 + stamp.nanosec


def request_payload(request, now_ns, max_age_s, max_skew_s, max_reference_future_s=0.3):
    if not request.state_header.frame_id or not request.depth.header.frame_id:
        raise ValueError("depth and state frame_id are required")
    image_time = stamp_ns(request.depth.header.stamp)
    state_time = stamp_ns(request.observation_header.stamp)
    reference_time = stamp_ns(request.state_header.stamp)
    if not request.observation_header.frame_id:
        raise ValueError("observation frame is required")
    if reference_time-now_ns > max_reference_future_s * 1e9 or (max_age_s > 0 and now_ns-reference_time > max_age_s * 1e9):
        raise ValueError("reference epoch outside bounded planning window")
    if abs(image_time - state_time) > max_skew_s * 1e9:
        raise ValueError("depth/state timestamps exceed synchronization tolerance")
    if any(stamp > now_ns or (max_age_s > 0 and now_ns-stamp > max_age_s*1e9) for stamp in (image_time, state_time)):
        raise ValueError("observation is stale or in the future")
    image = request.depth
    formats = {"32FC1": ("f", 4, 1.0), "16UC1": ("H", 2, 0.001)}
    if image.encoding not in formats:
        raise ValueError("depth encoding must be 32FC1 meters or 16UC1 millimeters")
    fmt, size, scale = formats[image.encoding]
    if not image.width or not image.height or image.width * image.height > 1048576:
        raise ValueError("depth must contain 1..1048576 pixels")
    if image.step < image.width * size or len(image.data) != image.step * image.height:
        raise ValueError("invalid depth row stride or data length")
    unpacker = struct.Struct((">" if image.is_bigendian else "<") + str(image.width) + fmt)
    rows = [[(value * scale if math.isfinite(value) else (20.0 if value > 0 else 0.0))
             for value in unpacker.unpack_from(image.data, row * image.step)] for row in range(image.height)]
    state = [value for vector in (request.velocity, request.acceleration, request.relative_goal) for value in (vector.x, vector.y, vector.z)]
    if any(not math.isfinite(value) for value in state):
        raise ValueError("state must be finite")
    if len(request.request_id) > 128:
        raise ValueError("request_id exceeds 128 characters")
    return {"request_id": request.request_id, "depth": rows, "state": state}


def validate_result(result, request_id):
    if result["request_id"] != request_id:
        raise ValueError("worker returned a mismatched request_id")
    states, scores, best = result["endstates"], result["scores"], result["best_index"]
    if not states or len(states) != len(scores) or type(best) is not int or not 0 <= best < len(states):
        raise ValueError("invalid worker candidate dimensions/index")
    if any(len(state) != 9 for state in states):
        raise ValueError("each candidate must contain 9 values")
    values = [value for state in states for value in state] + scores + [result["duration_s"]]
    if any(not math.isfinite(value) for value in values) or result["duration_s"] <= 0:
        raise ValueError("invalid worker numeric output")
    if best != min(range(len(scores)), key=scores.__getitem__):
        raise ValueError("best_index does not select minimum cost")
    if not result["model_version"] or not result["device"]:
        raise ValueError("worker identity is missing")
