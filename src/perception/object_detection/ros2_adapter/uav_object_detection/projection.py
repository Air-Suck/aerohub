# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS RGB-D measurement pairing and capture-time camera TF, without a target KF."""
import copy
from collections import deque
import math
import threading
import time

import numpy as np
from rclpy.clock import ClockType
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import TransformException
from uav_object_detection_interfaces.msg import ObjectDetection, ObjectDetectionFrame

from .measurement import locate


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def rotation(q):
    values = [q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1) > .02:
        raise ValueError('invalid unit quaternion')  # Same validation as stage2 geometry.rotation.
    x, y, z, w = np.asarray(values) / np.linalg.norm(values)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def depth_array(msg, scale_16u):
    kind = {'32FC1': 'f4', '16UC1': 'u2'}.get(msg.encoding)
    if kind is None or msg.width <= 0 or msg.height <= 0:
        raise ValueError('unsupported depth encoding/dimensions')
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + kind)
    row = msg.width * dtype.itemsize
    if msg.step < row or len(msg.data) != msg.height * msg.step:
        raise ValueError('invalid depth payload/stride')
    raw = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :row].copy()
    image = raw.view(dtype).reshape(msg.height, msg.width).astype(float)
    return image * scale_16u if msg.encoding == '16UC1' else image


def intrinsics(info):
    k = np.asarray(info.k)
    if (info.width <= 0 or info.height <= 0 or not np.isfinite(k).all() or min(k[0], k[4]) <= 0 or
            not np.isfinite(info.d).all() or np.any(np.abs(info.d) > 1e-9)):
        raise ValueError('missing/invalid calibration or distorted image')
    return np.array([k[0], k[4], k[2], k[5]])


class Projector:
    def __init__(self, buffer, output_frame, sync_tolerance=.05, transform_wait=.25,
                 scale_16u=.001, perspective=False, registered=False):
        # Defaults retain stage2 pairing and SUPER-derived TF-arrival wait; no age gate.
        if any(not math.isfinite(v) or v <= 0 for v in (sync_tolerance, transform_wait, scale_16u)):
            raise ValueError('invalid synchronization/depth configuration')
        self.buffer, self.output_frame = buffer, output_frame
        self.sync_ns, self.wait = round(sync_tolerance*1e9), transform_wait
        self.scale, self.perspective, self.registered = scale_16u, perspective, registered
        self.lock = threading.RLock()
        self.reset()

    def reset(self):
        with self.lock:
            self.depth = deque(maxlen=60)
            self.info = {'rgb': deque(maxlen=60), 'depth': deque(maxlen=60)}

    def on_depth(self, msg):
        if stamp_ns(msg.header) <= 0 or not msg.header.frame_id:
            raise ValueError('missing depth acquisition timestamp/frame')
        image = depth_array(msg, self.scale)
        with self.lock:
            if self.depth and stamp_ns(msg.header) <= stamp_ns(self.depth[-1][0].header):
                raise ValueError('duplicate/backward depth timestamp')
            self.depth.append((msg, image))

    def on_info(self, kind, msg):
        try:
            intrinsics(msg)
            if not msg.header.frame_id:
                raise ValueError('missing calibration frame')
        except ValueError:
            with self.lock:self.info[kind].clear()
            raise
        with self.lock:
            self.info[kind].append(copy.deepcopy(msg))

    def _info_at(self, kind, stamp):
        candidates = [m for m in self.info[kind] if stamp_ns(m.header) <= stamp]
        if not candidates:
            raise ValueError('missing '+kind+' CameraInfo at acquisition time')
        return max(candidates, key=lambda m: stamp_ns(m.header))

    def project(self, frame, token, active):
        result = ObjectDetectionFrame(image_header=frame.header, request_id=frame.request_id,
            generation=frame.generation, tool_generation=token, worker_generation=frame.worker_generation,
            model_version=frame.model_version, timings_json=frame.timings_json)
        result.header.frame_id = self.output_frame
        result.detections = [ObjectDetection(detection=d) for d in frame.detections]
        try:
            if not self.registered:
                raise ValueError('aligned RGB/depth must be explicitly configured: registered_depth')
            rgb_ns = stamp_ns(frame.header)
            with self.lock:
                if not self.depth:
                    raise ValueError('missing synchronized depth')
                depth_msg, depth = min(self.depth, key=lambda pair: abs(stamp_ns(pair[0].header)-rgb_ns))
                depth_ns = stamp_ns(depth_msg.header)
                if abs(depth_ns-rgb_ns) > self.sync_ns:
                    raise ValueError('depth synchronization tolerance exceeded')
                rgb_info, depth_info = self._info_at('rgb', rgb_ns), self._info_at('depth', depth_ns)
            result.depth_header = depth_msg.header
            result.header = Header(stamp=depth_msg.header.stamp, frame_id=self.output_frame)
            for info in (rgb_info, depth_info):
                if ((info.width, info.height) != (frame.width, frame.height) or
                        info.header.frame_id != frame.header.frame_id):
                    raise ValueError('registered RGB/depth camera frame/dimensions mismatch')
            if depth_msg.header.frame_id != frame.header.frame_id or depth.shape != (frame.height, frame.width):
                raise ValueError('depth frame/dimensions mismatch')
            k = intrinsics(depth_info)
            if not np.allclose(intrinsics(rgb_info), k, atol=1e-5, rtol=0):
                raise ValueError('registered RGB/depth intrinsics mismatch')
            if self.perspective:
                yy, xx = np.indices(depth.shape)
                depth = depth / np.sqrt(1+((xx-k[2])/k[0])**2+((yy-k[3])/k[1])**2)
            # Query the full camera transform: includes the installation extrinsic once.
            # No settings.json, latest-TF fallback, extrapolation or truth lookup.
            started = time.monotonic()
            while True:
                if not active(token):
                    raise ValueError('CANCELLED_GENERATION')
                try:
                    transform = self.buffer.lookup_transform(self.output_frame, depth_msg.header.frame_id,
                        Time(nanoseconds=depth_ns, clock_type=ClockType.ROS_TIME))
                    break
                except TransformException as error:
                    if time.monotonic()-started >= self.wait:
                        raise ValueError('capture-time TF wait expired: '+str(error)) from error
                    time.sleep(.005)
            r = rotation(transform.transform.rotation)
            p = transform.transform.translation
            position = np.array([p.x, p.y, p.z])
            if not np.isfinite(position).all():
                raise ValueError('nonfinite camera position')
            for measured in result.detections:
                try:
                    d = measured.detection
                    mask = np.frombuffer(bytes(d.mask.data), np.uint8).reshape(frame.height, frame.width)
                    diagnostic = {}
                    point, _unused_kf_variance = locate(depth, d.xyxy, k, r, position,
                        (np.eye(3), np.zeros(3)), {'depth_method': 'gemini2_reference'}, diagnostic, mask)
                    measured.position.x, measured.position.y, measured.position.z = map(float, point)
                    measured.depth_m = diagnostic['depth_m']
                    measured.depth_spread_m = diagnostic['spread_m']
                    measured.valid_depth_pixels = diagnostic['valid_pixels']
                    measured.position_valid = True
                except ValueError as error:
                    measured.position_error = str(error)
            result.valid = all(d.position_valid for d in result.detections)
            result.error = '; '.join(d.position_error for d in result.detections if not d.position_valid)
        except ValueError as error:
            result.error = str(error)
            for d in result.detections:
                d.position_valid, d.position_error = False, result.error
        return result
