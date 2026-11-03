#!/usr/bin/env python3
"""Encode Gazebo PPM frames into a broadly playable MP4."""

import argparse
import bisect
import json
import math
from pathlib import Path

import cv2


CAMERA_X = -2.0
CAMERA_Z = 45.0
HORIZONTAL_FOV = 1.25
FORMATION_EDGES = [
    (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
    (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 1),
]


def project(position, width, height):
    x, y, z = position
    depth = CAMERA_Z - z
    if depth <= 0.0:
        return None
    focal = width / (2.0 * math.tan(HORIZONTAL_FOV / 2.0))
    point = (
        round(width / 2.0 + (x - CAMERA_X) * focal / depth),
        round(height / 2.0 - y * focal / depth),
    )
    if not (0 <= point[0] < width and 0 <= point[1] < height):
        return None
    return point


def draw_formation(frame, points):
    layer = frame.copy()
    for begin, end in FORMATION_EDGES:
        if points[begin] is None or points[end] is None:
            continue
        cv2.line(layer, points[begin], points[end], (15, 15, 15), 3, cv2.LINE_AA)
        cv2.line(layer, points[begin], points[end], (76, 76, 230), 2, cv2.LINE_AA)
    cv2.addWeighted(layer, 0.80, frame, 0.20, 0, frame)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("frames", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--title", default="PX4 + Gazebo Harmonic")
    parser.add_argument("--trails", type=Path)
    args = parser.parse_args()

    paths = sorted(args.frames.glob("frame_*.ppm"))
    if not paths:
        raise SystemExit(f"No frames found in {args.frames}")
    first = cv2.imread(str(paths[0]))
    if first is None:
        raise SystemExit(f"Cannot read {paths[0]}")
    height, width = first.shape[:2]
    trace_times = []
    trace_positions = []
    if args.trails:
        metrics = json.loads(args.trails.read_text(encoding="utf-8"))
        for timestamp, positions in metrics.get("vehicle_traces", []):
            trace_times.append(timestamp)
            trace_positions.append(positions)
        if not trace_times:
            raise SystemExit(f"No vehicle_traces found in {args.trails}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # AVC/H.264 is supported by browsers, Windows, macOS, Android and iOS.
    # OpenCV's older mp4v output is valid but is not broadly supported.
    writer = cv2.VideoWriter(
        str(args.output), cv2.VideoWriter_fourcc(*"avc1"), args.fps, (width, height)
    )
    if not writer.isOpened():
        raise SystemExit("OpenCV could not open the H.264/AVC MP4 writer")

    for path in paths:
        frame = cv2.imread(str(path))
        if frame is None or frame.shape[:2] != (height, width):
            raise SystemExit(f"Invalid frame: {path}")
        if trace_times:
            current_trace = bisect.bisect_right(trace_times, path.stat().st_mtime) - 1
            if current_trace >= 0:
                points = [project(position, width, height)
                          for position in trace_positions[current_trace]]
                draw_formation(frame, points)
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (width, 58), (12, 18, 28), -1)
        cv2.addWeighted(overlay, 0.72, frame, 0.28, 0, frame)
        cv2.putText(frame, args.title, (22, 36), cv2.FONT_HERSHEY_SIMPLEX,
                    0.78, (255, 255, 255), 2, cv2.LINE_AA)
        writer.write(frame)
    writer.release()
    print(f"Encoded {len(paths)} H.264/AVC frames at {args.fps:g} fps: {args.output}")


if __name__ == "__main__":
    main()
