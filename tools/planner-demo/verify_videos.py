#!/usr/bin/env python3
"""Check that demo MP4 files can be decoded and report basic metadata."""

import json
import sys
from pathlib import Path

import cv2


def inspect(path: Path):
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {path}")
    frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc_value = int(capture.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((fourcc_value >> (8 * i)) & 0xFF) for i in range(4))
    for index in (0, frames // 2, frames - 1):
        capture.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError(f"cannot decode frame {index} from {path}")
    capture.release()
    if frames < 100 or fps <= 0 or width < 640 or height < 360 or width * 9 != height * 16:
        raise RuntimeError(f"unexpected video metadata for {path}")
    if fourcc.lower() not in {"avc1", "h264"}:
        raise RuntimeError(f"incompatible codec {fourcc!r} in {path}; expected H.264/AVC")
    return {
        "file": str(path),
        "size_bytes": path.stat().st_size,
        "width": width,
        "height": height,
        "fps": fps,
        "codec": fourcc,
        "frames": frames,
        "duration_seconds": frames / fps,
    }


if __name__ == "__main__":
    print(json.dumps([inspect(Path(name)) for name in sys.argv[1:]], indent=2))
