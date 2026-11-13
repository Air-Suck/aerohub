"""Real-image CUDA comparison; does not start ROS, a simulator or flight control."""
import argparse
import base64
import hashlib
import importlib.util
import json
import os
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from adapter import Predictor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--legacy-stage2', type=Path, help='Optional migration-only comparison to the original local stage2')
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        raise ValueError('Cannot read input image')
    height, width = image.shape[:2]
    ok, png = cv2.imencode('.png', image)
    if not ok:
        raise ValueError('PNG encoding failed')
    body = dict(request_id='yolo26n-seg-baseline', stamp_ns=1234567890123456789, generation=17,
                frame='diagnostic_camera_optical', width=width, height=height, encoding='png_bgr8',
                image_base64=base64.b64encode(png).decode())
    migrated = Predictor()
    try:
        if migrated.metadata['device'] != 'cuda:0':
            raise AssertionError('This check requires actual CUDA; no CPU fallback')
        actual = migrated.infer(body)
        original = YOLO(str(Path(__file__).parent / 'artifacts/yolo26n-seg.pt'))
        original.model.model[-1].end2end = False
        reference = original.predict(image, imgsz=640, rect=False, retina_masks=True, conf=.35,
                                     iou=.7, max_det=300, device='0', verbose=False)[0]
        expected = [(box, mask) for box, mask in zip(reference.boxes, reference.masks.data)
                    if reference.names[int(box.cls)] in migrated.config['class_map']]
        if not expected or len(expected) != len(actual['boxes']):
            raise AssertionError('Expected a nonempty fixed-image detection match')
        comparisons = []
        for box, (truth_box, truth_mask) in zip(actual['boxes'], expected):
            mask = cv2.imdecode(np.frombuffer(base64.b64decode(box['mask_png_base64']), np.uint8), cv2.IMREAD_UNCHANGED).astype(bool)
            truth = truth_mask.cpu().numpy().astype(bool)
            iou = float(np.logical_and(mask, truth).sum() / max(1, np.logical_or(mask, truth).sum()))
            error = float(np.max(np.abs(np.array(box['xyxy']) - truth_box.xyxy[0].cpu().numpy())))
            # Preserve the previously used stage2 check_segmentation.py criteria.
            if iou < .995 or error > .01:
                raise AssertionError(f'Official comparison failed: {iou=}, {error=}')
            comparisons.append(dict(category=box['category'], mask_iou=iou, box_max_error_px=error))
        legacy_equal = None
        if args.legacy_stage2:
            path = args.legacy_stage2 / 'src/inference/models/yolo/adapter.py'
            spec = importlib.util.spec_from_file_location('legacy_yolo_adapter', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            previous = os.environ.get('YOLO_CONFIG')
            os.environ['YOLO_CONFIG'] = str(args.legacy_stage2 / 'config/yolo26n-seg.cuda.yaml')
            try:
                legacy = module.Predictor()
                try:
                    old = legacy.infer(body)
                    legacy_equal = old['boxes'] == actual['boxes'] and old['transform'] == actual['transform']
                    if not legacy_equal:
                        raise AssertionError('Migration changed boxes, scores, mask pixels or preprocessing')
                finally:
                    legacy.close()
            finally:
                if previous is None:os.environ.pop('YOLO_CONFIG', None)
                else:os.environ['YOLO_CONFIG'] = previous
        report = dict(passed=True, scope='real-image CUDA numerical comparison; not flight or live-camera accuracy',
            image_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),
            metadata=migrated.metadata, comparisons=comparisons, legacy_boxes_masks_transform_equal=legacy_equal)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report))
    finally:
        migrated.close()


if __name__ == '__main__':
    main()
