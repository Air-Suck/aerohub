# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS-environment client; no Torch or Ultralytics imports."""
import base64
import json
import time
import uuid
import cv2
import numpy as np
from uav_inference.model_client import ModelClient, WorkerError


class HttpDetector:
    def __init__(self, config):
        self.config = config
        self.client = ModelClient(config['endpoint'], config['http_timeout'])
        self.metadata = self.client.request('/health/ready')
        if self.metadata['model_id'] != config['model_type']:
            raise ValueError('endpoint model differs from selected model configuration')
        self.model_version = self.metadata['model_version']
        self.worker_generation = self.metadata['worker_generation']
        self.timings = {}

    def detect(self, image, stamp_ns, frame, generation, request_id=None):
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or max(image.shape[:2]) > 2048:
            raise ValueError('expected BGR8 image up to 2048x2048')
        started = time.perf_counter()
        ok, png = cv2.imencode('.png', image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            raise ValueError('PNG encoding failed')
        text = base64.b64encode(png).decode('ascii')
        if len(text)>16777216:
            raise ValueError('encoded image exceeds 16 MiB limit')
        body = dict(request_id=request_id or uuid.uuid4().hex, stamp_ns=int(stamp_ns),frame=frame,
                    generation=int(generation),width=image.shape[1],height=image.shape[0],
                    encoding='png_bgr8',image_base64=text)
        encoded = time.perf_counter()
        try:
            result = self.client.request('/infer', body)
        except WorkerError as exc:
            raise ValueError(exc.code+': '+str(exc)) from exc
        returned = time.perf_counter()
        if any(result[k] != body[k] for k in ('request_id','stamp_ns','generation','frame','width','height')):
            raise ValueError('worker response identity/geometry mismatch')
        if result['model_version'] != self.model_version or result['worker_generation'] != self.worker_generation:
            raise ValueError('worker changed: restart detector and estimator for new generation')
        boxes = result['boxes']
        for box in boxes:
            coords = box['xyxy']
            if (len(coords)!=4 or not np.isfinite(coords).all() or
                not 0<=coords[0]<coords[2]<=body['width'] or not 0<=coords[1]<coords[3]<=body['height'] or
                not np.isfinite(box['confidence']) or not 0<=box['confidence']<=1 or
                box['category'] not in self.config['class_map'].values()):
                raise ValueError('invalid detection response')
            encoded_mask=box.get('mask_png_base64','')
            if self.config['model_type'].endswith('-seg') and not encoded_mask:
                raise ValueError('segmentation response missing instance mask')
            if encoded_mask:
                raw=base64.b64decode(encoded_mask,validate=True)
                if (len(raw)<24 or raw[:8]!=b'\x89PNG\r\n\x1a\n' or
                    (int.from_bytes(raw[16:20],'big'),int.from_bytes(raw[20:24],'big'))!=(body['width'],body['height'])):
                    raise ValueError('mask PNG geometry mismatch')
                mask=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_UNCHANGED)
                if mask is None or mask.shape!=image.shape[:2] or mask.dtype!=np.uint8 or not np.isin(mask,[0,1]).all():
                    raise ValueError('invalid binary instance mask')
                box['mask']=mask
        self.timings = dict(encode_ms=(encoded-started)*1000,http_ms=(returned-encoded)*1000,
                            client_total_ms=(returned-started)*1000,payload_bytes=len(png),
                            json_bytes=len(json.dumps(body).encode()),worker=result['timings_ms'])
        self.request_id = body['request_id']
        return boxes,result['transform']
