# SPDX-License-Identifier: GPL-3.0-or-later
"""YOLO26n-seg image/schema handling on the public Aerohub TorchBackend."""
import base64
import hashlib
import json
import os
from pathlib import Path
import time
import cv2
import numpy as np
import torch
import ultralytics
from ultralytics import YOLO
from ultralytics.data.augment import LetterBox
from ultralytics.utils.nms import non_max_suppression
from ultralytics.utils.ops import scale_boxes, process_mask_native
import yaml
from aerohub_inference.backends.torch import TorchBackend


class TensorNetwork(torch.nn.Module):
    def __init__(self, name):
        super().__init__()
        self.network = YOLO(name+'.yaml').model.float()
        self.segment = name.endswith('-seg')
        # Explicit one-to-many for both families. Do not treat NMS-free output as raw.
        if name.startswith('yolo26'):
            self.network.model[-1].end2end = False

    def forward(self, image):
        output=self.network(image)[0]
        return dict(raw=output[0],proto=output[1]) if self.segment else output

    def load_state_dict(self, state_dict, strict=True, assign=False):
        result=super().load_state_dict(state_dict,strict=strict,assign=assign)
        # Same supported Conv/BN fusion used by the official inference predictor.
        # Fuse only after the public backend loads the original state_dict.
        self.network.fuse(verbose=False)
        return result


class Predictor:
    def __init__(self):
        config_path = Path(os.environ.get('YOLO_CONFIG', Path(__file__).with_name('worker.yaml'))).resolve()
        self.config = yaml.safe_load(config_path.read_text())
        c = self.config
        if 'YOLO_DEVICE' in os.environ:
            c['device'] = os.environ['YOLO_DEVICE']
        name, size = c['model_type'], c['input_size']
        if name != 'yolo26n-seg' or c['backend'] != 'ultralytics_pt':
            raise ValueError('unsupported model/backend')
        if c['device'] not in ('cpu','0') or size < 32 or size % 32 or not 0 < c['confidence'] < 1:
            raise ValueError('unsupported device/input size/confidence')
        source = Path(c['weights'])
        if not source.is_absolute():
            source = config_path.parent / source
        manifest = json.loads((source.parent/'manifest.json').read_text())
        expected = next(x for x in manifest if x['model']==name)
        if hashlib.sha256(source.read_bytes()).hexdigest() != expected['sha256']:
            raise ValueError('checkpoint hash mismatch')
        state = source.with_suffix('.state.pth')
        converted = next(x for x in json.loads((source.parent/'state-manifest.json').read_text()) if x['model']==name)
        if (converted['source_sha256'] != expected['sha256'] or
            hashlib.sha256(state.read_bytes()).hexdigest() != converted['state_sha256']):
            raise ValueError('state_dict provenance mismatch')
        torch.set_num_threads(int(os.environ.get('YOLO_THREADS','2')))
        model = TensorNetwork(name)
        self.segment=model.segment
        self.names = {int(k):v for k,v in converted['names'].items()}
        model.network.names = self.names
        if len(self.names)!=80 or any(k not in self.names.values() for k in c['class_map']):
            raise ValueError('COCO class map mismatch')
        count = sum((size//stride)**2 for stride in (8,16,32))
        outputs={'raw':dict(shape=[1,84+(32 if self.segment else 0),count],dtype='float32')}
        if self.segment:
            if model.network.model[-1].nm!=32:raise ValueError('unsupported segmentation prototype count')
            outputs['proto']=dict(shape=[1,32,size//4,size//4],dtype='float32')
        self.backend = TorchBackend(lambda:model, state, name,
            {'image':dict(shape=[1,3,size,size],dtype='float32')},
            outputs,
            device='cpu' if c['device']=='cpu' else 'cuda:0')
        self.backend.warmup({'image':np.zeros((1,3,size,size),np.float32)})
        self.generation = time.time_ns()
        version = dict(weights=expected['sha256'], state=converted['state_sha256'],
                       model=name, size=size, confidence=c['confidence'], classes=c['class_map'],
                       ultralytics=ultralytics.__version__, head='one-to-many+nms',fusion='official-ConvBN',
                       task='segment' if self.segment else 'detect',masks='official-retina-original-pixels-png' if self.segment else 'none')
        self.metadata = dict(self.backend.metadata, worker_generation=self.generation,
            model_version=hashlib.sha256(json.dumps(version,sort_keys=True).encode()).hexdigest(),
            processing=version, torch=torch.__version__, max_image=[2048,2048], encoding='png_bgr8')

    def infer(self, body):
        start = time.perf_counter()
        if body['encoding'] != 'png_bgr8':
            raise ValueError('only lossless PNG BGR8 supported')
        raw = base64.b64decode(body['image_base64'],validate=True)
        # Validate PNG IHDR before decoding to avoid unbounded decompression.
        if len(raw)<24 or raw[:8]!=b'\x89PNG\r\n\x1a\n':
            raise ValueError('invalid PNG')
        w,h = int.from_bytes(raw[16:20],'big'),int.from_bytes(raw[20:24],'big')
        if (w,h)!=(body['width'],body['height']) or not 0<w<=2048 or not 0<h<=2048:
            raise ValueError('PNG geometry does not match request')
        image = cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        if image is None or image.shape!=(h,w,3):
            raise ValueError('PNG decoding failed')
        decoded = time.perf_counter()
        size = self.config['input_size']
        gain = min(size/h,size/w)
        resized=[round(w*gain),round(h*gain)]
        pad=[(size-resized[0])/2,(size-resized[1])/2]
        letterbox=LetterBox(new_shape=(size,size),auto=False,stride=32)(image=image)
        tensor=np.ascontiguousarray(letterbox[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.
        prepared=time.perf_counter()
        outputs, backend_ms=self.backend.infer({'image':tensor})
        forwarded=time.perf_counter()
        raw_tensor=torch.from_numpy(outputs['raw'])
        boxes=non_max_suppression(raw_tensor,conf_thres=self.config['confidence'],iou_thres=.7,nc=80,max_det=300)[0]
        boxes[:,:4]=scale_boxes((size,size),boxes[:,:4],(h,w))
        wanted=torch.tensor([self.names[int(cls)] in self.config['class_map'] for cls in boxes[:,5]],dtype=torch.bool)
        boxes=boxes[wanted]
        masks=None
        if self.segment and len(boxes):
            masks=process_mask_native(torch.from_numpy(outputs['proto'][0]),boxes[:,6:],boxes[:,:4],(h,w))
        selected=[]
        for index,row in enumerate(boxes.numpy()):
            x1,y1,x2,y2,score,cls=row[:6]
            if not np.isfinite([x1,y1,x2,y2,score,cls]).all() or cls!=int(cls) or int(cls) not in self.names:
                raise ValueError('invalid detector output')
            category=self.config['class_map'].get(self.names[int(cls)])
            coords=np.clip([x1,y1,x2,y2],[0,0,0,0],[w,h,w,h]).tolist()
            if category and coords[2]>coords[0] and coords[3]>coords[1]:
                detection=dict(xyxy=coords,confidence=float(score),category=category)
                if masks is not None:
                    mask=masks[index].numpy().astype(np.uint8)
                    if not mask.any():continue  # Same empty-mask filtering as official predictor.
                    ok,png=cv2.imencode('.png',mask)
                    if not ok:raise ValueError('mask PNG encoding failed')
                    detection['mask_png_base64']=base64.b64encode(png).decode('ascii')
                selected.append(detection)
        finished=time.perf_counter()
        return dict(**{k:body[k] for k in ('request_id','stamp_ns','generation','frame','width','height')},
            model_version=self.metadata['model_version'],worker_generation=self.generation,boxes=selected,
            transform=dict(gain=gain,resized=resized,pad=pad,input_size=size,coordinates='original_pixels'),
            timings_ms=dict(decode=(decoded-start)*1000,preprocess=(prepared-decoded)*1000,
                backend_total=(forwarded-prepared)*1000,postprocess=(finished-forwarded)*1000,
                worker_total=(finished-start)*1000,**backend_ms))

    def close(self):
        self.backend.close()
