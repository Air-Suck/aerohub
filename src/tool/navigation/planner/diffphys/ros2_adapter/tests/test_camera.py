"""Compare deployment sampling + model preprocessing with official operations."""
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import torch
import torch.nn.functional as F

spec = importlib.util.spec_from_file_location('camera', Path(__file__).parents[1] / 'uav_diffphys_planner/camera.py')
camera = importlib.util.module_from_spec(spec)
spec.loader.exec_module(camera)

def main():
    rng = np.random.default_rng(71)
    raw = rng.uniform(.05, 40., (480,640)).astype('float32')
    raw[::17, ::13] = 0
    expected = torch.from_numpy(raw.copy())
    expected[expected == 0] = 24
    expected = 3 / expected.clamp(.3,24) - .6
    expected = F.max_pool2d(F.interpolate(expected[43:-43,58:-58][None,None], (36,48), mode='nearest'),3)
    selected = None
    for endian in ('<','>'):
        padded = np.zeros((480,644), dtype=endian+'f4')
        padded[:,:640] = raw
        header = SimpleNamespace(frame_id='optical', stamp=123)
        image = SimpleNamespace(height=480,width=640,step=644*4,encoding='32FC1',
                                is_bigendian=endian=='>',data=padded.tobytes(),header=header)
        sampled = camera.sample_high_speed(image)
        assert sampled.header == header and image.width == 640
        selected = np.frombuffer(sampled.data,dtype='<f4').reshape(36,48).copy()
        actual = F.max_pool2d(3 / torch.from_numpy(selected)[None,None].clamp(.3,24) - .6,3)
        torch.testing.assert_close(actual, expected, rtol=0, atol=0)
    for invalid in (float('nan'), float('inf'), -1.):
        broken = raw.copy()
        broken[0,0] = invalid
        image.data = broken.astype('<f4').tobytes()
        image.is_bigendian = False
        image.step = 640 * 4
        try: camera.sample_high_speed(image)
        except ValueError: pass
        else: raise AssertionError('nonfinite or negative source accepted')
    os.environ['DIFFPHYS_PROFILE'] = 'high_speed'
    from adapter import Predictor
    predictor = Predictor()
    reference_h = None
    hidden = None
    for i in range(30):
        state = torch.tensor([.2*i,-.3,.1,7.,.2,0.,.1,-.2,.97,.2],device=predictor.device)[None]
        with torch.inference_mode():
            reference, reference_h = predictor.model(expected.to(predictor.device),state,reference_h)
        result = predictor.infer(dict(request_id=str(i),depth=selected.tolist(),state=state[0].tolist(),hidden_state=hidden))
        torch.testing.assert_close(torch.tensor(result['action']),reference[0].cpu(),rtol=0,atol=0)
        torch.testing.assert_close(torch.tensor(result['hidden_state']),reference_h[0].cpu(),rtol=0,atol=0)
        hidden = result['hidden_state']
    print('PASS: exact official preprocessing, endian/stride/header, 30 recurrent frames on',predictor.device)

if __name__ == '__main__': main()
