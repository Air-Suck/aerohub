"""Export locked YOPO forward and compare every output on real CUDA execution."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch

HERE = Path(__file__).resolve().parent
MODEL = HERE.parent
ROOT = HERE.parents[5]
SOURCE = ROOT / 'src/tool/navigation/planner/yopo/upstream/YOPO'
sys.path.insert(0, str(SOURCE))
from config.config import cfg
from policy.yopo_network import YopoNetwork


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', choices=['cuda', 'cpu'], default='cuda')
    args = parser.parse_args()
    if args.device == 'cuda' and not torch.cuda.is_available():
        raise RuntimeError('Real CUDA reference execution required')
    device = torch.device(args.device)
    cfg['train'] = False
    weights = SOURCE / 'saved/YOPO_1/epoch50.pth'
    expected = (MODEL / 'weights.sha256').read_text().split()[0]
    assert hashlib.sha256(weights.read_bytes()).hexdigest() == expected
    torch.manual_seed(7)
    model = YopoNetwork().to(device).eval()
    model.load_state_dict(torch.load(weights, map_location=device, weights_only=True))
    out = HERE / 'artifacts'
    out.mkdir(exist_ok=True)
    depth = torch.rand(1, 1, cfg['image_height'], cfg['image_width'], device=device)
    state = np.array([[1., 0., 0., 0., 0., 0., 10., 0., 0.]], dtype=np.float32)
    transform = model.state_transform
    obs = torch.from_numpy(transform.prepare_input_cpu(transform.normalize_obs_cpu(state))).to(device)
    path = out / 'yopo.onnx'
    with torch.inference_mode():
        torch.onnx.export(model, (depth, obs), str(path), input_names=['depth', 'obs'],
                          output_names=['predictions', 'scores'], opset_version=17, dynamo=False)
    onnx.checker.check_model(onnx.load(path))
    session = ort.InferenceSession(str(path), providers=['CPUExecutionProvider'])
    lattice = transform.lattice_primitive
    np.savez(
        out / 'yopo-runtime.npz',
        lattice_yaw=transform.lattice_yaw_np,
        lattice_pitch=transform.lattice_pitch_np,
        lattice_Rbp=transform.lattice_Rbp_np,
        goal_length=float(transform.goal_length),
        vel_max=float(lattice.vel_max),
        acc_max=float(lattice.acc_max),
        yaw_diff=float(lattice.yaw_diff),
        pitch_diff=float(lattice.pitch_diff),
        radio_range=float(lattice.radio_range),
        segment_time=float(lattice.segment_time),
        vertical_num=int(lattice.vertical_num),
        horizon_num=int(lattice.horizon_num),
        image_height=int(cfg['image_height']),
        image_width=int(cfg['image_width']),
    )
    # Fixed synthetic samples test export fidelity, not calibration representativeness.
    results = []
    (out / 'synthetic-inputs').mkdir(exist_ok=True)
    (out / 'references').mkdir(exist_ok=True)
    for i in range(8):
        depth = torch.rand_like(depth)
        state = np.array([[i * .5, .1, -.1, .2, 0., 0., 10., i - 4., .5]], dtype=np.float32)
        obs = torch.from_numpy(transform.prepare_input_cpu(transform.normalize_obs_cpu(state))).to(device)
        feed = {'depth': depth.cpu().numpy(), 'obs': obs.cpu().numpy()}
        with torch.inference_mode():
            reference = [x.cpu().numpy() for x in model(depth, obs)]
        actual = session.run(None, feed)
        errors = []
        for ref, got in zip(reference, actual):
            np.testing.assert_allclose(got, ref, rtol=1e-4, atol=1e-5)
            errors.append(float(np.max(np.abs(got-ref))))
        np.savez(out / 'synthetic-inputs' / f'synthetic-{i:03}.npz', **feed)
        np.savez(out / 'references' / f'reference-{i:03}.npz', predictions=reference[0], scores=reference[1])
        results.append(errors)
    report = {'weights_sha256': expected, 'onnx_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'reference_device': args.device, 'torch': torch.__version__, 'inputs': {x.name: x.shape for x in session.get_inputs()},
              'outputs': {x.name: x.shape for x in session.get_outputs()},
              'atol': 1e-5, 'rtol': 1e-4, 'max_abs_errors': results,
              'scope': 'synthetic export fidelity; not representative quantization calibration or flight acceptance'}
    (out / 'export-report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
