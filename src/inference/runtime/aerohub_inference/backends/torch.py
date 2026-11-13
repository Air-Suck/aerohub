"""Torch tensor execution; model construction and image processing belong to callers."""
import hashlib
from pathlib import Path
import threading
import time

import numpy as np
import torch


class TorchBackend:
    """One model, fixed named tensors, synchronous CPU/CUDA execution."""

    def __init__(self, model_factory, weights_path, model_id, inputs, outputs, device='cuda'):
        self.device = torch.device(device)
        if self.device.type not in ('cuda', 'cpu'):
            raise ValueError('device must be CPU or CUDA')
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA is required; no CPU fallback')
        if not inputs or not outputs:
            raise ValueError('input and output tensor contracts are required')
        self.inputs = {name: dict(spec, dtype=np.dtype(spec['dtype']).name)
                       for name, spec in inputs.items()}
        self.outputs = {name: dict(spec, dtype=np.dtype(spec['dtype']).name)
                        for name, spec in outputs.items()}
        weights = Path(weights_path)
        self._lock = threading.Lock()
        self._closed = False
        self._model = model_factory().to(self.device).eval()
        self._model.load_state_dict(torch.load(weights, map_location=self.device, weights_only=True))
        self.metadata = dict(model_id=model_id,
                             model_version='sha256:' + hashlib.sha256(weights.read_bytes()).hexdigest(),
                             device=str(self.device), inputs=self.inputs, outputs=self.outputs)

    def _synchronize(self):
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)

    def infer(self, inputs, output_dtypes=None):
        if not isinstance(inputs, dict) or set(inputs) != set(self.inputs):
            raise ValueError('input names must match {}'.format(sorted(self.inputs)))
        output_dtypes = output_dtypes or {}
        if set(output_dtypes) - set(self.outputs):
            raise ValueError('unknown output dtype override')
        prepared = {}
        for name, spec in self.inputs.items():
            value = np.asarray(inputs[name])
            if value.shape != tuple(spec['shape']):
                raise ValueError('{} shape {} does not match {}'.format(name, value.shape, spec['shape']))
            if not np.issubdtype(value.dtype, np.number):
                raise ValueError('{} must contain numeric values'.format(name))
            prepared[name] = np.ascontiguousarray(value, dtype=spec['dtype'])
        with self._lock, torch.inference_mode():
            if self._closed:
                raise RuntimeError('backend is closed')
            started = time.perf_counter()
            tensors = {name: torch.from_numpy(value).to(self.device) for name, value in prepared.items()}
            self._synchronize()
            transferred = time.perf_counter()
            values = self._model(**tensors)
            self._synchronize()
            forwarded = time.perf_counter()
            if isinstance(values, torch.Tensor):
                values = (values,)
            if isinstance(values, dict):
                if set(values) != set(self.outputs):
                    raise RuntimeError('model output names do not match contract')
                named = values
            else:
                if len(values) != len(self.outputs):
                    raise RuntimeError('model output count does not match contract')
                named = dict(zip(self.outputs, values))
            result = {}
            for name, spec in self.outputs.items():
                value = named[name]
                if list(value.shape) != list(spec['shape']):
                    raise RuntimeError('{} output shape does not match contract'.format(name))
                result[name] = np.array(value.cpu().numpy(),
                                        dtype=output_dtypes.get(name, spec['dtype']), copy=True)
            finished = time.perf_counter()
        return result, dict(input_transfer=(transferred-started)*1000,
                            forward_and_sync=(forwarded-transferred)*1000,
                            output_transfer=(finished-forwarded)*1000)

    def warmup(self, inputs):
        return self.infer(inputs)

    def close(self):
        with self._lock:
            if not self._closed:
                self._synchronize()
                self._model = None
                self._closed = True
