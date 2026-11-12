"""Model-agnostic TCIM HMM loading, tensor validation and synchronous inference."""

import hashlib
import threading
import time
from pathlib import Path

import numpy as np


def _default_module_factory(hmm_path, device_id):
    import tcim_lite as tcim

    option = tcim.runtime.Option(device_id)
    return tcim.runtime.Module.load(str(hmm_path), option=option)


def _tensor_spec(info):
    return {
        "shape": [int(value) for value in info.shape],
        "dtype": np.dtype(info.dtype).name,
        "format": str(info.format),
    }


class HmmBackend:
    """Own one HMM module and expose named NumPy inputs and outputs."""

    def __init__(self, hmm_path, model_id, device_id=0, module_factory=None):
        self.hmm_path = Path(hmm_path).resolve()
        if not self.hmm_path.is_file():
            raise ValueError("HMM file does not exist: {}".format(self.hmm_path))
        if not isinstance(model_id, str) or not model_id:
            raise ValueError("model_id must be a nonempty string")
        if type(device_id) is not int or device_id < 0:
            raise ValueError("device_id must be a nonnegative integer")

        factory = module_factory or _default_module_factory
        self._module = factory(self.hmm_path, device_id)
        self._lock = threading.Lock()
        self._closed = False
        self.inputs = {
            self._module.get_input_name(index): _tensor_spec(
                self._module.get_input_info(self._module.get_input_name(index))
            )
            for index in range(self._module.get_num_inputs())
        }
        self.outputs = {
            self._module.get_output_name(index): _tensor_spec(
                self._module.get_output_info(self._module.get_output_name(index))
            )
            for index in range(self._module.get_num_outputs())
        }
        if not self.inputs or not self.outputs:
            raise RuntimeError("HMM must expose at least one input and one output")

        digest = hashlib.sha256(self.hmm_path.read_bytes()).hexdigest()
        try:
            tcim_version = self._module.get_model_version()
        except AttributeError:
            tcim_version = "unknown"
        self.metadata = {
            "model_id": model_id,
            "model_version": "sha256:" + digest,
            "device": "houmo:xh2:{}".format(device_id),
            "tcim_model_version": str(tcim_version),
            "inputs": self.inputs,
            "outputs": self.outputs,
        }

    def infer(self, inputs, output_dtypes=None):
        """Run one inference with exact named inputs and return named outputs."""
        if not isinstance(inputs, dict):
            raise ValueError("inputs must be a mapping")
        missing = sorted(set(self.inputs) - set(inputs))
        extra = sorted(set(inputs) - set(self.inputs))
        if missing or extra:
            raise ValueError(
                "input names mismatch; missing={}, extra={}".format(missing, extra)
            )

        prepared = {}
        for name, spec in self.inputs.items():
            value = np.asarray(inputs[name])
            expected_shape = tuple(spec["shape"])
            if value.shape != expected_shape:
                raise ValueError(
                    "{} shape {} does not match {}".format(
                        name, value.shape, expected_shape
                    )
                )
            dtype = np.dtype(spec["dtype"])
            if not np.issubdtype(value.dtype, np.number):
                raise ValueError("{} must contain numeric values".format(name))
            prepared[name] = np.ascontiguousarray(value, dtype=dtype)

        output_dtypes = output_dtypes or {}
        unknown_outputs = sorted(set(output_dtypes) - set(self.outputs))
        if unknown_outputs:
            raise ValueError(
                "output dtype override names are unknown: {}".format(unknown_outputs)
            )

        with self._lock:
            if self._closed:
                raise RuntimeError("backend is closed")
            started = time.perf_counter()
            for name, value in prepared.items():
                self._module.set_input(name, value)
            prepared_at = time.perf_counter()
            self._module.run()
            self._module.sync()
            forwarded_at = time.perf_counter()

            result = {}
            for name in self.outputs:
                tensor = self._module.get_output(name)
                dtype = output_dtypes.get(name)
                if dtype is None:
                    result[name] = np.array(tensor.numpy(), copy=True)
                else:
                    result[name] = np.array(
                        tensor.astype(np.dtype(dtype).type).numpy(), copy=True
                    )
        finished = time.perf_counter()
        timings_ms = {
            "input_transfer": (prepared_at - started) * 1000,
            "forward_and_sync": (forwarded_at - prepared_at) * 1000,
            "output_transfer": (finished - forwarded_at) * 1000,
        }
        return result, timings_ms

    def warmup(self, inputs):
        return self.infer(inputs)

    def close(self):
        with self._lock:
            if not self._closed:
                self._module.sync()
                self._module = None
                self._closed = True
