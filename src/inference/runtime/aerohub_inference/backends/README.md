# Tensor backend contract

A model Predictor owns preprocessing, decoding and response semantics. An execution
backend owns the loaded model, target device, tensor transfer and synchronous execution.
Backends are optional imports: importing the FastAPI runtime does not import Torch or TCIM.
Model environments provide their own NumPy/framework dependencies.

Both `TorchBackend` and `aerohub_houmo.HmmBackend` expose:

- `inputs` / `outputs`: mappings from tensor name to `shape` and NumPy `dtype`;
  hardware-specific fields may be included.
- `metadata`: at least `model_id`, `model_version` (artifact SHA256), `device`,
  `inputs`, `outputs`; models may extend identity with processing parameters.
- `infer(inputs, output_dtypes=None)`: exact named NumPy tensors →
  `(named_outputs, timings_ms)`. Shapes are fixed by the loaded model contract.
  The backend casts inputs to its declared dtype and returns owned CPU arrays.
  `output_dtypes` optionally selects CPU output dtypes by output name.
- `warmup(inputs)`: execute supplied model-specific tensors.
- `close()`: release the model after outstanding execution, safe to repeat;
  inference after close raises `RuntimeError`.

Each backend serializes access to its model. Timings use milliseconds and the same
keys: `input_transfer`, `forward_and_sync`, `output_transfer`. Host tensor validation
occurs before these intervals. CUDA timings synchronize the selected device at phase
boundaries; they are wall-clock measurements, not isolated kernel profiling.

The HTTP runtime serializes complete requests, including CPU processing, and keeps
its busy slot until actual execution completes after caller cancellation. Backend
`close()` is invoked during normal worker shutdown. This contract does not define
cross-model scheduling or hot replacement.

`TorchBackend` receives a model factory, weights, device and named tensor specs.
It loads `state_dict` once and runs `eval()` / `inference_mode()`, with no CPU fallback
when CUDA is requested. Tuple outputs follow the declared output order; dictionaries
must have exactly the declared names. `HmmBackend` discovers specs from the compiled
HMM and uses the matching target TCIM runtime. Image operations and model-specific
state transforms belong in neither engine.
