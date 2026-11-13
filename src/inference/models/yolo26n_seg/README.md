# YOLO26n-seg Inference

This component migrates only YOLO26n-seg. It returns original-image pixel boxes,
categories, confidence and binary instance masks. It does not calculate depth,
track target identities, publish TF, or command a vehicle. The separate
[Object Detection Tool](../../../perception/object_detection/README.md) combines
these results with ROS depth, CameraInfo and capture-time TF to report positions.

## Environment and preparation

Use Linux x86_64, Python 3.10, uv and an NVIDIA driver compatible with the locked
CUDA wheels. Versions and wheel sources are fixed by pyproject.toml/uv.lock.
ROS 2 Humble runs in its own environment; do not install ROS in the model venv.
The public runtime, TorchBackend and ModelClient are reused without modification.

From the repository root (setup also works from another working directory):

```bash
python3 deployment/inference/setup_models.py --config config/inference.yolo26n-seg.example.yaml --dry-run
python3 deployment/inference/setup_models.py --config config/inference.yolo26n-seg.example.yaml --check-cuda
bash deployment/inference/start_yolo26n_seg.sh
```

The manifest ID is `yolo26n-seg`. Existing YOPO/DiffPhys entries are retained;
the example selects only this model. Setup prepares a verified checkpoint in this
model's ignored artifacts/, syncs its own .venv with `uv sync --locked`, converts
the state_dict, checks CPU loading, and checks real CUDA when --check-cuda is set.
It starts no services. Incorrect existing artifacts cause failure, not overwrite.

The worker defaults to port 8002 and CUDA device 0. INFERENCE_HOST/INFERENCE_PORT
override binding. Use an explicitly reachable endpoint between containers.
CUDA_VISIBLE_DEVICES selects visible GPUs. YOLO_DEVICE=cpu is an explicit CPU mode,
not an automatic fallback. YOLO_CONFIG optionally selects another worker.yaml;
relative weight paths resolve against that config's directory.

## ROS facade

In the ROS Humble environment:

```bash
bash deployment/inference/build_yolo26n_seg_ros.sh
source log/inference/yolo26n_seg_ros_ws/install/setup.bash
python3 -m uav_inference.yolo_node --ros-args -r __ns:=/drone0 \
  -p endpoint:=http://127.0.0.1:8002
```

Start the worker and wait for `/health/ready` before the facade. The service
`<namespace>/inference/yolo/infer` uses
`uav_inference_interfaces/srv/InferDetections`. It accepts an RGB8/BGR8 ROS Image,
request_id and generation, preserves the original acquisition header and geometry,
and reports worker/model identity. Masks are mono8 0/1 in original image geometry.
No target-age policy is applied here. One request can run at a time; BUSY and
communication timeout do not cancel an active GPU kernel or trigger auto-retry.
`inference/yolo/status` checks worker readiness and rejects changed worker identity;
restart the facade and invalidate caller sessions after replacing the worker.

## Sources and verification

Checkpoint source/hash and license are in artifacts.json and NOTICE.md. The
Ultralytics one-to-many head, Conv/BN fusion, NMS and process_mask_native processing
retain the existing stage2 numerical behavior. No ONNX export is required.

```bash
src/inference/models/yolo26n_seg/.venv/bin/python \
  src/inference/models/yolo26n_seg/test_baseline.py \
  --image /path/to/real-image.jpg --output /path/to/model-result.json
```

The comparison uses the previous stage2 criteria (mask IoU >=0.995 and box error
<=0.01 px). Optional --legacy-stage2 performs the local migration comparison;
normal operation never depends on that workspace. See validation.json for the
actual scope and gaps. Numerical/interface checks are not live RGB-D accuracy,
30-second tracking, physical hardware or flight acceptance.
