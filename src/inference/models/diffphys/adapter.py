"""DiffPhys network only; no simulator API, task state, or motion publication."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

import torch
import torch.nn.functional as F

from schemas import InferRequest


MODEL_DIR = Path(os.environ.get("DIFFPHYS_MODEL_DIR", str(Path(__file__).resolve().parent)))
PROFILES = {
    "swarm": {
        "metadata": "official-swarm-odom",
        "source": None,
        "weights": "artifacts/swarm/swarm.pth",
        "source_sha256_key": "upstream_model_sha256",
        "preprocessing": "swarm-v1-depth-planar-pool3-state10-explicit-h-v1",
    },
    "high_speed": {
        "metadata": "official-high-speed-odom",
        "source": "artifacts/high_speed/model.py",
        "weights": "artifacts/high_speed/base.pth",
        "source_sha256_key": "high_speed_flight/src/e2e_planner_v2/src/model.py",
        "preprocessing": "high-speed-hfov91-crop82-nearest36x48-pool3-state10-explicit-h-v2",
    },
}


class Predictor:
    def __init__(self):
        self.manifest = json.loads((MODEL_DIR / "artifacts.json").read_text())
        self.profile = os.environ.get("DIFFPHYS_PROFILE", "swarm")
        if self.profile not in PROFILES:
            raise ValueError("DIFFPHYS_PROFILE must be swarm or high_speed")
        profile = PROFILES[self.profile]
        if profile["source"] is None:
            source = MODEL_DIR.parents[3] / "src/planner/diffphys/upstream/model.py"
        else:
            source = MODEL_DIR / profile["source"]
        source_checksum = hashlib.sha256(source.read_bytes()).hexdigest()
        if profile["source_sha256_key"] == "upstream_model_sha256":
            expected_source_checksum = self.manifest["upstream_model_sha256"]
        else:
            expected_source_checksum = self.manifest["high_speed_files"][profile["source_sha256_key"]]["sha256"]
        if source_checksum != expected_source_checksum:
            raise RuntimeError("Unsupported DiffPhys model source; existing source preserved")
        weights = MODEL_DIR / profile["weights"]
        checksum = hashlib.sha256(weights.read_bytes()).hexdigest()
        expected_weights = (self.manifest["files"]["swarm/swarm.pth"] if self.profile == "swarm" else
                            self.manifest["high_speed_files"]["high_speed_flight/src/e2e_planner_v2/base.pth"]["sha256"])
        if checksum != expected_weights:
            raise RuntimeError("Checkpoint checksum mismatch")
        device = os.environ.get("DIFFPHYS_DEVICE", "cuda")
        if device not in ("cpu", "cuda"):
            raise ValueError("DIFFPHYS_DEVICE must be cpu or cuda")
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA required; use DIFFPHYS_DEVICE=cpu for explicit CPU checks")
        self.device = torch.device(device)
        torch.set_num_threads(int(os.environ.get("DIFFPHYS_THREADS", "4")))
        # Keep full FP32 precision for cross-device recurrent-state comparison.
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cuda.matmul.allow_tf32 = False
        spec = importlib.util.spec_from_file_location("diffphys_upstream_model", source)
        module = importlib.util.module_from_spec(spec)
        # Execute the verified source without writing __pycache__ into the submodule.
        exec(compile(source.read_bytes(), str(source), "exec"), module.__dict__)
        self.model = module.Model(10, 6).to(self.device).eval()
        self.model.load_state_dict(torch.load(str(weights), map_location=self.device, weights_only=True), strict=True)
        self.preprocessing_version = profile["preprocessing"]
        self.metadata = dict(model_id="diffphys", profile=profile["metadata"],
                             model_version="sha256:" + checksum + ";" + self.preprocessing_version,
                             device=device, source_revision=self.manifest["upstream_revision"],
                             source_sha256=source_checksum,
                             preprocessing_version=self.preprocessing_version,
                             hidden_state_ownership="caller", observation_dim=10, action_dim=6,
                             torch_version=torch.__version__)
        self.infer(dict(request_id="warmup", depth=[[24.0] * 48 for _ in range(36)],
                        state=[0.0] * 9 + [0.15], hidden_state=None))

    def preprocess_depth(self, depth):
        depth = torch.tensor(depth, dtype=torch.float32, device=self.device)[None, None]
        depth = 3 / depth.clamp(0.3, 24) - 0.6
        return F.max_pool2d(depth, (3, 3))

    @torch.inference_mode()
    def infer(self, request):
        started = time.perf_counter()
        request = InferRequest.model_validate(request)
        depth = self.preprocess_depth(request.depth)
        state = torch.tensor(request.state, dtype=torch.float32, device=self.device)[None]
        hidden = None if request.hidden_state is None else torch.tensor(
            request.hidden_state, dtype=torch.float32, device=self.device)[None]
        output = self.model(depth, state, hidden)
        action, next_hidden = output[0], output[-1]
        action = action[0].cpu()
        next_hidden = next_hidden[0].cpu()
        if not torch.isfinite(action).all() or not torch.isfinite(next_hidden).all():
            raise RuntimeError("Nonfinite model output")
        return dict(request_id=request.request_id, model_version=self.metadata["model_version"],
                    device=str(self.device), action=action.tolist(), hidden_state=next_hidden.tolist(),
                    timings_ms=dict(total=(time.perf_counter() - started) * 1000))
