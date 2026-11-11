"""Shared YOPO request pipeline for Torch and HMM engines."""
import time
import numpy as np
from processing import PREPROCESSING_VERSION, preprocess_depth


class YopoPredictor:
    def __init__(self, backend, processing, metadata_version=None):
        self.backend = backend
        self.processing = processing
        for direction in ("inputs", "outputs"):
            expected = getattr(processing, direction)
            actual = getattr(backend, direction)
            if set(actual) != set(expected) or any(
                list(actual[name]["shape"]) != expected[name]["shape"] for name in expected
            ):
                backend.close()
                raise ValueError("YOPO {} tensor contract mismatch".format(direction))
        version = "{};{};velocity={}".format(
            backend.metadata["model_version"], PREPROCESSING_VERSION, processing.vel_max)
        if metadata_version is not None:
            version += ";metadata=" + metadata_version
        self.metadata = dict(backend.metadata, model_version=version,
                             preprocessing_version=PREPROCESSING_VERSION,
                             velocity_m_s=processing.vel_max,
                             source_revision="bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132")
        try:
            self.backend.warmup({name: np.zeros(spec["shape"], dtype=np.float32)
                                 for name, spec in processing.inputs.items()})
        except Exception:
            backend.close()
            raise

    def close(self):
        self.backend.close()

    def infer(self, request):
        started = time.perf_counter()
        state = np.asarray(request["state"], dtype=np.float32)
        if state.shape != (9,):
            raise ValueError("state must contain 9 finite values")
        depth = preprocess_depth(
            request["depth"], self.processing.image_width, self.processing.image_height
        )
        obs = self.processing.prepare_observation(state)
        prepared = time.perf_counter()
        outputs, backend_timings = self.backend.infer(
            {"depth": depth, "obs": obs},
            output_dtypes={"predictions": np.float32, "scores": np.float32},
        )
        forwarded = time.perf_counter()
        endstates, scores, best = self.processing.decode(
            outputs["predictions"], outputs["scores"]
        )
        finished = time.perf_counter()
        return dict(
            request_id=request.get("request_id", ""),
            model_version=self.metadata["model_version"],
            device=self.metadata["device"],
            best_index=best,
            best_endstate=endstates[best].tolist(),
            endstates=endstates.tolist(),
            scores=scores.tolist(),
            duration_s=self.processing.segment_time,
            timings_ms=dict(
                preprocess=(prepared - started) * 1000,
                **backend_timings,
                forward_and_transfer=sum(backend_timings.values()),
                decode=(finished - forwarded) * 1000,
                total=(finished - started) * 1000,
            ),
        )
