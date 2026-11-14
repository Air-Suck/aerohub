"""YOPO HMM construction; preprocessing and requests use the shared model pipeline."""
import hashlib
import os
from pathlib import Path

from aerohub_houmo import HmmBackend
from predictor import YopoPredictor
from processing import YopoProcessing


class Predictor(YopoPredictor):
    def __init__(self):
        here = Path(__file__).resolve().parent
        hmm_path = Path(os.environ.get("YOPO_HMM_PATH", str(here / "model.hmm")))
        metadata_path = Path(os.environ.get("YOPO_RUNTIME_METADATA", str(here / "yopo-runtime.npz")))
        processing = YopoProcessing.from_file(metadata_path, float(os.environ.get("YOPO_VELOCITY", "6.0")))
        backend = HmmBackend(hmm_path, "yopo", device_id=int(os.environ.get("YOPO_DEVICE_ID", "0")))
        super().__init__(backend, processing, hashlib.sha256(metadata_path.read_bytes()).hexdigest())
