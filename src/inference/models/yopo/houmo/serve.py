"""Board-side YOPO HMM worker using the shared FastAPI inference runtime."""

import importlib.util
import sys
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parent.parent
if str(MODEL_DIR) not in sys.path:
    sys.path.insert(0, str(MODEL_DIR))

from aerohub_inference.app import create_app
# Load the sibling explicitly: the parent also contains a CUDA adapter.py.
_spec = importlib.util.spec_from_file_location(
    "yopo_houmo_adapter", Path(__file__).resolve().with_name("adapter.py")
)
_adapter = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_adapter)
Predictor = _adapter.Predictor
from schemas import InferRequest, InferResponse

app = create_app(Predictor, InferRequest, InferResponse)
