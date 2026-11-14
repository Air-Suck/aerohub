from aerohub_inference.app import create_app
from adapter import Predictor
from schemas import InferRequest, InferResponse

app = create_app(Predictor, InferRequest, InferResponse)
