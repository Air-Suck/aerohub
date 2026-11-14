"""Compare complete recurrent sequences against code extracted from official eval.py.

Does not import/run the evaluator (which arms and moves SimpleFlight vehicles).
"""

import argparse
import ast
import importlib.util
import json
import os
from pathlib import Path

import torch
import torch.nn.functional as F

from adapter import Predictor, MODEL_DIR
from schemas import InferRequest, InferResponse


def official_block():
    tree = ast.parse((MODEL_DIR / "artifacts/swarm/eval.py").read_text())
    main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
    loop = next(node for node in main.body if isinstance(node, ast.While))
    nodes, active = [], False
    for node in loop.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "env_R" for t in node.targets):
            active = True
        if active:
            nodes.append(node)
            if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) and node.value.func.id == "model":
                break
    if not nodes or not isinstance(nodes[-1].value, ast.Call) or nodes[-1].value.func.id != "model":
        raise AssertionError("Official evaluation block changed")
    return compile(ast.Module(body=nodes, type_ignores=[]), "official-swarm-eval-block", "exec")


@torch.inference_mode()
def check(device):
    os.environ["DIFFPHYS_DEVICE"] = device
    predictor = Predictor()
    spec = importlib.util.spec_from_file_location("official_diffphys_model", MODEL_DIR / "artifacts/swarm/model.py")
    reference = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reference)
    # Reference stays on CPU so CUDA also checks cross-device numerical agreement.
    model = reference.Model(10, 6).eval()
    model.load_state_dict(torch.load(str(MODEL_DIR / "artifacts/swarm/swarm.pth"), map_location="cpu", weights_only=True), strict=True)
    block = official_block()
    h, reference_h = None, None
    max_action, max_hidden = 0.0, 0.0
    inputs = []
    for i in range(40):
        yaw, pitch = torch.tensor(0.13 * i + 0.4), torch.tensor(0.1 + i * 0.004)
        cy, sy, cp, sp = yaw.cos(), yaw.sin(), pitch.cos(), pitch.sin()
        rotation = torch.tensor([[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0., cp]])[None]
        depth = (torch.arange(36 * 48, dtype=torch.float32).reshape(36, 48) % (47 + i) + 1) / 4
        # Exercise both clamp boundaries without inventing invalid-pixel semantics.
        depth[0, :3] = torch.tensor([0.01, 24., 100.])
        context = dict(torch=torch, F=F, args=argparse.Namespace(sr=3, no_odom=False),
                       R=rotation, target_v=torch.tensor([[1.1, -0.7, 0.2]]),
                       v=torch.tensor([[0.4 + i * 0.01, -0.2, 0.15]]),
                       extra=torch.tensor([[0.15]]), depth=depth[None],
                       device=torch.device("cpu"), model=model, hidden_state=reference_h)
        exec(block, context)
        request = dict(request_id=str(i), depth=depth.tolist(), state=context["state"][0].tolist(), hidden_state=h)
        result = InferResponse.model_validate(predictor.infer(request))
        expected_action, reference_h = context["action"], context["hidden_state"]
        action = torch.tensor(result.action)[None]
        actual_h = torch.tensor(result.hidden_state)[None]
        torch.testing.assert_close(action, expected_action, atol=1e-5, rtol=1e-4)
        torch.testing.assert_close(actual_h, reference_h, atol=1e-5, rtol=1e-4)
        max_action = max(max_action, (action - expected_action).abs().max().item())
        max_hidden = max(max_hidden, (actual_h - reference_h).abs().max().item())
        h = result.hidden_state
        inputs.append(request)
    # Branch/retry independence: no implicit per-worker memory, including after warmup.
    first = dict(inputs[0], request_id="reset", hidden_state=None)
    reset_a = predictor.infer(first)
    predictor.infer(inputs[-1])
    reset_b = predictor.infer(first)
    assert reset_a["action"] == reset_b["action"]
    assert reset_a["hidden_state"] == reset_b["hidden_state"]
    rejected = 0
    for invalid in [dict(first, depth=[[1.0]]), dict(first, state=[0.] * 7),
                    dict(first, hidden_state=[0.] * 191), dict(first, state=[float("nan")] * 10),
                    dict(first, depth=[[float("inf")] * 48 for _ in range(36)]),
                    dict(first, depth=[[0.] * 48 for _ in range(36)])]:
        try:
            predictor.infer(invalid)
        except ValueError:
            rejected += 1
        else:
            raise AssertionError("Invalid input accepted")
    print(json.dumps(dict(metadata=predictor.metadata, sequence_steps=len(inputs),
                          reference_device="cpu", max_action_abs_error=max_action,
                          max_hidden_abs_error=max_hidden, invalid_inputs_rejected=rejected,
                          reset_and_branch_independence="passed", atol=1e-5, rtol=1e-4), indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    check(parser.parse_args().device)
