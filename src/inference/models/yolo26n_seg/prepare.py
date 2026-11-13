# SPDX-License-Identifier: GPL-3.0-or-later
"""Prepare the established state_dict tensor contract from a verified checkpoint."""
import hashlib
import json
from pathlib import Path

import torch
from ultralytics import YOLO

from prepare_artifacts import prepare


def main():
    prepare()
    directory = Path(__file__).resolve().parent / 'artifacts'
    source = directory / 'yolo26n-seg.pt'
    target = source.with_suffix('.state.pth')
    model = YOLO(str(source)).model.float()
    if 'yolo26n-seg' not in str(model.yaml.get('yaml_file', '')) or len(model.names) != 80:
        raise ValueError('Checkpoint family/class table mismatch')
    state = {'network.' + name: value for name, value in model.state_dict().items()}
    if target.exists():
        previous = torch.load(target, map_location='cpu', weights_only=True)
        if set(previous) != set(state) or any(not torch.equal(previous[k], state[k]) for k in state):
            raise ValueError(f'Existing converted state differs: {target}')
    else:
        torch.save(state, target)
    record = dict(model='yolo26n-seg', source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  state_sha256=hashlib.sha256(target.read_bytes()).hexdigest(), names=model.names,
                  conversion='official model.float().state_dict(); network prefix; no ONNX export')
    (directory / 'state-manifest.json').write_text(json.dumps([record], indent=2) + '\n')
    print(json.dumps(record))


if __name__ == '__main__':
    main()
