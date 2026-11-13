"""Fetch only the pinned YOLO26n-seg checkpoint; never overwrite bad artifacts."""
import hashlib
import json
import shutil
from pathlib import Path
from urllib.request import urlopen

MODEL = Path(__file__).resolve().parent


def prepare():
    record = json.loads((MODEL / 'artifacts.json').read_text())
    directory = MODEL / 'artifacts'
    directory.mkdir(exist_ok=True)
    target = directory / record['filename']
    if not target.exists():
        temporary = target.with_suffix('.download')
        if temporary.exists():
            raise ValueError(f'Incomplete download exists: {temporary}; inspect it before retrying')
        try:
            with urlopen(record['url'], timeout=30) as response, temporary.open('xb') as output:
                shutil.copyfileobj(response, output)
            if hashlib.sha256(temporary.read_bytes()).hexdigest() != record['sha256']:
                raise ValueError('Downloaded checkpoint checksum mismatch')
            temporary.replace(target)
        except Exception:
            # Keep incomplete evidence; never accept it as a model.
            raise
    if hashlib.sha256(target.read_bytes()).hexdigest() != record['sha256']:
        raise ValueError(f'Existing checkpoint checksum mismatch: {target}')
    manifest = [dict(model=record['model'], url=record['url'], sha256=record['sha256'],
                     license=record['license'])]
    (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Verified {target}')


if __name__ == '__main__':
    prepare()
