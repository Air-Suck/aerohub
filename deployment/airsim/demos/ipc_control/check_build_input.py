"""Keep local build edits intact: only accept the recorded baseline or patched source."""
import hashlib
import json
from pathlib import Path
import sys

source, manifest, patch = map(Path, sys.argv[1:])
record = json.loads(manifest.read_text())
digest = hashlib.sha256(source.read_bytes()).hexdigest()
if digest not in (record['baseline_sha256'], record['patched_sha256'], *record.get('compatible_patched_sha256', [])):
    raise SystemExit('Build source differs from the recorded versions; preserving it without overwrite')
if hashlib.sha256(patch.read_bytes()).hexdigest() != record['patch_sha256']:
    raise SystemExit('Patch changed; review and update its provenance first')
print(json.dumps({'source_sha256': digest, 'patch_sha256':record['patch_sha256']}))
