"""Offline content/provenance checks for the curated EGO delivery."""
import ast
import hashlib
import json
from pathlib import Path
import re
import sys

root=Path(sys.argv[1]).resolve();demo=root/'deployment/airsim/demos/ego_planner_v2'
component=root/'src/planner/ego_planner_v2'
errors=[];count=0
for folder in (demo,component):
    for p in folder.rglob('*.py'):
        tree=ast.parse(p.read_text(encoding='utf-8-sig'),filename=str(p));count+=1
        text=p.read_text(encoding='utf8')
        uses_env=any(isinstance(n,ast.Attribute) and isinstance(n.value,ast.Name) and n.value.id=='os' and n.attr=='environ' for n in ast.walk(tree))
        if uses_env and not re.search(r'^import os\b|^from os\b',text,re.M):errors.append(str(p)+' lacks os import')
for p in (root/'deployment/robopi/deploy_lib.py',root/'tools/robopi/relocate_ego_target_metadata.py'):
    ast.parse(p.read_text(encoding='utf8'),filename=str(p));count+=1
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
baseline=json.loads((component/'SOURCE_BASELINE.json').read_text())
for group,folder in [('native_sha256','native'),('adapter_sha256','bridge')]:
    for rel,digest in baseline[group].items():
        if sha(component/'ros2_adapter'/folder/rel)!=digest:errors.append('source hash '+rel)
evidence=json.loads((demo/'evidence/index.json').read_text())
for row in evidence['files']:
    if sha(demo/row['file'])!=row['sha256']:errors.append('evidence hash '+row['file'])
for p in [component/'README.md',component/'ros2_adapter/bridge/README.md',demo/'README.md',demo/'VALIDATION.md',demo/'robopi/README.md']:
    for link in re.findall(r'\]\(([^)]+)\)',p.read_text(encoding='utf8')):
        if '://' in link or link.startswith('#'):continue
        target=link.split('#')[0]
        if target and not (p.parent/target).exists():errors.append('missing link '+str(p.relative_to(root))+' -> '+target)
assert not errors,'\n'.join(errors)
print(json.dumps(dict(passed=True,python_files=count,evidence_files=len(evidence['files']),native_files=len(baseline['native_sha256']),adapter_files=len(baseline['adapter_sha256'])),indent=2))
