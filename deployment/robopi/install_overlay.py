"""Board-side checked replacement of persistent overlays; one transient rollback."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import tarfile

def install(root, bundle):
    stage = root/'.deploy/stage'
    if stage.exists():
        raise RuntimeError('Previous staging exists; inspect it before retrying')
    stage.mkdir()
    # Python tarfile handles GNU long names; only these deployment components are accepted.
    with tarfile.open(bundle) as archive:
        for member in archive.getmembers():
            path = Path(member.name)
            if path.is_absolute() or '..' in path.parts or not path.parts or path.parts[0] not in ('as2', 'catora', 'swarm-formation') or member.issym() or member.islnk():
                raise RuntimeError('Unexpected archive member: '+member.name)
        archive.extractall(stage, filter='data')
    for component in stage.iterdir():
        manifest = json.loads((component/'deployment.json').read_text())
        for name, digest in manifest['files'].items():
            assert hashlib.sha256((component/'install'/name).read_bytes()).hexdigest() == digest, name
    switched=[]
    try:
        for name in ('as2','catora','swarm-formation'):
            incoming = stage/name
            if not incoming.exists(): continue
            target = root/('as2' if name == 'as2' else 'lib/'+name)
            target.mkdir(parents=True, exist_ok=True)
            previous = stage/(name+'.old')
            if (target/'install').exists(): os.rename(target/'install', previous)
            try:
                os.rename(incoming/'install', target/'install')
            except BaseException:
                if previous.exists(): os.rename(previous, target/'install')
                raise
            switched.append((target, previous))
            for file in incoming.iterdir(): os.replace(file, target/file.name)
            (target/'runtime').mkdir(exist_ok=True)
    except BaseException:
        for target, previous in reversed(switched):
            shutil.rmtree(target/'install')
            if previous.exists(): os.rename(previous, target/'install')
        raise
    shutil.rmtree(stage)
    bundle.unlink()
    print('Verified and replaced persistent installation; transient old installs removed')


if __name__ == '__main__':
    root = Path('/data/aerohub')
    install(root, root/'.deploy/overlay.tar.gz')
