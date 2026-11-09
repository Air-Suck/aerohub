"""Board-side checked replacement of persistent overlays; one transient rollback."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import tarfile

def install(root, bundle, as2_root=None):
    as2_root = Path(as2_root) if as2_root is not None else root/"as2"
    if not as2_root.is_relative_to(root) or ".." in as2_root.parts:
        raise ValueError("AS2 root must be below deployment root")
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
    if as2_root != root/'as2' and any(p.name != 'as2' for p in stage.iterdir()):
        raise ValueError('Candidate deployment accepts only AS2')
    for component in stage.iterdir():
        manifest = json.loads((component/'deployment.json').read_text())
        for name, digest in manifest['files'].items():
            assert hashlib.sha256((component/'install'/name).read_bytes()).hexdigest() == digest, name
    switched=[]
    try:
        for name in ('as2','catora','swarm-formation'):
            incoming = stage/name
            if not incoming.exists(): continue
            target = as2_root if name == 'as2' else root/('lib/'+name)
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
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--as2-root', type=Path, default=Path('/data/aerohub/as2'))
    args = parser.parse_args()
    root = Path('/data/aerohub')
    install(root, root/'.deploy/overlay.tar.gz', args.as2_root)
