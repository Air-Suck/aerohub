"""Keep Noble PX4/Agent libraries private when running in the Humble image."""
import os
from pathlib import Path
import shutil
import subprocess

roots = [Path('/opt/px4-gazebo'), Path('/opt/xrce')]
def is_elf(path):
    if not path.is_file():
        return False
    with path.open('rb') as stream:
        return stream.read(4) == b'\x7fELF'


binaries = sorted({p.resolve() for root in roots for p in root.rglob('*') if is_elf(p)})
private = Path('/opt/px4-runtime/lib')
private.mkdir(parents=True, exist_ok=True)
env = dict(os.environ, LD_LIBRARY_PATH='/opt/xrce/lib:/opt/xrce/lib64')
for binary in binaries:
    result = subprocess.run(['ldd', str(binary)], env=env, text=True, capture_output=True)
    if 'not found' in result.stdout:
        raise RuntimeError(result.stdout)
    for line in result.stdout.splitlines():
        fields = line.split()
        path = fields[2] if '=>' in fields else fields[0] if fields else ''
        if path.startswith('/') and not path.startswith('/opt/'):
            shutil.copy2(path, private / Path(path).name)
loader = private / 'ld-linux-x86-64.so.2'
if not loader.exists():
    raise RuntimeError('Expected amd64 dynamic loader is missing')
for binary in binaries:
    interpreter = subprocess.run(['patchelf', '--print-interpreter', str(binary)],
                                 capture_output=True)
    if interpreter.returncode == 0:
        subprocess.run(['patchelf', '--set-interpreter', str(loader), str(binary)], check=True)
    subprocess.run(['patchelf', '--force-rpath', '--set-rpath',
                    '/opt/px4-runtime/lib:/opt/xrce/lib:/opt/px4-gazebo/lib', str(binary)], check=True)
