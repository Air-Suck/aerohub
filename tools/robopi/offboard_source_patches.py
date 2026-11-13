"""Apply reviewed source patches to an isolated git-archive tree, idempotently."""
import os
from pathlib import Path
import subprocess


def apply_source_patch(source: Path, patch: Path) -> str:
    source, patch = source.resolve(), patch.resolve()
    env = dict(os.environ)
    # git-archive trees have no .git: do not discover the enclosing AeroHub repo.
    env['GIT_CEILING_DIRECTORIES'] = str(source.parent)
    def check(reverse=False):
        return subprocess.run(['git', 'apply', '--check', *(['--reverse'] if reverse else []), str(patch)],
                              cwd=source, env=env, capture_output=True, text=True)
    forward = check()
    if forward.returncode == 0:
        subprocess.run(['git', 'apply', str(patch)], cwd=source, env=env, check=True)
        return 'applied'
    if check(reverse=True).returncode == 0:
        return 'already_in_locked_source'
    raise RuntimeError('Patch does not match locked source: '+patch.name+'\n'+forward.stderr)
