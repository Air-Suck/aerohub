"""Extract only verified deployment inputs; never execute the official evaluator."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import zipfile


MODEL_DIR = Path(__file__).resolve().parent


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def prepare(archive):
    manifest = json.loads((MODEL_DIR / "artifacts.json").read_text())
    repo = MODEL_DIR.parents[4]
    source = repo / "src/tool/navigation/planner/diffphys/upstream/model.py"
    if not source.is_file() or digest(source) != manifest["upstream_model_sha256"]:
        raise ValueError("Initialize the supported diffphys upstream revision; existing source is preserved")
    revision = subprocess.check_output(["git", "-C", str(source.parent), "rev-parse", "HEAD"], text=True).strip()
    if revision != manifest["upstream_revision"]:
        raise ValueError("Unsupported upstream revision; existing checkout is preserved")
    destination = MODEL_DIR / "artifacts"
    destination.mkdir(exist_ok=True)
    groups = {
        manifest["inner_archive"]: {
            name: (destination / name, checksum)
            for name, checksum in manifest["files"].items()
        },
        manifest["high_speed_inner_archive"]: {
            name: (destination / record["target"], record["sha256"])
            for name, record in manifest["high_speed_files"].items()
        },
    }
    needed = {}
    for inner_archive, records in groups.items():
        pending = {}
        for name, (target, checksum) in records.items():
            if target.exists():
                if not target.is_file() or digest(target) != checksum:
                    raise ValueError("Existing artifact differs; preserving " + str(target))
            else:
                pending[name] = (target, checksum)
        if pending:
            needed[inner_archive] = pending
    if not needed:
        print("Verified existing DiffPhys swarm and high-speed artifacts")
        return
    if not archive.is_file() or digest(archive) != manifest["archive_sha256"]:
        raise ValueError("Supply the complete official validation_code.zip with --archive; expected SHA256 " + manifest["archive_sha256"])
    with zipfile.ZipFile(archive) as outer:
        for inner_archive, pending in needed.items():
            with outer.open(inner_archive) as stream:
                with tarfile.open(fileobj=stream, mode="r|gz") as inner:
                    for member in inner:
                        if member.name not in pending:
                            continue
                        if not member.isfile() or member.size > 16 * 1024 * 1024:
                            raise ValueError("Unexpected artifact type/size: " + member.name)
                        target, checksum = pending[member.name]
                        data = inner.extractfile(member).read()
                        if hashlib.sha256(data).hexdigest() != checksum:
                            raise ValueError("Artifact checksum mismatch: " + member.name)
                        target.parent.mkdir(parents=True, exist_ok=True)
                        with tempfile.NamedTemporaryFile(dir=str(target.parent), delete=False) as output:
                            temporary = Path(output.name)
                            output.write(data)
                        try:
                            # An existing target is never silently overwritten.
                            os.link(str(temporary), str(target))
                        finally:
                            temporary.unlink()
                        print("Prepared", member.name, "->", target.relative_to(destination))
                        del pending[member.name]
                        if not pending:
                            break
            if pending:
                raise ValueError("Missing official artifacts from " + inner_archive + ": " + ", ".join(pending))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True)
    prepare(parser.parse_args().archive.resolve())
