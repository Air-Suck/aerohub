"""Relocate metadata exported by this board's ROS build into the builder mounts."""
from pathlib import Path
import sys

work = Path(sys.argv[1]).resolve()
install = work/'root/install'
if not (work/'ws/src').is_dir() or install.is_symlink() or not install.resolve().is_relative_to(work):
    raise SystemExit('Expected a prepared isolated OHOS work root, not the original dependency export')
if not (install / "share/rosidl_generator_py/package.xml").is_file():
    raise SystemExit(f"Missing ROS install in {install}")
prefixes = {
    b"/mnt/ohos/ohos-robot-toolchain/18/native": b"/work/root/ohos-robot-toolchain/18/native",
    b"/mnt/ohos/install": b"/work/root/install",
    b"/mnt/ohos/out": b"/target-out",
    b"/sys_prod/robot/install": b"/work/root/install",
    b"/sys_prod/robot/out": b"/target-out",
}
for path in sorted(install.rglob("*")):
    if path.is_symlink() or not path.is_file() or path.suffix not in {".cmake", ".pc", ".sh", ".bash", ".dsv"}:
        continue
    original = updated = path.read_bytes()
    for old, new in prefixes.items():
        updated = updated.replace(old, new)
    if original != updated:
        path.write_bytes(updated)
        print(path.relative_to(install))