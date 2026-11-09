"""Create a new isolated workspace from the user's official OHOS environment."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
ROS_COMMIT = '8f588ad3c244781cc1cfc7472cfe1cbaff6702c3'
SOURCES = {
    'px4_msgs': ('https://github.com/PX4/px4_msgs.git',
                 '86d8239e962f6939e05c3737784f60c02fa884db'),
    'vision_msgs': ('https://github.com/ros-perception/vision_msgs.git',
                    '5f35e557cfb5e085ac5c6dc0210c5736c2c362f7'),
    'mocap4r2_msgs': ('https://github.com/MOCAP4ROS2-Project/mocap4r2_msgs.git',
                     '2bce86f38b7129d235d58ebd8977beadb0a83123'),
    'pid_controller': ('https://github.com/RPS98/pid_controller.git',
                       'bc4743307db3f18affa232c04f068896cc69b528'),
}


def run(*args):
    return subprocess.run(list(map(str, args)), check=True)


def copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    run('cp', '-a', '--reflink=auto', source, target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-native', required=True, type=Path)
    parser.add_argument('--ros-base', required=True, type=Path)
    parser.add_argument('--ros-install', required=True, type=Path)
    parser.add_argument('--sysdeps', type=Path, default=os.environ.get('OHOS_SYSDEPS'))
    parser.add_argument('--work-root', type=Path,
                        default=os.environ.get('OHOS_WORK_ROOT', 'log/airsim/ohos-as2'))
    parser.add_argument('--source-cache', type=Path,
                        help='Optional directory of Git repositories named as the locked dependencies')
    args = parser.parse_args()
    if args.sysdeps is None:
        parser.error('Set OHOS_SYSDEPS in .env or pass --sysdeps')
    for key in ('sdk_native', 'ros_base', 'ros_install', 'sysdeps', 'work_root'):
        value = getattr(args, key).expanduser()
        setattr(args, key, (ROOT / value).resolve() if not value.is_absolute() else value.resolve())
    checks = [args.sdk_native / 'llvm/bin/clang',
              args.ros_base / 'docker/ohos.toolchain.xhw.cmake',
              args.ros_base / 'src/ros2/rmw_fastrtps/rmw_fastrtps_shared_cpp/package.xml',
              args.ros_install / 'share/rclcpp/package.xml',
              args.ros_install / 'include/libyaml_vendor/yaml.h',
              args.ros_install / 'lib/libyaml.so',
              args.sysdeps / 'lib/cmake/GeographicLib/geographiclib-config.cmake',
              args.sysdeps / 'lib/libGeographicLib.so',
              args.sysdeps / 'include/eigen3/Eigen/Core',
              args.sysdeps / 'lib/libsframe.a', args.sysdeps / 'lib/libssl.so',
              args.sysdeps / 'lib/libcrypto.so', args.sysdeps / 'lib/libpython3.12.so.1.0',
              args.sysdeps / 'include/python3.12/Python.h',
              args.sysdeps / 'lib/python3.12/site-packages/numpy/_core/include/numpy/arrayobject.h',
              args.sysdeps / 'include/GeographicLib/LocalCartesian.hpp']
    missing = [str(p) for p in checks if not p.exists()]
    if missing:
        parser.error('Missing official environment inputs:\n' + '\n'.join(missing))
    commit = subprocess.check_output(['git', '-C', str(args.ros_base), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != ROS_COMMIT:
        parser.error(f'ros_ros2_base must be at {ROS_COMMIT}; got {commit}')
    if args.work_root.exists():
        parser.error('Work root already exists; choose a new --work-root. Existing work is never overwritten.')
    for source in (args.sdk_native, args.ros_base, args.ros_install, args.sysdeps):
        if args.work_root.is_relative_to(source) or source.is_relative_to(args.work_root):
            parser.error('Work root and input directories must not contain one another')
    run('bash', ROOT / 'tools/airsim/prepare_sources.sh')
    base = args.work_root / 'root'
    workspace = args.work_root / 'ws/src'
    workspace.mkdir(parents=True)
    copy(args.sdk_native, base / 'ohos-robot-toolchain/18/native')
    copy(args.ros_base, base / 'ros_ros2_base')
    copy(args.ros_install, base / 'install')
    # Relocate exported CMake/pkg-config/environment metadata in the copy only.
    replacements = {str(args.sysdeps): '/target-out', str(args.ros_install): '/work/root/install',
                    '/target/out': '/target-out', '/target/install': '/work/root/install',
                    '/home/ipads/tmp/uav-lesson/artifacts/robopi-stage3/target/deps/out': '/target-out'}
    for path in (base / 'install').rglob('*'):
        if path.is_symlink() or not path.is_file() or path.suffix not in ('.cmake', '.pc', '.sh', '.bash', '.dsv'):
            continue
        text = path.read_text()
        updated = text
        for old, new in sorted(replacements.items(), key=lambda pair: -len(pair[0])):
            updated = updated.replace(old, new)
        if updated != text:
            path.write_text(updated)
    setup = base / 'install/setup.bash'
    setup.unlink(missing_ok=True)
    setup.write_text(
        'export AMENT_PREFIX_PATH=/work/root/install\n'
        'export CMAKE_PREFIX_PATH=/work/root/install\n'
        'export PYTHONPATH=/work/root/install/lib/python3.12/site-packages\n')
    sysroot = base / 'ohos-robot-toolchain/18/native/sysroot/usr'
    shutil.copytree(args.sysdeps / 'include/python3.12', sysroot / 'include/python3.12', dirs_exist_ok=True)
    for name in ('libsframe.a', 'libpython3.12.so.1.0'):
        target = sysroot / 'lib' / name
        if target.is_symlink():
            target.unlink()
        shutil.copyfile(args.sysdeps / 'lib' / name, target)
    link = sysroot / 'lib/libpython3.12.so'
    link.unlink(missing_ok=True)
    link.symlink_to('libpython3.12.so.1.0')
    numpy = 'lib/python3.12/site-packages/numpy/_core/include'
    shutil.copytree(args.sysdeps / numpy, sysroot / numpy, dirs_exist_ok=True)
    toolchain = base / 'ros_ros2_base/docker/ohos.toolchain.xhw.cmake'
    text = toolchain.read_text()
    old = 'set(CMAKE_FIND_ROOT_PATH "${OHOS_SDK_NATIVE}/sysroot")'
    new = 'set(CMAKE_FIND_ROOT_PATH "${OHOS_SDK_NATIVE}/sysroot" /target-out)'
    if old not in text and new not in text:
        raise RuntimeError('Unrecognized toolchain: CMAKE_FIND_ROOT_PATH requires review')
    toolchain.write_text(text.replace(old, new))
    copy(base / 'ros_ros2_base/src/ros2/rmw_fastrtps/rmw_fastrtps_shared_cpp',
         workspace / 'rmw_fastrtps_shared_cpp')
    for name, (url, revision) in SOURCES.items():
        target = workspace / name
        origin = str(args.source_cache.expanduser().resolve() / name) if args.source_cache else url
        run('git', 'clone', '--no-checkout', origin, target)
        run('git', '-C', target, 'checkout', '--detach', revision)
    pid = workspace / 'pid_controller'
    pid_target = workspace / 'as2_motion_controller/thirdparties/pid_controller'
    pid_target.parent.mkdir(parents=True)
    pid.rename(pid_target)
    env = dict(os.environ, OHOS_WORK_ROOT=str(args.work_root))
    subprocess.run([sys.executable, str(ROOT / 'tools/airsim/prepare_ohos_sources.py')], env=env, check=True)
    (args.work_root / 'prepared-inputs.json').write_text(json.dumps({
        'sdk_native': str(args.sdk_native), 'ros_install': str(args.ros_install),
        'sysdeps': str(args.sysdeps), 'ros_base_commit': commit,
        'sources': {name: revision for name, (_, revision) in SOURCES.items()},
    }, indent=2) + '\n')
    print(f'Prepared {args.work_root}. Set OHOS_WORK_ROOT and OHOS_SYSDEPS in .env, then run run.sh check.')


if __name__ == '__main__':
    main()
