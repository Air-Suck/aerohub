#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: build_workspace.sh check | build [package ...] | shell

check: validate mounted SDK, ROS target environment and host generator.
build: compile the mounted workspace; optional packages select --packages-up-to.
shell: open a build shell; it does not start a ROS application.

Mount an isolated prepared tree at /work (root/ and ws/) and target libraries
read-only at /target-out. SDK and target dependencies are not included in the image.
EOF
}

check() {
  local path
  for path in "$WS_ROOT/ros_ros2_base" "$OHOS_WORKSPACE/src" \
    "$WS_ROOT/ohos-robot-toolchain/18/native/llvm/bin/clang" \
    "$WS_ROOT/install/setup.bash" "$OHOS_SYSDEPS/include" \
    "$OHOS_SYSDEPS/lib/libssl.so" "$OHOS_SYSDEPS/lib/libcrypto.so" \
    "$OHOS_HOST_PYTHON"; do
    if [[ ! -e "$path" ]]; then
      echo "Missing build input: $path" >&2
      return 1
    fi
  done
  if [[ ! -w "$WS_ROOT/ohos-robot-toolchain/18/native" || ! -w "$OHOS_WORKSPACE" ]]; then
    echo "Mount the isolated SDK copy and workspace writable" >&2
    return 1
  fi
  printf 'Target: %s / %s\nPrefix: %s\n' "$OHOS_CPU" "$OHOS_PYTHON_SOABI" "$OHOS_INSTALL_PREFIX"
  "$OHOS_HOST_PYTHON" --version
  "$WS_ROOT/ohos-robot-toolchain/18/native/llvm/bin/clang" --version | head -1
  echo "Build inputs ready"
}

case "${1:---help}" in
  -h|--help|help) usage ;;
  check) check ;;
  shell) check; exec bash -l ;;
  build)
    shift
    check
    packages=()
    if (( $# )); then
      for package in "$@"; do
        [[ "$package" =~ ^[a-z][a-z0-9_]*$ ]] || { echo "Invalid ROS package: $package" >&2; exit 2; }
      done
      packages=(--packages-up-to "$@")
    fi
    cmake_args=(-DBUILD_TESTING=OFF "-DCMAKE_PREFIX_PATH=$OHOS_SYSDEPS"
      -DCMAKE_MODULE_PATH=/opt/aerohub/ohos/cmake "-DOPENSSL_ROOT_DIR=$OHOS_SYSDEPS"
      "-DOPENSSL_INCLUDE_DIR=$OHOS_SYSDEPS/include"
      "-DOPENSSL_CRYPTO_LIBRARY=$OHOS_SYSDEPS/lib/libcrypto.so"
      "-DOPENSSL_SSL_LIBRARY=$OHOS_SYSDEPS/lib/libssl.so"
      "-DEIGEN3_INCLUDE_DIR=$OHOS_SYSDEPS/include/eigen3"
      "-DPYTHON_EXECUTABLE=$OHOS_HOST_PYTHON" "-DPYTHON_SOABI=$OHOS_PYTHON_SOABI")
    pybind="$WS_ROOT/install/share/moveit_task_constructor_core/cmake"
    if [[ -f "$pybind/pybind11Config.cmake" ]]; then
      cmake_args+=("-Dpybind11_DIR=$pybind")
    fi
    exec bash -lc 'exec build-ros-humble "$@"' build_workspace \
      --custom --wd "$OHOS_WORKSPACE" --custom-prefix "$OHOS_INSTALL_PREFIX" \
      --colcon-args "${packages[@]}" --cmk-args "${cmake_args[@]}"
    ;;
  *) usage >&2; exit 2 ;;
esac
