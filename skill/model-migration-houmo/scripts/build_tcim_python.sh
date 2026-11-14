#!/usr/bin/env bash
# Run inside houmo-migrate-<model> with TCIM_RUNTIME_PATH and OHOS_SYSDEPS mounts.
set -euo pipefail
output=${1:?Usage: build_tcim_python.sh /workspace/path/to/output}
tcim=${TCIM_RUNTIME_PATH:-/opt/houmo/tcim}
sysdeps=${OHOS_SYSDEPS:-/target-out}
sdk=${OHOS_NATIVE_SDK:-/opt/ohos-sdk/native}
python=${PYTHON:-/opt/venv/dadao/bin/python}

[[ -f "$tcim/python/CMakeLists.txt" ]] || {
  echo "TCIM Python binding source is missing: $tcim/python" >&2; exit 1;
}
[[ -f "$sysdeps/include/python3.12/Python.h" ]] || {
  echo "OHOS Python headers are missing: $sysdeps/include/python3.12" >&2; exit 1;
}
[[ -f "$sysdeps/lib/libpython3.12.so" ]] || {
  echo "OHOS libpython is missing: $sysdeps/lib/libpython3.12.so" >&2; exit 1;
}

build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
prefix="$build/prefix"
cmake --fresh -S "$tcim/python" -B "$build/cmake" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$sdk/build/cmake/ohos.toolchain.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DCMAKE_CXX_FLAGS=-Wno-unused-command-line-argument \
  "-DCMAKE_MODULE_LINKER_FLAGS=-L$sysdeps/lib -lpython3.12" \
  -DPython_EXECUTABLE="$python" \
  -DPython_INCLUDE_DIR="$sysdeps/include/python3.12" \
  -DPython_LIBRARY="$sysdeps/lib/libpython3.12.so" \
  -Dpybind11_DIR="$("$python" -c 'import pybind11; print(pybind11.get_cmake_dir())')" \
  -DENABLE_DEV_CTRL=OFF
cmake --build "$build/cmake" --parallel
cmake --install "$build/cmake"

[[ ! -e "$output" ]] || { echo "Output exists; choose a new directory: $output" >&2; exit 1; }
mkdir -p "$output"
cp -a "$tcim/python/tcim_lite" "$output/tcim_lite"
extension=$(find "$prefix" -maxdepth 1 -name 'pytcim*.so' -print -quit)
[[ -n "$extension" ]] || { echo "pytcim extension was not installed" >&2; exit 1; }
cp "$extension" "$output/tcim_lite/pytcim.cpython-312-aarch64-linux-ohos.so"
echo "Built $output/tcim_lite"
