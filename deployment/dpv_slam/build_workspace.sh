#!/usr/bin/env bash
set -euo pipefail

repo=${1:-/workspace/aerohub}
workspace=${2:-/workspace/dpv}
source_root="$repo/src/basic_function/state_estimator/dpv_slam/upstream"
python_root="$workspace/python"
cuda_root="$workspace/cuda"
wheelhouse="$workspace/wheelhouse"
build_root="$workspace/build"

cleanup_ownership() {
  if [[ -n ${HOST_UID:-} && -n ${HOST_GID:-} ]]; then
    chown -R "$HOST_UID:$HOST_GID" "$workspace"
  fi
}
trap cleanup_ownership EXIT

test -f "$source_root/setup.py"
test "${DPVO_COMMIT:?DPVO_COMMIT must be verified by the host launcher}" = \
  0ac95b656d1fda91c271d2a106460d19ad966fc7
echo '30d02dc2b88a321cf99aad8e4ea1152a44d791b5b65bf95ad036922819c0ff12  '"$repo/deployment/dpv_slam/models/dpvo.pth" \
  | sha256sum --check

rm -rf "$python_root"
mkdir -p "$python_root" "$cuda_root" "$wheelhouse" "$build_root"

cuda_manifest="$repo/deployment/dpv_slam/cuda-build-packages.sha256"
cuda_marker="$cuda_root/.aerohub-manifest.sha256"
expected_cuda_manifest=$(sha256sum "$cuda_manifest" | cut -d' ' -f1)
if [[ ! -f $cuda_marker ]] || [[ $(cat "$cuda_marker") != "$expected_cuda_manifest" ]]; then
  rm -rf "$cuda_root"
  mkdir -p "$cuda_root" "$workspace/cuda-packages"

  download_in_ranges() {
    local url=$1 output=$2
    local partial="$output.partial"
    local chunk="$output.chunk"
    local total start end expected actual
    total=$(curl --fail --silent --show-error --location --head \
      --retry 10 --retry-all-errors "$url" \
      | tr -d '\r' | awk 'tolower($1) == "content-length:" {value=$2} END {print value}')
    [[ $total =~ ^[0-9]+$ ]] && ((total > 0))
    if [[ -f $output ]] && ! sha256sum --check --status \
      <(printf '%s  %s\n' "$checksum" "$output"); then
      mv "$output" "$partial"
    fi
    start=0
    if [[ -f $partial ]]; then
      start=$(stat -c %s "$partial")
    fi
    ((start <= total))
    while ((start < total)); do
      end=$((start + 4 * 1024 * 1024 - 1))
      if ((end >= total)); then
        end=$((total - 1))
      fi
      expected=$((end - start + 1))
      curl --fail --silent --show-error --location \
        --retry 20 --retry-all-errors --speed-limit 1024 --speed-time 180 \
        --range "$start-$end" --output "$chunk" "$url"
      actual=$(stat -c %s "$chunk")
      if ((actual != expected)); then
        printf 'range %s-%s returned %s bytes, expected %s\n' \
          "$start" "$end" "$actual" "$expected" >&2
        exit 1
      fi
      cat "$chunk" >> "$partial"
      start=$((end + 1))
      printf 'downloaded %s/%s bytes: %s\n' "$start" "$total" "$output"
    done
    mv "$partial" "$output"
    rm -f "$chunk"
  }

  while read -r checksum package; do
    package_path="$workspace/cuda-packages/$package"
    if ! echo "$checksum  $package_path" | sha256sum --check --status; then
      download_in_ranges \
        "https://conda.anaconda.org/nvidia/linux-64/$package" "$package_path"
    fi
    echo "$checksum  $package_path" | sha256sum --check
  done < "$cuda_manifest"
  for package in "$workspace"/cuda-packages/*.tar.bz2; do
    tar -xjf "$package" -C "$cuda_root"
  done
  printf '%s\n' "$expected_cuda_manifest" > "$cuda_marker"
fi

export CUDA_HOME="$cuda_root"
export PATH="$python_root/bin:$python_root/ninja/data/bin:$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib:$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$python_root:$source_root:${PYTHONPATH:-}"
export TORCH_CUDA_ARCH_LIST=8.9
export MAX_JOBS=${MAX_JOBS:-4}
export DPV_WHEEL_MANIFEST="$repo/deployment/dpv_slam/python-wheels.sha256"
export DPV_PYTHON_ROOT="$python_root"

wheel_manifest="$DPV_WHEEL_MANIFEST"
while read -r checksum filename url; do
  echo "$checksum  $wheelhouse/$filename"
done < "$wheel_manifest" | sha256sum --check
pip_wheel="$wheelhouse/pip-24.3.1-py3-none-any.whl"
PYTHONPATH="$pip_wheel" python3 -m pip install \
  --upgrade --no-index --no-deps --target "$python_root" \
  "$wheelhouse"/*.whl
command -v ninja
ninja --version

rm -rf "$build_root/dpvo-extensions"
mkdir -p "$build_root/dpvo-extensions"
(
  cd "$build_root/dpvo-extensions"
  DPVO_SOURCE="$source_root" python3 \
    "$repo/deployment/dpv_slam/build_extensions.py" \
    build_ext --build-lib "$python_root"
)

rm -rf "$workspace/ros/build" "$workspace/ros/install" "$workspace/ros/log"
mkdir -p "$workspace/ros"
set +u
source /opt/ros/humble/setup.bash
source /opt/px4_ws/install/setup.bash
set -u
colcon --log-base "$workspace/ros/log" build \
  --base-paths "$repo/src/basic_function/state_estimator/dpv_slam/ros2_adapter" \
  --build-base "$workspace/ros/build" \
  --install-base "$workspace/ros/install" \
  --merge-install --packages-select uav_dpv_slam_ros2

rm -rf "$workspace/plugin/build" "$workspace/plugin/install" "$workspace/plugin/log"
mkdir -p "$workspace/plugin"
set +u
source /root/aerostack2_ws/install/setup.bash
set -u
colcon --log-base "$workspace/plugin/log" build \
  --base-paths "$repo/src/basic_function/state_estimator/dpv_slam/plugin" \
  --build-base "$workspace/plugin/build" \
  --install-base "$workspace/plugin/install" \
  --merge-install --packages-select uav_dpv_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

python3 -m pip freeze --path "$python_root" | LC_ALL=C sort \
  > "$workspace/python-freeze.txt"
python3 - <<'PY'
from importlib import metadata
import os
from pathlib import Path

from packaging.utils import canonicalize_name, parse_wheel_filename

manifest = Path(os.environ["DPV_WHEEL_MANIFEST"])
expected = {}
for raw in manifest.read_text().splitlines():
    _, filename, _ = raw.split(maxsplit=2)
    name, version, _, _ = parse_wheel_filename(filename)
    expected[canonicalize_name(name)] = str(version)
installed = {
    canonicalize_name(distribution.metadata["Name"]): distribution.version
    for distribution in metadata.distributions(path=[os.environ["DPV_PYTHON_ROOT"]])
}
if installed != expected:
    missing = sorted(set(expected) - set(installed))
    extra = sorted(set(installed) - set(expected))
    mismatched = sorted(
        name for name in set(expected) & set(installed)
        if expected[name] != installed[name]
    )
    raise SystemExit(
        f"DPV-SLAM wheel mismatch: missing={missing} extra={extra} "
        f"mismatched={[(name, expected[name], installed[name]) for name in mismatched]}"
    )
PY
python3 - <<'PY'
import importlib
import json
import os
from pathlib import Path

import torch

for name in ("cuda_corr", "cuda_ba", "lietorch_backends", "torch_scatter"):
    importlib.import_module(name)
source = Path(os.environ["DPVO_SOURCE"])
record = {
    "dpvo_commit": os.environ["DPVO_COMMIT"],
    "torch": torch.__version__,
    "cuda": torch.version.cuda,
    "cuda_available": torch.cuda.is_available(),
    "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
}
if not record["cuda_available"]:
    raise SystemExit("CUDA is unavailable in the shared-container workspace")
Path("/workspace/dpv/provenance.json").write_text(json.dumps(record, indent=2) + "\n")
print(json.dumps(record, indent=2))
PY

set +u
source "$workspace/ros/install/setup.bash"
source "$workspace/plugin/install/setup.bash"
set -u
ros2 pkg prefix uav_dpv_slam_ros2
ros2 pkg prefix uav_dpv_slam
