#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workspace="${PIN_SLAM_WORKSPACE:-${repo_root}/log/workspaces/pin_slam}"
wheelhouse="${workspace}/wheelhouse"
mkdir -p "${wheelhouse}"

download() {
  local filename=$1
  local url=$2
  local checksum=$3
  local output="${wheelhouse}/${filename}"
  if ! printf '%s  %s\n' "${checksum}" "${output}" | sha256sum --check --status; then
    curl --fail --location --retry 5 --retry-all-errors \
      --output "${output}.tmp" "${url}"
    printf '%s  %s\n' "${checksum}" "${output}.tmp" | sha256sum --check -
    mv "${output}.tmp" "${output}"
  fi
}

download \
  pip-24.3.1-py3-none-any.whl \
  https://files.pythonhosted.org/packages/ef/7d/500c9ad20238fcfcb4cb9243eede163594d7020ce87bd9610c9e02771876/pip-24.3.1-py3-none-any.whl \
  3790624780082365f47549d032f3770eeb2b1e8bd1f7b2e02dace1afa361b4ed
download \
  setuptools-75.6.0-py3-none-any.whl \
  https://files.pythonhosted.org/packages/55/21/47d163f615df1d30c094f6c8bbb353619274edccf0327b185cc2493c2c33/setuptools-75.6.0-py3-none-any.whl \
  ce74b49e8f7110f9bf04883b730f4765b774ef3ef28f722cce7c273d253aaf7d
download \
  wheel-0.45.1-py3-none-any.whl \
  https://files.pythonhosted.org/packages/0b/2c/87f3254fd8ffd29e4c02732eee68a83a1d3c346ae39bc6822dcbcb697f2b/wheel-0.45.1-py3-none-any.whl \
  708e7481cc80179af0e556bbf0cc00b8444c7321e2700b8d8580231d13017248

PYTHONPATH="${wheelhouse}/pip-24.3.1-py3-none-any.whl" python3 -m pip download \
  --only-binary=:all: \
  --no-deps \
  --require-hashes \
  --extra-index-url https://download.pytorch.org/whl/cu124 \
  --dest "${wheelhouse}" \
  --requirement "${repo_root}/deployment/pin_slam/requirements.lock"

(
  cd "${wheelhouse}"
  sha256sum --check "${repo_root}/deployment/pin_slam/bootstrap-wheels.sha256"
)
printf 'PIN-SLAM wheelhouse prepared at %s\n' "${wheelhouse}"
