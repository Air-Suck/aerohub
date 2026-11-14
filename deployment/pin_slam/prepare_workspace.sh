#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workspace="${PIN_SLAM_WORKSPACE:-${repo_root}/log/workspaces/pin_slam}"
image="${ONBOARD_COMPUTER_IMAGE:-onboard_computer:v0.1.0}"
upstream_rel="src/basic_function/state_estimator/pin_slam/upstream"
upstream="${repo_root}/${upstream_rel}"
expected_upstream="e2b7fbf328e5fac1d638e217d70a69cbe9f30a52"
wheelhouse="${workspace}/wheelhouse"

if [[ ! -e "${upstream}/.git" ]]; then
  printf 'PIN-SLAM submodule is not initialized: %s\n' "${upstream_rel}" >&2
  printf 'Run: git submodule update --init -- %s\n' "${upstream_rel}" >&2
  exit 2
fi
if [[ "$(git -C "${upstream}" rev-parse --show-toplevel)" != "${upstream}" ]]; then
  printf 'PIN-SLAM upstream path is not an independent Git worktree: %s\n' "${upstream}" >&2
  exit 2
fi
gitlink=$(git -C "${repo_root}" ls-files -s "${upstream_rel}" | awk '$1 == "160000" {print $2}')
upstream_head=$(git -C "${upstream}" rev-parse HEAD)
if [[ "${gitlink}" != "${expected_upstream}" || "${upstream_head}" != "${expected_upstream}" ]]; then
  printf 'PIN-SLAM source mismatch: expected=%s gitlink=%s checkout=%s\n' \
    "${expected_upstream}" "${gitlink:-missing}" "${upstream_head}" >&2
  exit 2
fi
if [[ -n "$(git -C "${upstream}" status --short --untracked-files=no)" ]]; then
  printf 'PIN-SLAM upstream submodule has tracked modifications\n' >&2
  git -C "${upstream}" status --short --untracked-files=no >&2
  exit 2
fi

mkdir -p "${workspace}"/{home,ros}
docker image inspect "${image}" > "${workspace}/shared-image.json"

if [[ ! -d "${wheelhouse}" ]]; then
  printf 'PIN-SLAM wheelhouse is missing: %s\n' "${wheelhouse}" >&2
  exit 2
fi
(
  cd "${wheelhouse}"
  sha256sum --check "${repo_root}/deployment/pin_slam/bootstrap-wheels.sha256"
)

prepare_container="aerohub-pin-slam-prepare-$$"
cleanup_prepare_container() {
  docker rm --force "${prepare_container}" >/dev/null 2>&1 || true
}
trap cleanup_prepare_container EXIT INT TERM

docker run --rm --init --interactive --network host \
  --name "${prepare_container}" \
  --label aerohub.task=pin-slam-workspace-prepare \
  --label "aerohub.workspace=${workspace}" \
  --env HOME=/workspace/home \
  --env PIP_NO_CACHE_DIR=1 \
  --env "HOST_UID=$(id -u)" \
  --env "HOST_GID=$(id -g)" \
  --volume "${repo_root}:/repo:ro" \
  --volume "${workspace}:/workspace" \
  --entrypoint bash "${image}" -s <<'CONTAINER'
set -eo pipefail
restore_workspace_ownership() {
  for generated in /workspace/venv /workspace/ros /workspace/python-freeze.txt; do
    if [[ -e "${generated}" ]]; then
      chown -R "${HOST_UID}:${HOST_GID}" "${generated}"
    fi
  done
}
trap restore_workspace_ownership EXIT
if [[ ! -x /workspace/venv/bin/python ]]; then
  python3 -m venv --without-pip --system-site-packages /workspace/venv
  PYTHONPATH=/workspace/wheelhouse/pip-24.3.1-py3-none-any.whl \
    /workspace/venv/bin/python -m pip install \
    --no-index --find-links /workspace/wheelhouse \
    pip==24.3.1 setuptools==75.6.0 wheel==0.45.1
fi
source /workspace/venv/bin/activate
python -m pip install --no-index --find-links /workspace/wheelhouse \
  --require-hashes \
  --requirement /repo/deployment/pin_slam/requirements.lock
python - <<'PY'
from importlib import metadata
from pathlib import Path
import re
import sysconfig

def canonical(name):
    return re.sub(r"[-_.]+", "-", name).lower()

expected = {
    "pip": "24.3.1",
    "setuptools": "75.6.0",
    "wheel": "0.45.1",
}
for raw in Path("/repo/deployment/pin_slam/requirements.lock").read_text().splitlines():
    line = raw.strip()
    if not line or line.startswith("#"):
        continue
    requirement = line.split()[0]
    name, version = requirement.split("==", 1)
    expected[canonical(name)] = version

purelib = sysconfig.get_paths()["purelib"]
actual = {
    canonical(dist.metadata["Name"]): dist.version
    for dist in metadata.distributions(path=[purelib])
}
if actual != expected:
    missing = sorted(set(expected) - set(actual))
    extra = sorted(set(actual) - set(expected))
    mismatched = sorted(
        name for name in set(actual) & set(expected)
        if actual[name] != expected[name]
    )
    raise SystemExit(
        f"local distribution mismatch: missing={missing} extra={extra} "
        f"versions={[(name, expected[name], actual[name]) for name in mismatched]}"
    )
print(f"verified {len(actual)} locally installed distributions")
PY
set +u
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
set -u
colcon --log-base /workspace/ros/log build \
  --base-paths /repo/src/basic_function/state_estimator/pin_slam/ros2_adapter \
               /repo/src/basic_function/state_estimator/pin_slam/plugin \
  --build-base /workspace/ros/build \
  --install-base /workspace/ros/install \
  --merge-install \
  --packages-select uav_pin_slam_ros2 uav_pin_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
python -m pip freeze > /workspace/python-freeze.txt
CONTAINER
trap - EXIT INT TERM

for required in \
  "${workspace}/venv/bin/python" \
  "${workspace}/ros/install/setup.bash" \
  "${workspace}/python-freeze.txt"; do
  if [[ ! -e "${required}" ]]; then
    printf 'Required workspace artifact is missing: %s\n' "${required}" >&2
    exit 1
  fi
done

{
  printf 'repository=%s\n' "$(git -C "${repo_root}" rev-parse HEAD)"
  printf 'upstream_path=%s\n' "${upstream_rel}"
  printf 'upstream=%s\n' "${upstream_head}"
  printf 'upstream_url=%s\n' "$(git -C "${upstream}" remote get-url origin)"
  printf 'upstream_clean=true\n'
  printf 'shared_image=%s\n' "$(docker image inspect "${image}" --format '{{.Id}}')"
  printf 'requirements_lock_sha256=%s\n' \
    "$(sha256sum "${repo_root}/deployment/pin_slam/requirements.lock" | awk '{print $1}')"
  printf 'workspace=%s\n' "${workspace}"
  git -C "${repo_root}" status --short
} > "${workspace}/source-state.txt"

printf 'PIN-SLAM workspace prepared at %s\n' "${workspace}"
