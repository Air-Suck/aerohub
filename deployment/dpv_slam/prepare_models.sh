#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
target="$root/deployment/dpv_slam/models/dpvo.pth"
archive=$(mktemp "${TMPDIR:-/tmp}/dpvo-models.XXXXXX.zip")
wheelhouse="$root/deployment/dpv_slam/wheelhouse"
trap 'rm -f "$archive"' EXIT
mkdir -p "$(dirname "$target")"
url='https://www.dropbox.com/scl/fo/2g6e134ruh1acmaye5bmy/AK0DY9WLZ25elq-7N7sJcB0/models.zip?rlkey=4j336vcc4h324j4p4ji3ppl0t&dl=1'
curl --fail --location --retry 3 --output "$archive" "$url"
echo '89f43de2c92676ddcf7f49e8ae3f8940b7af73f549a7a5b8308850b67de78479  '"$archive" | sha256sum --check
unzip -p "$archive" dpvo.pth > "$target"
echo '30d02dc2b88a321cf99aad8e4ea1152a44d791b5b65bf95ad036922819c0ff12  '"$target" | sha256sum --check

mkdir -p "$wheelhouse"
while read -r checksum filename url; do
  curl --fail --location --continue-at - --retry 20 --retry-all-errors \
    --speed-limit 1024 --speed-time 180 --output "$wheelhouse/$filename" "$url"
  echo "$checksum  $wheelhouse/$filename" | sha256sum --check
done <<'WHEELS'
919387eb55330f5757c6bea9165c5ff5cfe63a642682ea788a6d472576d81737 einops-0.8.1-py3-none-any.whl https://files.pythonhosted.org/packages/87/62/9773de14fe6c45c23649e98b83231fffd7b9892b6cf863251dc2afa73643/einops-0.8.1-py3-none-any.whl
99f893e30497a4b66842821bac316386f7bd5c4f47ad35c9073ef089aa33af32 yacs-0.1.8-py3-none-any.whl https://files.pythonhosted.org/packages/38/4f/fe9a4d472aa867878ce3bb7efb16654c5d63672b86dc0e6e953a67018433/yacs-0.1.8-py3-none-any.whl
5a91f508183d5acd95d87651aeb1ddafdb7632728dc56b357552cffd9913cc71 pypose-0.7.3-py3-none-any.whl https://files.pythonhosted.org/packages/d8/0d/9222a064704d243ef899b8e321271e08399532539551497f70b6af2325c1/pypose-0.7.3-py3-none-any.whl
14432af305ea68627a084cd702124fd5d0c1f5b8a413b05f4e14757202d1cf6c numba-0.62.1-cp310-cp310-manylinux2014_x86_64.manylinux_2_17_x86_64.whl https://files.pythonhosted.org/packages/a1/13/9a27bcd0baeea236116070c7df458414336f25e9dd5a872b066cf36b74bf/numba-0.62.1-cp310-cp310-manylinux2014_x86_64.manylinux_2_17_x86_64.whl
f2d47f34e4029e6df3395de34cc1c66440a8d72712993a6e6168db228686711b llvmlite-0.45.1-cp310-cp310-manylinux2014_x86_64.manylinux_2_17_x86_64.whl https://files.pythonhosted.org/packages/a6/7b/6d7585998a5991fa74dc925aae57913ba8c7c2efff909de9d34cc1cd3c27/llvmlite-0.45.1-cp310-cp310-manylinux2014_x86_64.manylinux_2_17_x86_64.whl
cb6514b05ae6f54eef4f517429c072e70be2d7a0e3b3adc4879e0e57612c6b2e torch_scatter-2.1.2+pt25cu124-cp310-cp310-linux_x86_64.whl https://data.pyg.org/whl/torch-2.5.0%2Bcu124/torch_scatter-2.1.2%2Bpt25cu124-cp310-cp310-linux_x86_64.whl
WHEELS
