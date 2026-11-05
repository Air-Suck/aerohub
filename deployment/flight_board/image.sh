#!/usr/bin/env bash
# Transfer the frozen runtime without rebuilding PX4 or DDS.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
image=flight_board:airsim-px4-humble-threaded-20261105
expected=sha256:8c33881fde6985fe5a43a894bde075471df85064e45ac8e5de28efebcc737cd2
archive=${2:-$root/artifacts/images/flight_board-humble-threaded-20261105-amd64.tar}
check_image() {
  actual=$(docker image inspect "$image" --format '{{.Id}}')
  if [[ "$actual" != "$expected" ]]; then
    echo "Image ID mismatch: $actual (expected $expected)" >&2
    exit 1
  fi
}
case ${1:-help} in
  export)
    check_image
    mkdir -p "$(dirname "$archive")"
    if [[ -e "$archive" || -e "$archive.sha256" ]]; then
      echo "Archive already exists: $archive" >&2
      exit 1
    fi
    docker image save -o "$archive.partial" "$image"
    mv "$archive.partial" "$archive"
    (cd "$(dirname "$archive")" && sha256sum "$(basename "$archive")") > "$archive.sha256"
    ;;
  load)
    (cd "$(dirname "$archive")" && sha256sum -c "$(basename "$archive").sha256")
    docker image load -i "$archive"
    check_image
    ;;
  check) check_image ;;
  *)
    echo "Usage: bash deployment/flight_board/image.sh {check|export|load} [archive.tar]"
    [[ ${1:-help} == help ]]
    ;;
esac
