#!/usr/bin/env bash
set -euo pipefail

if (($# < 2 || $# > 3)); then
  echo "Usage: $0 INPUT_MP4 OUTPUT_MP4 [OUTPUT_FPS]" >&2
  exit 2
fi

demo_dir=/home/noah/code/UAV/demo
image=uav/demo-ffmpeg:22.04
input=$(basename "$1")
output=$(basename "$2")
output_fps=${3:-}
video_filter=()
if [[ -n $output_fps ]]; then
  video_filter=(-vf "minterpolate=fps=${output_fps}:mi_mode=mci:mc_mode=aobmc:me_mode=bidir:vsbmc=1")
fi

docker image inspect "$image" >/dev/null 2>&1 || \
  docker build -t "$image" -f "$demo_dir/Dockerfile.ffmpeg" "$demo_dir"

docker run --rm --user "$(id -u):$(id -g)" -v "$demo_dir:/demo" "$image" \
  -hide_banner -loglevel error -y -i "/demo/$input" -map 0:v:0 -an \
  "${video_filter[@]}" \
  -c:v libx264 -preset medium -crf 18 -profile:v baseline -level:v 3.1 \
  -pix_fmt yuv420p -movflags +faststart "/demo/$output"

docker run --rm --user "$(id -u):$(id -g)" -v "$demo_dir:/demo" "$image" \
  -v error -i "/demo/$output" -f null -
