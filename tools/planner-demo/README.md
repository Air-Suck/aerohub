# Headless PX4/Gazebo planner demos

This directory records the two planner reproductions without a desktop session.
Gazebo Harmonic runs with `--headless-rendering` (enabled by PX4's `HEADLESS=1`),
publishes an RGB camera stream on `/demo/camera/image`, and
`capture_gz_images` converts each transport message into a PPM frame. OpenCV
adds the title overlay and creates an intermediate stream, then standard FFmpeg
encodes H.264 Constrained Baseline, YUV 4:2:0 MP4 with the `moov` atom moved to
the front (`faststart`). The Swarm demo overlays the official 12-edge formation
graph using the seven vehicles' current actual odometry: six center-to-ring edges
and a six-edge outer hexagon, with no markers, labels, or history trails. FFmpeg
then interpolates the safe 3 fps capture to 12 fps. Temporary frames and the
intermediate file are removed only after the final video has passed a complete
FFmpeg decode.

Run on RKOH:

```bash
cd /home/noah/code/UAV/demo
./run_catora_demo.sh
./run_swarm_demo.sh
```

Outputs:

- `catora_px4_gazebo_headless.mp4`
- `swarm_formation_px4_gazebo_headless.mp4`

The Swarm demo uses the official `34 m × 15 m × 3 m` random-map parameters:
60 cylinders and 20 elliptical rings. The upstream generator uses
`random_device`; this reproduction fixes seed `31` so the Gazebo geometry and
planner point cloud are repeatable. Seven PX4 vehicles hold a `1.0×` hexagon
and receive only the global destination `(24,0)`. Three pillars intersect
nominal straight vehicle paths, so the formation must deform locally instead
of following a corridor. To keep seven-vehicle SITL stable under software
rendering, the camera is activated after takeoff at `640×360`, 3 fps.

Validate existing outputs without rerunning the simulations:

```bash
docker run --rm -v /home/noah/code/UAV/demo:/demo --entrypoint python3 \
  uav/catora:noetic-px4 /demo/verify_videos.py \
  /demo/catora_px4_gazebo_headless.mp4 \
  /demo/swarm_formation_px4_gazebo_headless.mp4
```

The scripts use the existing stage-2 verification programs as their pass/fail
criterion. They run serially because both stacks use host networking and the
same ROS/MAVLink/Gazebo ports.
