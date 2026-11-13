# Object Detection Tool: ROS RGB-D measurements

This Tool calls a separately deployed YOLO26n-seg Inference ROS service, selects
the configured category, measures depth inside each instance mask and reports
unfiltered 3D positions. It subscribes to standard ROS RGB/depth/CameraInfo and
queries historical TF; there is no AirSim API, settings.json, Elastic dependency,
target KF, identity association, predicted velocity or motion-command publisher.

## Files and interfaces

- `interfaces/`: uav_object_detection_interfaces, typed DetectObjects service and
  ObjectDetectionFrame/ObjectDetection measurement messages.
- `ros2_adapter/`: uav_object_detection node and pure depth/projection routines.
- Deployment/parameter example: deployment/object_detection/.
- Capability: cards/tool/object-detection.yaml.

`<namespace>/tools/object_detection/detect` takes request_id, generation and an
RGB8/BGR8 Image. It returns per-instance category/confidence/box/mask plus:

- position_valid and position in meters, with the output frame and **depth capture
  stamp** in result.header;
- original RGB image_header and paired depth_header; caller/tool/worker generation
  and model identity remain distinct;
- selected depth, depth spread and valid pixel count; spread is diagnostic, not
  a complete localization covariance or another confidence score;
- position_error if depth/calibration/TF is unavailable. Never consume a position
  when position_valid is false. A failed 3D result may retain its 2D detections.

An empty detection list with valid=true is a measured no-detection result, not a
predicted target. A downstream navigation component owns association, KF and loss
policy. This Tool does not publish target odometry or refresh target observations.

## Inputs and launch

Build and source the ROS overlay as described in the model README, then run:

```bash
ros2 run uav_object_detection object_detection --ros-args -r __ns:=/drone0 \
  --params-file deployment/object_detection/params.example.yaml
```

The example enables the latest-only RGB topic stream and simulation time, assumes
an existing /clock and a provider for drone0/odom -> drone0/FrontCamera_optical,
and references AirSim-style ROS topic names. All topics/frames are parameters;
no algorithm-side simulator dependency exists. For a physical camera, configure
its registered/rectified topics, actual CameraInfo, output frame and clock.
Default stream_enabled=false supports service-only use. registered_depth must be
explicitly true after checking RGB/depth registration; the Tool cannot infer it.

Supported depth: 32FC1 meters or 16UC1 with configurable scale (default .001 m).
Default depth is planar optical Z; set depth_is_perspective only for radial range.
RGB/depth must have matching registered geometry, optical frame and intrinsics.
Only rectified/zero-distortion images are supported. CameraInfo is selected no
later than each acquisition stamp, rather than reusing future calibration.

Depth pairing keeps stage2's 0.05 s skew constraint, 60-frame bounded history and
nearest matching sample. TF is queried at the exact integer depth stamp with a
0.25 s arrival wait inherited from the stage2/SUPER integration. No latest lookup,
extrapolation, fixed source-age deadline or extra observation-confirmation rule
is introduced. The complete camera TF contains the installation extrinsic once;
do not apply an additional camera-to-body transform after it.

Measurement depth functions preserve the pinned upstream mask erosion/ROI fallback
and histogram rules, with source/license notices installed by the package.
Existing stage2 pose/registration limitations are not solved by this migration.

## Lifecycle and validation

`tools/object_detection/enable` (SetBool) stops/resumes new calls and streaming.
Stop clears pending input, changes tool generation and rejects late results;
it does not cancel an already running GPU kernel or stop aircraft execution.
On clock rollback, pending data, calibration/depth histories and TF are cleared.
The status topic is `tools/object_detection/status`; streamed measurements are
`perception/object_detection/detections`. Local request locking and latest-only
queuing do not constitute global GPU/mission resource arbitration.

The independent ROS probe at deployment/object_detection/ros_probe.py uses recorded
RGB and real CUDA inference, but deliberately synthetic depth/TF. It checks wire
semantics and known geometry, not sensor accuracy. Tests in tests/ exercise actual
ROS messages/TF2, unit conversion, capture-time interpolation, missing inputs and
cancellation. See model validation.json for executed checks and remaining gaps.
