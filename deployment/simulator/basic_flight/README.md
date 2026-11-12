# Simulator backend minimum flight acceptance

This mission is the shared AirSim/ProjectAirSim acceptance workload. It uses only
AS2 services and native Takeoff, GoTo and Land actions. Sensor checks run independently and never provide motion input.

The simulator is selected with a profile passed to `simulator/simulator.sh`. Start
the matching `flight_board:v0.1.1` vehicle with the generated
`<run-dir>/runtime.json` mounted read-only and set
`SIMULATOR_RUNTIME_CONFIG` to that path. AirSim retains its flight_board clock and
sensor processes. ProjectAirSim uses the manager-owned ROS bridge and must not
start those AirSim-specific processes. Set `FLIGHT_BOARD_SENSORS=0` for the
ProjectAirSim vehicle. Select `FLIGHT_BOARD_IMAGE=flight_board:v0.1.1` in entrypoints
that support this variable; hard-coded image references require an explicit change.
The earlier `flight_board:sim-manager-dev` tag was used during development.
Build/load instructions and the new tag's validation scope are documented in
[flight_board](../../flight_board/README.md#使用固定镜像).

This source addition is not itself a flight result. A run is accepted only when
`result.json` confirms native landing and disarm and the separate sensor report
contains fresh pose, IMU, RGB, depth, CameraInfo and LiDAR samples from one clock
authority.

ProjectAirSim 的统一传感器接口使用 `projectairsim_contract_topics.json`；
`projectairsim_topics.json` 检查原生 bridge。`verify_sensor_contract.py` 对原生与统一
话题做真实样本配对，验证转换、时间戳及配置静态 TF。冷启动后使用
`projectairsim_calibration_topics.json` 单独复查原生/统一接口的标定晚订阅。
实现、构建补丁与支持边界见 `simulator/projectairsim/SENSORS.md`。

HUD observation flight: `display_flight.py --output <directory>` reuses the
same AS2 mission, adds a six-second hold after GoTo, sends display timer events,
and records HUD speed plus AS2 pose in `samples.jsonl`. Run in the configured
ROS mission environment with the simulator, PX4 and AS2 already ready. It does
not start flight infrastructure. GoTo performs normal arrival deceleration;
this is not a separate emergency braking behavior.

With display enabled, the simulator manager owns a service-only ProjectAirSim
kinematics reader (`display-telemetry` auxiliary). HUD speed is the magnitude
of upstream `twist.linear` in m/s, not a rendering-frame position derivative or
PX4 estimate. The reader uses `connect_services()` to preserve the ROS bridge's
single-consumer native topic connection. Telemetry older than one wall second
is unavailable. Mission timer uses wall time and does not command flight.
