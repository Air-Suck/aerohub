# Simulator backend minimum flight acceptance

This mission is the shared AirSim/ProjectAirSim acceptance workload. It uses only
AS2 services and native Takeoff, GoTo and Land actions. Sensor checks run independently and never provide motion input.

The simulator is selected with a profile passed to `simulator/simulator.sh`. Start
the matching `flight_board:sim-manager-dev` vehicle with the generated
`<run-dir>/runtime.json` mounted read-only and set
`SIMULATOR_RUNTIME_CONFIG` to that path. AirSim retains its flight_board clock and
sensor processes. ProjectAirSim uses the manager-owned ROS bridge and must not
start those AirSim-specific processes.

This source addition is not itself a flight result. A run is accepted only when
`result.json` confirms native landing and disarm and the separate sensor report
contains fresh pose, IMU, RGB, depth, CameraInfo and LiDAR samples from one clock
authority.

ProjectAirSim 的统一传感器接口使用 `projectairsim_contract_topics.json`；
`projectairsim_topics.json` 检查原生 bridge。`verify_sensor_contract.py` 对原生与统一
话题做真实样本配对，验证转换、时间戳及配置静态 TF。冷启动后使用
`projectairsim_calibration_topics.json` 单独复查原生/统一接口的标定晚订阅。
实现、构建补丁与支持边界见 `simulator/projectairsim/SENSORS.md`。
