# FAST-LIVO2 接入与验收

FAST-LIVO2 同时提供视觉-惯性-激光状态估计和配准点云。AeroHub 保持单一算法进程：状态通过原生 AS2 State Estimator 插件发布，配准点通过独立 Perception 接口发布。算法不向 PX4 回灌外部里程计。

## 来源与运行边界

- ROS 2 算法：`ros2_upstream/`，锁定 `SuperLDG/FASTLIVO2_ROS2@a713004f0ba0624c8fb80d85c7047fe62523c6fb`。
- 依赖：`dependencies/livox_ros_driver2/`、`dependencies/rpg_vikit/`、`dependencies/sophus/`，均为锁定 Git 子模块。
- AeroHub 代码：`ros2_adapter/` 负责 AirSim ROS 2 消息适配，`plugin/` 负责 AS2 状态接入。
- 运行环境仅使用 `flight_board:v0.1.0` 和 `onboard_computer:v0.1.0`，源码在外部工作区补编译并只读挂载。不得构建或使用 FAST-LIVO2 专用镜像。
- PX4 使用 `flight_board:v0.1.0` 内置版本；不得改用仓库 `px4/`。
- `upstream/`、`ros1_proxy/`、`Dockerfile.ros1`、`Dockerfile.as2`、`compose.yaml` 与 `smoke.sh` 是历史 ROS 1/custom-image 实现，不是当前构建、运行或验收入口。

## 接口和坐标

| 接口 | 含义 |
| --- | --- |
| `/<namespace>/fast_livo2/odometry` | FAST-LIVO2 原生局部世界中的算法状态 |
| `/<namespace>/self_localization/pose` | AS2 权威位姿，表达于 canonical ENU `earth/map/odom` |
| `/<namespace>/self_localization/twist` | 机体原点速度，表达于 `base_link` |
| `/<namespace>/perception/fast_livo2/registered_points` | 原生 LIVO 世界中的配准点，不是占据图或 ESDF |

`earth_from_livo_world` 将 FAST-LIVO2 的 FLU 世界转换到 AS2/PX4 使用的 ENU 世界。AS2 的 `earth -> map` 和 `map -> odom` 为恒等关系，`odom -> base_link` 使用转换后的 ENU 位姿；`odom -> livo_world` 保存该世界转换的静态 TF。因此控制器和 PX4 对 `odom` 的解释一致，而点云无需改写原生数据。

插件保留测量时间并拒绝错误帧、非有限值、非单位四元数、过期、未来、重复和乱序状态。`publish_hz` 必须为 `0`，避免宿主重复发布陈旧状态。AirSim 1.8.1 不提供 LiDAR 逐点时间，当前适配明确采用 instantaneous scan，不伪造时间偏移。

## 构建

在仓库根目录执行：

```bash
export FAST_LIVO2_WORKSPACE="$PWD/log/workspaces/fast_livo2"
deployment/fast_livo2/prepare_workspace.sh

export AS2_EXECUTION_WORKSPACE="$PWD/log/workspaces/as2_execution"
deployment/onboard_computer/prepare_execution_overlay.sh
```

两个脚本均使用固定共享镜像。FAST-LIVO2 脚本校验算法与依赖 gitlink，AS2 脚本从锁定 Fork commit 导出源码，避免把子模块工作区中的临时改动带入构建。

适配层回归：

```bash
FAST_LIVO2_WORKSPACE="$FAST_LIVO2_WORKSPACE" \
  deployment/fast_livo2/test_adapters.sh
```

## AirSim + PX4 SITL 验收

验收必须使用有屏模式，并为每次运行选择新的证据目录：

```bash
export DISPLAY=:3
export AIRSIM_RENDER_MODE=windowed
export FAST_LIVO2_WORKSPACE="$PWD/log/workspaces/fast_livo2"
export AS2_EXECUTION_WORKSPACE="$PWD/log/workspaces/as2_execution"

python3 deployment/fast_livo2/run_airsim.py run \
  --scenario shadow --output log/fast_livo2/shadow
python3 deployment/fast_livo2/run_airsim.py run \
  --scenario authority --output log/fast_livo2/authority
python3 deployment/fast_livo2/run_airsim.py run \
  --scenario sensor_fault --output log/fast_livo2/sensor-fault
python3 deployment/fast_livo2/run_airsim.py run \
  --scenario algorithm_fault --output log/fast_livo2/algorithm-fault
```

运行器先验证 AirSim 窗口和 PX4 `pre_flight_checks_pass`，再启动传感器、算法和 AS2。AirSim 1.8.1 偶尔无法建立航向参考；这种情况会在上层链启动前失败并保留证据，需要使用全新输出目录重启完整 AirSim/PX4 运行，不能绕过 preflight。

故障场景在空中真实停止传感器或算法容器，检测对应 ROS 流停止后请求悬停并直接发出 PX4 降落指令。成功必须同时满足降落、解除武装和无本任务遗留资源。门限与证据要求见 `deployment/fast_livo2/validation.yaml`。

当前已通过同源的 shadow、authority、传感器中断和算法退出矩阵。真实数据集和真机仍不在本次验收范围内。
