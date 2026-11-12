# Simulator Manager

用户配置入口为根目录 [config/](../../config/README.md)，默认读取
`config/simulator/default.yaml`；同目录其他场景 YAML 使用 `--config` 显式选择。
普通场景 YAML 可纳入 Git，只有 `*.local.yaml` 被忽略。启动命令为
`bash simulator/simulator.sh start`，可用 `--config` 选择另一份 YAML。
YAML 可直接配置安装路径、场景、车辆与常用传感器参数；bridge 参数在内部生成。

统一入口为 `simulator/simulator.sh`，通过配置的 `backend` 选择 `airsim` 或
`projectairsim`。未填写 backend 的历史 profile 继续按 AirSim 处理。

Manager 管理仿真进程、原生场景/settings、PX4 连接清单和仿真侧 adapter。
`deployment/flight_board` 消费生成的 `runtime.json` 启动 PX4；AS2 的 Pixhawk
platform、原生 Behavior 和控制器沿用现有部署。Manager 不提供运动控制接口。

```text
工作站任务 → AS2 Behavior / 控制器 → Pixhawk platform → PX4
                                                    ↕ MAVLink/HIL
                                           AirSim / ProjectAirSim
                                                    ↓
                                       后端 adapter → ROS 传感器与 /clock
```

## 复用上游

ProjectAirSim 场景由原生 Python `World` 加载，保留 JSONC、robot-config 展开和 schema
校验；本项目仅在展开后同步 profile 中的 PX4 连接设置，不维护第二套场景解析器。
`prepare_scene.py` 从上游 PX4 示例生成独立场景，可选复制上游 LiDAR 定义。
二进制、Python client 和 ROS bridge 应使用匹配的源码版本。

传感器与时钟使用上游 `ros/projectairsim_ros2_cpp`，不另写数据采集 bridge。
桥接器的控制 service/action 不用于本项目飞行执行。`publish_tf=false`，避免仿真
truth TF 与 PX4/AS2 状态权威冲突。AS2 状态仍来自 PX4 raw odometry。

AS2 的 Gazebo、multirotor simulator 和 Pixhawk 平台各自处理对应平台；这里保持
Pixhawk 平台，通过切换 PX4 的仿真环境接入 ProjectAirSim，无需增加 AS2 平台插件。

## 旧 profile / 资源配置方式

以下保留给原有部署脚本；新用户按根目录 config/ 的 YAML 操作即可。

在现有本地资源配置 `simulator/airsim/config/resources.local.yaml` 中登记以下资源，
格式参考相邻的 `resources.example.yaml`：

- `projectairsim_blocks`：打包后的直接可执行文件 `Blocks/Binaries/Linux/Blocks`。
- `projectairsim_scene_px4`：原生 PX4 场景 JSON/JSONC。
- `projectairsim_python_client`：匹配版本的 `client/python/projectairsim/src`。
- `projectairsim_python`：安装了 client 依赖的 Python/venv 解释器。
- `projectairsim_ros2_setup`：bridge 外部 overlay 的 `install/setup.bash`。

以下变量由操作者设置为自己的源码、解释器及运行目录：

```bash
PYTHONPATH="$PROJECTAIRSIM_SOURCE/client/python/projectairsim/src" \
  "$PROJECTAIRSIM_PYTHON" simulator/projectairsim/prepare_scene.py \
  --sim-config-path "$PROJECTAIRSIM_SOURCE/client/python/example_user_scripts/sim_config" \
  --output "$SCENE_OUTPUT" --lidar

bash simulator/projectairsim/build_bridge.sh "$PROJECTAIRSIM_SOURCE" "$BRIDGE_WORKSPACE"

bash simulator/simulator.sh check \
  --profile simulator/projectairsim/config/basic_px4.json --run-dir "$RUN_DIR"
bash simulator/simulator.sh start \
  --profile simulator/projectairsim/config/basic_px4.json --run-dir "$RUN_DIR" \
  --render-mode windowed --timeout 60
bash simulator/simulator.sh status --run-dir "$RUN_DIR"
bash simulator/simulator.sh stop --run-dir "$RUN_DIR"
```

Bridge 构建默认复用固定 ROS 工具镜像，源码只读挂载，产物在外部 workspace。
可通过 `PROJECTAIRSIM_ROS_IMAGE` 选择已有兼容镜像。离线环境可将本地 FetchContent
源码缓存作为额外 `-DFETCHCONTENT_SOURCE_DIR_...` 参数传给构建脚本。
有屏启动需使用当前桌面会话的 DISPLAY/XAUTHORITY。

`start` 生成 `runtime.json`、`resolved_scene.json`、`state.json` 和各进程日志。
`status` 检查所记录 PID 的启动标识和 adapter 存活；ready 表示场景加载完成且进程
启动成功，飞行就绪须另查 PX4、AS2 状态及传感器。它不是持续自动恢复 watchdog。
`stop` 只清理本次启动并记录的进程组。先确认车辆已降落、disarm，再停止部署与仿真。

## 接口契约

`runtime.json` schema_version=1，包含 backend 和 vehicles。每个 vehicle 提供 name、
namespace，以及 PX4 的 tcp_port、control_local_port、control_remote_port、lockstep。
ProjectAirSim 场景中的 robot 集合必须与 vehicles 一致，controller 必须是原生
`px4-api`。连接配置在场景副本中同步，不改上游原始场景文件。

飞控镜像使用 `flight_board:v0.1.1`，构建与加载见
[flight_board 使用说明](../../deployment/flight_board/README.md#使用固定镜像)。
支持镜像环境变量的部署入口显式设置 `FLIGHT_BOARD_IMAGE=flight_board:v0.1.1`；
manager 不会替部署入口切换镜像。

将 runtime.json 只读挂载到 flight_board，并设置 `SIMULATOR_RUNTIME_CONFIG` 与
`VEHICLE_NAME`。ProjectAirSim 必须设置 `FLIGHT_BOARD_SENSORS=0`，由 manager-owned
原生 bridge 提供唯一 `/clock`，不再启动 AirSim 的 sensor/clock 进程。
namespace、PX4 system id、AS2 fleet 配置与 ROS_DOMAIN_ID 仍需由部署层保持一致。

传感器保留原生 bridge 话题，同时可通过 `adapter.sensor_contract=true` 提供与现有
AirSim adapter 相同的 `/<namespace>/sensor_measurements/...` 命名、编码与静态 TF。
接口、支持范围和补丁说明见 [传感器接入](../projectairsim/SENSORS.md)，验证配置见
`deployment/simulator/basic_flight/projectairsim_contract_topics.json`。下游算法不能仅因能收到
消息就视为完成兼容：必须核对相机 optical frame、LiDAR/IMU 外参、深度含义、单位及
时间戳。当前上游向量转换为 `(x,-y,-z)`，不能直接声明为 NED→ENU，亦不能用
actual_pose 替换 AS2 权威位姿。

## 当前验证边界（2026-11-12）

源码版本 `92032a6c4d98724d6f23b7346b8c01be955910c3`（`v1.1.0-10-g92032a6`）的
Blocks Development 包构建完成；原生 ROS bridge 的 colcon 汇总为 41 tests、0 failures、
0 errors、0 skipped。有屏 Blocks + PX4 SITL + 原生 AS2 的 2 m 起飞、水平 3 m 到点、
降落/disarm 通过，GoTo 完成后独立检查 0.5 m 目标容差。

初次传感器验收因 CameraInfo 缺失及 LiDAR 末帧空回波失败，原始失败报告保留。
后续用可重放补丁修复标定 QoS 和订阅回调注册竞态；标准接口加入标定、静态 TF、
RGB8 转换与原生测量保留。修复后起飞前八类接口检查、逐样本/配置 TF 比对，以及
基础飞行期间 40 秒检查均通过，`/clock` 只有一个发布者。实际配对比例超过99%。
LiDAR 空回波保留并计数，验收窗口内要求存在有效点，不能以最后一帧代表整段数据。

这验证了接口传输、配置外参与基础飞行共存。已知几何/运动下的物理标定、传感器
reset/断连完整矩阵、实际下游算法、同套 AirSim 对照及正式四方任务拓扑尚未完成。
本次诊断任务在 AS2 容器中执行，不作为工作站任务部署验收。

最新原始结果位于被忽略的 `log/projectairsim/sensors-acceptance/`，早期结果位于
`log/projectairsim/windowed-px4/`，过程记录见
`skill/temp/projectairsim-manager/progress.md`。失败报告保留，不将基础飞行和接口通过
等同于所有算法、所有传感器或多机兼容完成。

### 连接编辑器 Play 实例

`bash simulator/simulator.sh attach --config config/simulator/blocks.local.yaml`
复用已有 ProjectAirSim 服务，执行与 start 相同的原生场景加载和传感器桥接。
会话记录 `owns_host: false`，失败清理和 stop 均只终止本会话辅助进程。
AirSim 后端暂不支持 attach。详见根目录 `config/README.md`。
