# flight_board 镜像

复用 `px4io/px4-sitl-gazebo` 的飞控二进制，封装单机 PX4 SITL、Micro XRCE-DDS
Agent、Humble AirSim 传感器桥和全局时钟桥。镜像保留原 PX4 所链接的 Gazebo 库，
默认不启动 Gazebo。AirSim 场景独立运行。

## 使用固定镜像

日常启动直接使用 `flight_board:airsim-px4-humble-threaded-20261105`，Compose
不执行 build，也不自动拉取镜像。该版本为 Linux amd64，镜像 ID：
`sha256:8c33881fde6985fe5a43a894bde075471df85064e45ac8e5de28efebcc737cd2`。
PX4、DDS Agent、传感器桥、supervisor 和默认 DDS 配置均已包含在镜像内。
AirSim 场景与车辆配置仍由宿主机提供；RoboPi 的 DDS 网络配置仍按运行环境挂载。

本机校验及离线打包（默认输出到 `artifacts/images/`，不进入 Git）：

```bash
bash deployment/flight_board/image.sh check
bash deployment/flight_board/image.sh export
```

将生成的 `.tar` 和 `.tar.sha256` 一起复制到另一台 amd64 机器，再执行：

```bash
bash deployment/flight_board/image.sh load /path/to/flight_board-humble-threaded-20261105-amd64.tar
docker compose -f deployment/flight_board/compose.yaml up -d
```

加载入口校验归档文件的 SHA256 及加载后的镜像 ID。无需源码编译或下载
基础镜像。当前交付使用离线包，尚未推送镜像仓库。

## 采集线程与相机开关

正式采集始终采用每个物理传感器一个独立线程，不提供串行采集模式。
同一相机的 RGB/深度合并 RPC；ROS 定时器只投递任务，慢采集最多保留一个待处理请求。
线程、相机开关与退出回收由镜像构建中的 12 项测试验证。

默认开启相机。仅需要 LiDAR 时，Compose 可设置 `FLIGHT_BOARD_CAMERAS=0`；
传感器节点的对应参数是 `camera_enabled:=false`。这只关闭 ROS 侧相机采集，
不修改 AirSim settings 中的相机模型。

已有旧固定镜像时，可以只升级传感器层，不重建 PX4/Agent，也不访问网络：

```bash
docker build --network none --pull=false -f deployment/flight_board/Dockerfile.sensors \
  -t flight_board:airsim-px4-humble-threaded-20261105 .
```

自建镜像 ID 不保证与本机交付 ID 一致；`image.sh check/load` 校验的是本次固定交付。
原 43632de22490 镜像保留作为来源和历史飞行验收基线，不能将旧镜像的飞行验收
自动视为新镜像的完整飞行验收。

## 镜像组成与来源

交付和使用均通过 tar 文件进行。[Dockerfile](Dockerfile) 保留为来源记录，
用于查看从 `px4io/px4-sitl-gazebo` 基础镜像开始添加的内容：

| 增加项 | 作用 |
| --- | --- |
| Micro XRCE-DDS Agent 2.4.2 及固定版本依赖 | 将 PX4 接入 DDS，独立安装在 `/opt/xrce` |
| ROS 2 Humble、标准传感器消息与 TF 支持 | 为镜像内传感器和时钟桥提供运行环境 |
| AirSim 传感器桥 | 发布图像、CameraInfo、LiDAR 与静态 TF |
| AirSim 时钟桥 | 发布全局 `/clock` |
| supervisor 与容器入口 | 配置和管理 PX4、Agent、传感器进程，汇总日志及健康检查 |
| DDS UDPv4 配置与 AirSim 配置示例 | 提供默认通信配置与车辆配置参考 |

PX4 固件沿用基础镜像，不重新编译；仅调整 ELF 解释器和 RPATH 以隔离运行库。
启动器按车辆配置调整容器内启动脚本副本。
默认不启动 Gazebo，保留 PX4 所链接的 Gazebo 运行库。

默认基础镜像锁定为 digest：
`sha256:54a7a15ba35b3f4786256296d89edd144e000f40084cb6ed692fcbf74c9168f2`。
固件提交为 `61907962c9220437ec01a98fa26bfbf5cbb254b5`，报告版本 `1.18.0 80`，
属于开发快照，不是正式稳定版。本仓库继续使用已验收的该固定基线；
companion AS2 使用 `companion_as2_dependencies:humble-px4-1.18` 配套消息。
不需要额外的 1.17/1.18 消息翻译节点。
最终运行环境为 Ubuntu 22.04 / ROS 2 Humble，与 RoboPi 的 ROS 发行版一致。
PX4 及 Agent 保留 Noble 构建产物；其动态加载器和系统库放在
`/opt/px4-runtime/lib`，由各 ELF 的解释器和 RPATH 单独加载，不覆盖 Humble
系统库，不向 ROS 进程注入 Noble 库。Dockerfile 保留完整来源与依赖收集步骤。

[RoboPi 入口](../robopi/README.md)直接使用同一 flight_board 镜像：vehicle
运行 PX4、Agent、传感器，另一个 clock 容器运行全局时钟；无需独立的传感器
镜像。`FLIGHT_BOARD_SENSORS=0` 仅用于诊断时关闭传感器，默认值为 1。

Agent 2.4.2 及其依赖的重建输入为 `vendor/` 中的固定版本源码归档，
文件名和 SHA256 保留在 Dockerfile。归档不随源码交付；日常加载 tar 无需这些文件。
如需重建，须自行准备对应归档后再执行 Dockerfile。
传感器接口见 [接口说明](../../runtime/airsim_sensors/README.md)。

## 运行模式

管理输出保持简洁，供 Codex 或命令行检查：

```bash
docker exec <container> python3 /opt/flight_board/supervisor.py status
docker logs <container>
```

status 输出 JSON：mode、各进程 alive/exit_code、healthy。仅表示必要进程
存活，不表示飞行就绪；HEALTHCHECK 复用同一检查。未观察到的退出码为 null。
日志统一输出标准输出，使用 [px4]、[agent]、[sensors]、[clock] 前缀。
进程身份校验参考 LeUAV 的 supervisor（PID + /proc 启动时间）。

DDS 配置复用 LeUAV 的显式 UDPv4 profile，位于
`config/dds/fastdds_udp.xml`。flight_board 和 companion_as2 镜像均加载
该文件，Compose 不再共享 host IPC；AS2 需使用更新后的配套镜像。
Agent 仍使用镜像构建阶段编译的产物，无 builder 镜像依赖。

- `vehicle`（默认）：一架飞机的 PX4 + Agent + 传感器桥。任一子进程退出，
  关闭其余子进程并让容器报错退出；停止容器会清理全部子进程。
- `clock`：只运行全局 AirSim → `/clock` 桥。同一仿真世界只启动一份。
- `shell`：进入已加载 ROS 环境的 Bash，适合诊断。

```bash
# 先用 config/airsim/settings/px4-sensors.json 启动 AirSim，再执行：
docker compose -f deployment/flight_board/compose.yaml up -d
docker compose -f deployment/flight_board/compose.yaml logs -f
docker compose -f deployment/flight_board/compose.yaml down
```

示例使用已验证的单机场景端口：TCP 4580，AirSim 控制 UDP 14640/14680，
RPC 41471，DDS Agent 8891，ROS 域 81。容器内 PX4 实例编号 20、系统 ID 21，
但对外统一 namespace 为 `drone0`。这些不是固定要求。

| 环境变量 | 默认值 / 用途 |
| --- | --- |
| AIRSIM_SETTINGS | `/config/settings.json`，需挂载；用于车辆和传感器配置 |
| VEHICLE_NAME | `PX4`，对应 AirSim Vehicles 中的键 |
| VEHICLE_NAMESPACE | `drone0`，ROS topic 和 TF 前缀 |
| PX4_INSTANCE | `0`，PX4 进程实例；host 网络多机时应唯一，避免其它 MAVLink 端口冲突 |
| PX4_SYSTEM_ID | `1`，PX4 系统 ID 和 DDS key；逐机唯一 |
| XRCE_AGENT_PORT | `8888`，逐机 Agent 端口；host 网络时逐机唯一 |
| AIRSIM_HOST | `127.0.0.1`，仿真 TCP 与传感器 RPC 地址 |
| AIRSIM_RPC_PORT | `41471`，场景共享 RPC 端口 |
| ROS_DOMAIN_ID | `81`，与 AS2、App 一致 |

PX4 仿真 TCP 端口及控制 UDP 端口从该车辆的 AirSim 配置读取。启动时复制
PX4 启动脚本到容器私有目录后修改，不改原镜像或宿主机源码。
若 AirSim 不在本机，还需要在 AirSim settings 中配置正确的监听和控制地址；
仅修改 AIRSIM_HOST 不会修改远端场景配置。

同一镜像可复制多个 vehicle 服务，各自指定车辆名、namespace、系统 ID、
实例编号和 Agent 端口；共享一份多机 AirSim settings 和一个 clock 服务。
当前示例只配置一架飞机，不是七机验收配置。

## 时间和控制边界

clock 模式从 `getMultirotorState.timestamp` 读取 AirSim ClockFactory 时间，
不依赖相机或 LiDAR 帧率。桥自身使用实际时间轮询；传感器节点使用 ROS 仿真
时间调度，数据保留 AirSim 采样时间戳。AS2/App 需要启用 `use_sim_time=true`。
AirSim 时间倒退时会发布新时间并告警，消费者仍需处理任务和估计器重置。
暂停/变速、重启和 PX4 时间转换需要在完整控制闭环中进一步验收。

本镜像不会自动解锁、起飞或启动规划任务。AS2 控制组件和 App 不包含在此镜像。

## RoboPi 单机闭环验收

已部署且正在运行板端 AS2、有线网络与 `.env` 配置完成后，可用专用场景验收。
先停止其他占用 AirSim/PX4 单机场景端口的 PC 实例；本入口不部署、不重启板端
AS2，也不依赖 companion_as2 镜像。默认仅被动检查：

```bash
bash deployment/flight_board/verify_robopi.sh
```

显式启用仿真飞行并观察 10 分钟内存：

```bash
VERIFY_AS2_FLIGHT=1 VERIFY_MEMORY_SECONDS=600 \
  bash deployment/flight_board/verify_robopi.sh
```

飞行流程包括解锁、Offboard、1.5 米起飞、约 1 米水平移动、2 秒悬停、降落和
解除解锁。检查到点误差与悬停漂移均不超过 0.5 米。脚本记录实际反馈、传感器、
内存、进程健康和镜像 ID；结束后清理其创建的 PC 场景与容器，保留既有板端 AS2。
默认记录在 `artifacts/flight-board-robopi-verification/`，可用 `VERIFY_OUTPUT` 更换。

## 可选 PC 开发验收

以下仅用于 PC 开发检查，不是正式交付的前置要求。正式 RoboPi 验收使用上面的
`verify_robopi.sh`，无需准备 companion 镜像。

```bash
bash deployment/flight_board/verify.sh
```

验收前先加载固定 flight_board tar，并按[场景资源说明](../../config/airsim/README.md)
配置 `airsim_nh`。验收使用 `config/airsim/nh.json`，也支持 `AIRSIM_BINARY`
覆盖可执行文件位置；不会读取开发者的固定场景目录。

默认订阅端使用 `companion_as2_dependencies:humble-px4-1.18`，包含 Humble
标准消息与配套 PX4 消息。需提前加载该镜像的 tar，或按
[固定消息依赖说明](../companion_as2/px4_msgs/README.md)准备。
已有包含相同接口的镜像可用 `VERIFY_ROS_IMAGE=<image>` 指定。
该依赖仅供 PC 独立验收，不是 RoboPi 运行链路的额外要求。

脚本先检查所需镜像是否已存在，再启动独立 AirSimNH 场景、逐机容器和 clock
容器，订阅传感器和时钟，并检查 PX4 DDS 连接。不会解锁或发送运动指令。
`VERIFY_AS2=1` 时改用 `companion_as2:airsim-px4` 执行 AS2 交互验收。
结果保存在 `artifacts/flight-board-verification`，退出时清理验证资源。
传感器/时钟接收与完整 AS2 飞行控制链路是两项独立验收。

## Humble 单镜像完整验收（2026-11-05）

固定镜像 `flight_board:airsim-px4-humble-20261105-43632de22490` 已通过
PC AirSim/PX4 → RoboPi AS2 单机仿真闭环：解锁、Offboard、1.5 米起飞、
到点、悬停、降落、解除解锁。到点误差 0.148 米，
两秒悬停漂移 0.408 米，均低于 0.5 米阈值。

传感器、静态 TF、时钟与定位在飞行后及观察接近结束时均无缺项。
AS2 进程未重启，观察 600.5 秒，进程树 RSS
651.2–651.7 MiB，预热后最大增长 0.4 MiB，
低于预设 64 MiB 阈值；未出现此前的持续快速增长。十分钟观察不等同于长期
耐久或多机验收。

入口：`deployment/flight_board/verify_robopi.sh`，本次设置
`VERIFY_AS2_FLIGHT=1 VERIFY_MEMORY_SECONDS=600`。证据位于
`artifacts/flight-board-humble-flight-acceptance/`，含飞行反馈、前后数据接收、
内存样本、进程健康、镜像 ID、日志与场景记录。

## 此前的被动检查（2026-11-05）

当前固定 flight_board 已整合 Humble 传感器与 clock。PC 被动验收通过；
RoboPi 接收定位、时钟、IMU、GPS、电池、7 路图像和 CameraInfo、LiDAR 与静态 TF。
现有 AS2 进程未重启，观察 180.5 秒，进程树 RSS 为
651.2–651.3 MiB，预热后最大增长 0.0 MiB。
记录在 `artifacts/flight-board-humble-robopi/`；PC 被动验收记录在
`artifacts/flight-board-humble-verification-final/`。未发送运动命令，未重新验收飞行，
该观察窗口不代表长期耐久或多机负载验证。

## 历史验证记录

以下为整合 Humble 前的历史记录，镜像 ID 和跨版本诊断不代表当前版本。

2026-11-05：更新后的独立被动验收入口通过实测，使用资源配置解析 AirSimNH、
固定 flight_board 和有仓库配方的消息依赖镜像；传感器、静态 TF、`/clock`、
PX4 里程计及 DDS 连接均通过，容器健康检查通过。记录位于
`artifacts/flight-board-portable-verification-2/`。本轮未发送飞行命令。

2026-11-04 本机已成功构建 `flight_board:airsim-px4`，并通过 AirSimNH
实测：逐机 PX4/Agent 连接、7 路图像、7 路 CameraInfo、非空 LiDAR、
6 个静态 TF、全局 `/clock` 和 PX4 `vehicle_odometry` 均可由独立 Humble
容器接收并解码。验证进程与容器已清理。
镜像 ID：`sha256:35cfc21f5f3e0d7d684a1676b0687e3a52a4e0ad924d5bbb9311da034d705d84`。
构建期 7 项数据转换测试与 ROS 节点入口导入检查通过。

Humble/Fast DDS 在发现 Jazzy/PX4 topic 时仍可能输出
`sequence size exceeds remaining buffer` 诊断；本轮传感器与时钟消息接收成功，
该早期结果不代表 AS2 所需的全部 PX4 消息兼容。
随后已通过 [配套 AS2 的单机完整控制与传感器验收](../companion_as2/README.md)，
包含解锁、Offboard、起飞、到点、悬停、降落、解除解锁；多机负载和暂停/变速仍待验收。
