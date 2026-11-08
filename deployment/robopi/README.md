# RoboPi AS2 运行

官方交叉编译 builder 的调用与挂载契约统一见
[oh-ros-build](../oh-ros-build/README.md)；本页维护板端部署、运行及工具链准备记录。

使用前须先完成[官方开发环境准备](../oh-ros-build/README.md#前置条件)，
再按[首次初始化步骤](../oh-ros-build/README.md#首次初始化项目工作区)执行
`prepare.sh`。下文操作均以此为前提。

本机操作统一从仓库根目录的 `scripts/robopi.sh` 进入。设备、网卡和运行参数
读取被忽略的 `.env`，公开模板见 [../../.env.example](../../.env.example)。

```text
PC: AirSim + PX4 1.18 / DDS Agent + Humble 传感器 / clock
                           ↕ 有线 DDS
RoboPi: OHOS Humble + AS2 platform / estimator / controller / Behavior
```

PC 不同时启动 onboard_computer，以免与板端产生重复的 AS2 节点。

## 持久部署布局

活动程序统一使用以下目录，后续部署替换对应 `install/`，不生成按时间累积的安装备份：

```text
/data/aerohub/
├── as2/
│   ├── install/              # AS2、PX4 消息及公共依赖
│   ├── board_env.sh
│   ├── config.yaml、fleet.yaml、run_as2.py
│   └── runtime/
└── lib/
    ├── catora/
    │   ├── install/          # CAT-ORA 及必要应用包
    │   ├── env.sh
    │   └── runtime/
    └── swarm-formation/
        ├── install/          # 六个原生规划包及必要应用包
        ├── env.sh            # 先加载 AS2，再加载本应用
        └── runtime/
```

`/data/aerohub/as2/board_env.sh` 只加载 AS2；应用加载
`/data/aerohub/lib/catora/env.sh`。未来算法使用 `/data/aerohub/lib/<应用名>/`，
各自携带必要应用包并复用 AS2；不要把算法包混进 AS2 安装。
Swarm 环境入口为 `/data/aerohub/lib/swarm-formation/env.sh`。

```bash
# 部署到新目录，保持当前 .env 与旧运行不变
bash scripts/deploy_lib.sh --component all
# 后续仅更新对应一侧
bash scripts/deploy_lib.sh --component as2
bash scripts/deploy_lib.sh --component catora
bash scripts/deploy_lib.sh --component swarm-formation
# 需要编译时同步对应算法与最小接口源码，复用官方 OHOS builder
bash scripts/build_lib.sh all
# 仅板端加载、启动和小型接口验证，不解锁、不飞行
bash scripts/verify_lib.sh all
```

部署先 staging 解包并核对哈希，再替换本次组件；临时旧安装只用于失败回退，
成功后删除。运行中的目标目录消费者会阻止替换。源码构建仍复用官方 builder；
打包器显式重定位逐包环境路径、Python shebang 和构建机 ELF 搜索路径，
检查 AArch64 与动态依赖不变，不直接搬移未处理的 install。

需要切换本机配置时在 AS2 部署命令追加 `--activate-config`，成功后才把 `.env`
中的 `ROBOPI_ROOT` 更新为 `/data/aerohub/as2`；`scripts/robopi.sh deploy` 包含该步骤。
默认配置已切换到 `/data/aerohub/as2`；历史路径 `/data/local/tmp/aerohub-as2`
中的进程已通过管理入口停止，历史文件暂留，不再作为活动安装。
下文早期验收记录中的 tmp 路径是历史位置。

## 持久布局简单验收（2026-11-05）

默认配置已激活新路径，AS2、CAT-ORA、Swarm-Formation 的板端简单验证均通过：

| 组件 | 检查 | 版本化记录 |
| --- | --- | --- |
| AS2 | `as2_core` 导入、PlatformInfo CDR 往返、加载默认配置后估计器存活 5 秒 | `validation/startup.json` |
| CAT-ORA | 应用模块导入、一次原生服务请求及 JSON 代理一致性，生成 16 点 | `../../src/planner/cat_ora/validation/robopi-smoke.json` |
| Swarm-Formation | 应用模块导入、单节点接收静态里程计/合成点云/手动目标，输出 2 段五阶 PolyTraj | `../../src/planner/swarm_formation/validation/robopi-smoke.json` |

本次使用官方 builder 交叉编译算法，复用已有 AS2 产物；原始构建、调试和最终
验收日志保存在本机 `artifacts/robopi-lib-acceptance/`。板端每个组件的
`runtime/verification/result.json` 保存最新结果，`deployment.json` 保存部署文件哈希。
验证进程、专用调试器和转发均已清理，程序保留安装；没有启动仿真或发送运动命令。
这次不覆盖真实点云、七机编队/代理完整输出或飞行闭环。

## 手动入口

```bash
# 首次填写配置；已有 .env 不要覆盖
cp .env.example .env

bash scripts/robopi.sh check       # HDC 与 IP 连通性
bash scripts/robopi.sh network     # 仅恢复板端临时 /24 地址，不修改 PC 网卡
bash scripts/robopi.sh sync-time   # 临时校准板端系统时钟到 PC，不改 RTC / 时区 / NTP 配置
bash scripts/robopi.sh build       # 同步当前 fork 与 1.18 消息，交叉编译 Runtime
bash scripts/robopi.sh deploy      # 校验传输、解压后切换私有 overlay，保留 install 备份
bash scripts/robopi.sh up          # PC 底座 + RoboPi AS2
bash scripts/robopi.sh status
bash scripts/robopi.sh verify      # 定位、时钟、传感器；不解锁
bash scripts/robopi.sh verify --flight  # 仿真：起飞、到点、悬停、降落、解除解锁
bash scripts/robopi.sh down        # 停止本入口拥有的进程与容器
```

`.env` 必须填写 HDC_BIN、HDC_TARGET、PC_IP、ROBOPI_IP、网卡、ROS_DOMAIN_ID
及板端私有目录。它是可信的本机 shell 配置，由脚本读取，不提交 Git。
PC 静态地址仍需本机管理员预先配置；两端不需要网关，PC 的 Wi-Fi 保留上网。
重启或重新插线可能丢失临时地址。`network` 仅操作 `.env` 指定的板端接口。
控制消息的传输时间戳要求两端系统时钟接近；`check` 会报告偏差，`up` 在偏差
超过 0.5 秒时拒绝启动。可显式执行 `sync-time` 临时校准，长期运行应配置
合适的时间同步服务；脚本不会自动修改系统时间。

`build` 复用下文已经准备的隔离 SDK、ROS 目标依赖及 RMW 修复工作区，不负责
从空机器自动创建这些资源；首次创建使用上面的 `prepare.sh` 入口。
OHOS 与 PC 产物隔离，部署前须停止板端 AS2。
当前 AS2 构建默认前缀为 `/data/aerohub/as2/install`；应用按上文独立布局打包。
已有产物需要经部署打包器重定位并校验，不能只改 `.env` 后搬运二进制。

## 当前组合与诊断

生成的 PC Compose 与两端 DDS 配置保存在 `artifacts/robopi-runtime/`；
DDS 只使用 `.env` 指定 IP 的接口，ROS 域默认 83，独立 Compose 项目为
`aerohub-robopi`。PC 仅使用固定 flight_board 镜像：vehicle 同时运行
PX4、Agent 和 Humble 传感器，clock 使用同一镜像单独运行全局时钟。
无需额外准备 Humble sensors 镜像，也不挂载 supervisor 源码。
首次使用按[固定镜像说明](../flight_board/README.md#使用固定镜像)加载新的
Humble 版本 tar；旧 Jazzy 版本不适用于此入口。

此前已复现 Jazzy 发布端导致板端 AS2 内存快速增长，独立 Humble sensors
曾作为过渡方案。当前已将 Humble 发布端合并到 flight_board，底层跨版本
中间件问题未宣称完成根因修复。

运行传感器和时钟发布端时，可检查 AS2 进程树内存：

```bash
source scripts/load_env.sh
python3 scripts/diagnostics/verify_sensor_memory.py \
  --duration 180 --output artifacts/sensor-memory.json
```

检查先预热 15 秒，随后允许最多 64 MiB 增长；进程退出、重启或超限均报错。
这项检查不启动或停止板端应用，结果只代表指定观察窗口。

板端启动器仅为自己的进程解除继承的 OOM 保护，默认监控进程树 RSS，超过
`AS2_MAX_RSS_MB=1536` 时停止本次 AS2，不修改系统级 OOM 或看门狗配置。
`up` 表示启动了进程，是否业务就绪以 `verify` 为准。

当前入口覆盖单机 AS2 跨机执行链；两库的简单加载与消息调用使用上文独立验证入口。
完整飞行使用独立应用：[CAT-ORA 四机及 PC/RoboPi 混合部署](../../demos/catora_open_field/README.md)、
[Swarm-Formation 三机真实点云混合部署](../../demos/swarm_open_field/README.md)。
两者都已完成指定场景板端执行验收，不扩展为七机、无线或长期稳定性结论。

需要用独立场景复验整合后的 flight_board 时，见[单机闭环验收入口](../flight_board/README.md#robopi-单机闭环验收)。

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

## 2026-11-05 本机跨机验收（原分离传感器方案）

当前 AS2 fork 与 PX4 1.18 消息已重编为 OHOS AArch64 产物并部署。
通过 PC AirSimNH/PX4 → RoboPi AS2 的定位、时钟、IMU、GPS、电池、
七路图像及 CameraInfo、LiDAR、静态 TF 检查；解锁、Offboard、起飞、
到点、悬停、降落及解除解锁全部通过。
到点误差约 0.154 米，两秒悬停漂移约 0.395 米，均低于 0.5 米验收阈值。
证据：`artifacts/robopi-runtime/interaction.json`、`sensors.json`；
本轮诊断已确认并处理发布端运行环境与系统时钟偏差两个问题。
AS2 进程树内存约 648 MiB，继续保留 1536 MiB 保护阈值。
这次通过不扩展为多机、长时间耐久或算法 Library 的验收。

## 首次工具链准备与历史经验

本轮日期：2026-11-04。使用官方 `build-ros-humble --custom` 流程。
这不是 PC Linux 安装目录复制上板，也未替换板端系统 ROS。

以下记录早期独立构建工作区的准备过程。当前活动源码已迁到 `thirdparty/` 的两个
fork，`uint` 可移植性修复已直接进入 `thirdparty/aerostack2/as2_core`，无需重复应用。
当前入口由 `scripts/prepare_ohos_sources.py` 同步该工作区的活动 AS2 源码与
PX4 1.18 消息；首次准备目标工具链仍参考下文。

## 实际环境

- Builder：`voxelsky/ohos-ros-humble-builder:v0.1.5`，本机 ID
  `sha256:3312006a50a2a7f4a26e1c92afcfbb0fd2296989b7e6d22e71b1f77caad0d25f`。
- SDK：`/path/to/ohos-sdk/native`，Clang 15.0.4，目标 AArch64 OHOS。
- 目标依赖快照：`/path/to/target/deps/{install,out}`。
- HDC：`/path/to/ohos-sdk/toolchains/hdc`。
- 设备：通过 `.env` 的 `HDC_TARGET` 指定；已验证系统为 OHOS Linux 6.6.86、aarch64。
- 系统 ROS：`/sys_prod/robot/{install,out}`，Python 3.12。
- 私有部署：`/data/local/tmp/aerohub-as2`；SDK/构建副本在仓库忽略目录
  `artifacts/ohos-as2`。

AS2 基线沿用仓库锁定源码，Pixhawk 本轮使用原 v1.17 补丁；没有宣称完成
PX4 1.18 的完整消息/控制适配。工作区其他同时开发的 v1.18 文件不属于本轮输入。

官方文档：
https://gitcode.com/openharmony-robot/docs/blob/main/device-dev/docker-build.md

官方 ros_ros2_base 本次取自 commit
`8f588ad3c244781cc1cfc7472cfe1cbaff6702c3`。
额外 vision_msgs 为 Humble commit `5f35e557cfb5e085ac5c6dc0210c5736c2c362f7`；
mocap4r2_msgs 为 `2bce86f38b7129d235d58ebd8977beadb0a83123`；PID 使用上游 v1.0。

## 隔离准备与发现的问题

官方脚本会执行 `ohla patch` 修改 SDK，必须用副本，不能直接修改公共 SDK。
本轮目录布局：

```text
artifacts/ohos-as2/
  root/
    ohos-robot-toolchain/18/native/  # SDK 副本
    ros_ros2_base/                 # 官方源码与工具链副本
    install/                      # 目标 ROS install 副本
  ws/src/                         # AS2、依赖和 RMW 修正源码
  ws/build/、ws/install/、ws/log/
```

准备使用 `cp -a --reflink=auto`，原始 SDK 和目标快照保持不变。
本机快照缺少 setup.bash，在副本中创建只设置以下路径的环境文件：

```bash
export AMENT_PREFIX_PATH=/work/root/install
export CMAKE_PREFIX_PATH=/work/root/install
export PYTHONPATH=/work/root/install/lib/python3.12/site-packages
```

快照中旧的 `/home/ipads/tmp/uav-lesson/artifacts/robopi-stage3/target/deps/out/`
及 `/target/out/` 引用在副本 CMake/pc/sh/dsv 文件中重定位到 `/target-out/`；
`/target/install/` 重定位到 `/work/root/install/`。
原始 out 仅只读挂载到 `/target-out`。

SDK 副本需从目标 out 补入 `libsframe.a`、Python 3.12 头文件和库、numpy 头文件。
官方工具链副本的 CMAKE_FIND_ROOT_PATH 加入 `/target-out`，避免误选 PC 库。
`Findyaml.cmake` 为目标 libyaml_vendor 的查找适配，参考 LeUAV 同名文件。

其他显式参数见 build_as2.sh：目标 OpenSSL、Eigen、pybind11；主机 Python
负责代码生成，PYTHON_SOABI 使用 `cpython-312-aarch64-linux-ohos`。
pybind11 使用快照已携带的 moveit_task_constructor_core 配置及头文件。
AS2 唯一源码修改为 as2-core-portability.patch 的 `uint` → `unsigned int`。

## 编译

准备完上述独立工作区后：

```bash
bash deployment/oh-ros-build/build_as2.sh as2_msgs
bash deployment/oh-ros-build/build_as2.sh as2_core
bash deployment/oh-ros-build/build_as2.sh as2_state_estimator
bash deployment/oh-ros-build/build_as2.sh as2_platform_pixhawk
bash deployment/oh-ros-build/build_as2.sh as2_motion_controller
bash deployment/oh-ros-build/build_as2.sh as2_behaviors_motion
```

需要的源包除 AS2 外还包括 vision_msgs、mocap4r2_msgs、px4_msgs。
PID 源码预取到 as2_motion_controller/thirdparties/pid_controller，以保持编译离线。
编译产物不可在 PC 执行；用 SDK llvm-readelf 审计 ELF，随后在实际板端测试。
custom-prefix 指定最终安装路径，不能将它随意改到其他路径后直接运行。

## 关键运行问题：RMW RTTI

最初 AS2 状态估计启动触发 `cast_or_create_topic:88` 断言。
独立标准 TF 探针复现：创建 listener 成功，再创建 broadcaster 立即断言。
简单 TFMessage/PlatformInfo 发布正常，改 UDP 不解决；CycloneDDS 对照还缺少
`mq_unlink` 符号，不作为可用替代。

官方当前 rmw_fastrtps_shared_cpp/src/utils.cpp 对 OHOS 已使用 static_cast，
避免原 dynamic_cast 在该环境的失败。将同一官方仓库中的
`src/ros2/rmw_fastrtps/rmw_fastrtps_shared_cpp` 复制到 ws/src 后：

```bash
bash deployment/oh-ros-build/build_as2.sh rmw_fastrtps_shared_cpp
```

仅将新库放在私有 overlay 并优先加载，系统 `/sys_prod/robot` 未更改。
重测后 TF 三项均通过，状态估计存活五秒并正常退出。

## 部署和板端测试

将 ws/install 打包发送到板端，在 `/data/local/tmp/aerohub-as2` 解压；
部署 board_env.sh 并 source。它加载系统 ROS 和逐包私有 overlay，
不依赖 PC 的 Python/共享库，也不复制整个系统依赖快照上板。

board_smoke.py、tf_probe.py、dds_probe.py 可在该目录运行。
full_smoke.py 需要同目录 run_as2.py（来自 onboard_computer/run.py，将临时目录
改到 `/data/local/tmp/aerohub-as2/runtime`）及 config/fleet/pid_speed_controller。
通过 AS2_CONFIG_FILE、AS2_FLEET_FILE、AS2_CONTROLLER_CONFIG_FILE 指向这些文件。
测试使用 ROS_DOMAIN_ID=83，与正常运行隔离。

## 已验证与边界

- 消息、核心、平台、估计器、控制器、行为均完成 OHOS 交叉编译。
- 板端 import as2_core 成功；PlatformInfo CDR 往返成功（39 bytes）。
- 修正私有 RMW 后标准 TF listener/broadcaster/static broadcaster 均成功。
- 板端完整启动发现 platform、state_estimator、controller_manager 和四类行为，
  launcher 退出码 0，测试进程已清理。
- 原始证据在 artifacts/ohos-as2 的 build、board-*.log 与 manifest.json。

尚未验证 PC AirSim/PX4 → 板端 AS2 定位和时钟接收、控制 ACK、起飞或任务。
板端只有 loopback 获得 IP，eth0 无地址；不能直接假设跨机 UDP DDS 可用。
后续需配置局域网或 HDC TCP；HDC 不能直接转发 DDS UDP。
本轮没有解锁、起飞或发送运动指令。
