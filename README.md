# Aerostack2 统一 Planner 集成

本项目是**基于开源 Aerostack2 的独立集成与扩展工程**，复用其飞行执行框架，并针对所选 PX4 版本应用兼容补丁。Aerostack2 原有架构、组件及上游代码归各自版权持有人所有；本项目不是 Aerostack2 官方发行版，也不表示获得上游作者的认可或背书。

本工程把 CAT-ORA 和 Swarm-Formation 注册为同一类 `planner` 模块。任务配置每次选择一个 Planner，两者共用任务校验、项目内的 Planner 生命周期、`TimedTrajectorySegment`、Output Gateway、Aerostack2 和 PX4/Gazebo 执行链路。

任务生命周期由 `/planner/control` 管理；默认执行后端通过逐机 `FollowTimedPath` Actions 调用扩展的原生 `FollowPathBehavior`。扩展保留原生 FollowPath Action 和插件，新接口负责外部时间化轨迹；规划层不再直接发布运动参考。`execution_backend:=legacy_setpoint` 仅保留旧链路用于回归对照。

第一阶段不包含 Planner 并行协作、在线热切换、自动选优、远程 Model Hub、真机或国产操作系统验收。

## 当前 AirSim / RoboPi 入口

当前部署按四个运行角色组织；完整职责见[项目简介](docs/项目简介.md#deployment-roles)。

| 角色 | 当前对应 |
| --- | --- |
| 仿真器 | PC 宿主 AirSim，提供环境、动力学和仿真真值 |
| 飞控板 | `flight_board` 容器中的 PX4 SITL；同时包装传感器/时钟桥 |
| 算力板 | RoboPi 原生运行算法、导航输入、Adapter、AS2 与 FollowTimedPath |
| 工作站 | 固定 ROS 桌面镜像中的 `mission.py`，通过 ROS 调用已就绪的算力板能力 |

从[部署准备](deployment/README.md)进入，再选择 [Library Demo](deployment/airsim/demos/README.md)。
SUPER 四方运行使用显式 `--execution-location four-part`，构建、地面检查、短程、原任务和停止命令见
[SUPER Demo](deployment/airsim/demos/super_forest/README.md#four-part)。默认 `pc` 行为保持兼容。
四方链路的地面及短程验证通过，原 forest8 在途中更新被拒，详见
[Validation](deployment/airsim/demos/super_forest/VALIDATION-four-part-20261109.md)；单次失败原因尚未确认。

共享镜像作为稳定工具，日常源码修改通过工作区补编译和加载 overlay 生效。
SUPER 工作站已实现此方式；通用 onboard 开发入口的现状见
[开发约定](deployment/onboard_computer/README.md#固定镜像与源码开发用户确认的目标)。
下文 PX4/Gazebo、多机 ROS 1 镜像与 Compose 命令保留为早期集成路径；当前 AirSim/RoboPi 从上述入口开始。

## PR 功能与目录迁移

AirSim / RoboPi 扩展与主线 Gazebo、FAST-LIVO2 入口并存：

- 自建算法和适配器：`src/planner/`；基础执行：`src/behavior/`；导航输入整理：`src/perception/navigation_inputs/`。
- AirSim 场景配置、模拟器启动与管理、传感器桥：`simulator/airsim/`；跨组件演示和 AS2 启动入口：`deployment/airsim/`；板端管理工具：`tools/robopi/`。
- 主线 AS2 依赖仍在 `as2/aerostack2/` 和 `as2/as2_platform_pixhawk/`；Pixhawk 的 PX4 1.18 适配已转为构建补丁；AS2 核心也已转为 `as2-airsim-pr.patch`，只在 AirSim/OHOS 构建副本应用。
- SUPER 实际源码与 CAT-ORA 一致：`src/planner/super/ros2_adapter/native/`，AS2 适配在 `bridge/`，`upstream/` 子模块只作参考。运行产物进入被忽略的 `log/airsim/`。

已有 Skill 集中在 `skill/`；历史运行证据保留在忽略的 `log/pr-history/`，复现过程记录在 `skill/temp/`。`docs/` 只跟踪三份主文档及其图片。原 PR 锁定的 AS2 `f387e0d` 已取得并转为构建补丁，官方子模块保持 main 锁定版本；这仍是构建时源码修改，不是无侵入插件替代。

主线依赖准备入口仍为 `bash deployment/prepare_sources.sh`；AirSim / SUPER 使用 `bash tools/robopi/prepare_sources.sh`。
两者都需从仓库根目录执行后续构建命令。AirSim 构建还依赖 PR 指定的基础镜像与外部仿真资源；目录迁移不表示它们已在当前机器上完成闭环验收。
迁移说明与验证边界见[项目简介](docs/项目简介.md#pr-目录适配)。

## 上游来源、修改范围与许可

本项目依赖并致谢以下上游项目：

| 上游 | 本工程中的用途 | 锁定版本 |
| --- | --- | --- |
| [Aerostack2](https://github.com/aerostack2/aerostack2) | 状态估计、运动控制、运动 Behavior 及公共接口 | `19c397309c6a41b279e6b551818ec12605cfcecc` |
| [as2_platform_pixhawk](https://github.com/aerostack2/as2_platform_pixhawk) | Aerostack2 与 PX4 的平台适配 | `fcba062219935acef69dfa009c8be376c707ee8a` |
| [px4_msgs](https://github.com/PX4/px4_msgs) | PX4 的 ROS 2 消息定义 | `86d8239e962f6939e05c3737784f60c02fa884db` |

Aerostack2 由 Universidad Politécnica de Madrid（UPM）的 Computer Vision and Aerial Robotics Group（CVAR）及其他贡献者开发。官方资料见 [Aerostack2 文档](https://aerostack2.github.io/)和[项目论文](https://arxiv.org/abs/2303.18237)。源码来源与版本由 [upstream.repos](deployment/upstream.repos) 和 [prepare_sources.sh](deployment/prepare_sources.sh)记录。

本工程新增的集成层主要包括统一 Planner 消息与服务、本地 Registry、任务配置解析、项目级生命周期、Output Gateway、两个算法 adapter、AS2 setpoint adapter，以及多机部署与验收脚本。CAT-ORA、Swarm-Formation 的算法核心，以及 PX4、Gazebo 和通信依赖，均来自相应上游，不作为本项目原创成果声明。相关算法版本与复现来源见[统一 Planner 主文档](docs/项目简介.md#part-2)。

上游兼容修改集中保存在 [deployment/platform/patches/](deployment/platform/patches)：

- `as2-platform-px4-v1.17.patch`：适配 PX4 v1.17 的姿态、GPS、电池等消息字段。
- `microxrce-fastdds-v2.12.2.patch`：适配 Micro XRCE-DDS Agent 与 Fast DDS 版本。

这些补丁由构建流程应用；其存在不改变原代码的版权归属和许可证。

**许可按组件分别适用。** 使用、修改或分发上游源码及构建产物时，应保留各组件要求的版权声明、许可证条件、免责声明及适用的 NOTICE。对于采用 BSD-3-Clause 的上游文件与组件，源码分发须保留相应声明，二进制分发须在附带文档或材料中重现要求的声明；未经许可不得用版权持有人或贡献者的名称为衍生产品背书。具体以所分发版本附带的许可证和文件头为准，不应将整个依赖集合一概视为同一许可证。

当前自建 ROS 2 包的 `package.xml` 声明为 Apache-2.0，但仓库根目录尚未提供完整的项目 LICENSE 文本；该声明不能覆盖第三方代码，也不代表整个交付物已完成许可证核查。历史审计还记录了 CAT-ORA 不同位置的许可声明差异，公开发布源码、镜像或二进制前需核实实际纳入版本的授权条件。

本节用于明确来源、贡献边界与许可责任；仅添加署名说明不能替代许可证要求，也不能单独保证不存在侵权。

## docs 文档目录

| 文档 | 内容 |
| --- | --- |
| [AS2 简介](docs/as2简介.md) | 原生架构、执行流程及层间调用接口 |
| [项目简介](docs/项目简介.md) | 项目目录、统一 Planner 设计、配置、使用与验收 |
| [开发者贡献指导](docs/开发者贡献指导.md) | AS2 扩展点、Model Hub 适配设计与开发约束 |

## 目录

- `src/planner/interfaces`：统一 ROS 2 消息与 `ControlPlanner` 服务
- `src/planner/framework`：Registry、配置解析、生命周期、完成判定和 Output Gateway
- `src/behavior/follow_path`：兼容原生 FollowPath 的时间化轨迹扩展、ROS 回归与 SITL 验收入口
- `src/planner/cat_ora/ros2_adapter/bridge`：CAT-ORA ROS 1 服务适配
- `src/planner/swarm_formation/ros2_adapter/bridge`：Swarm-Formation ROS 1 滚动轨迹适配
- `src/behavior/legacy_setpoint`：PX4 本地 NED setpoint 到 AS2 本地 ENU reference
- `deployment/integration/compose.yaml`：完整七机仿真部署
- `src/planner/framework/examples`：任务与机群部署配置
- `log/resolved`：运行时生成的 resolved config 和进程日志

## 前置镜像

完整 Compose 使用以下本地镜像：

```text
uav/aerostack2-px4:humble-v1.17
uav/px4-gazebo:v1.17.0-harmonic
uav/catora:noetic-upstream
uav/swarm-formation:noetic-px4
```

全新环境可以用一个入口准备源码并构建平台、runtime 和两个 Planner 集成镜像：

```bash
bash ./deployment/build_from_zero.sh
```

平台镜像无缓存构建使用 `NO_CACHE=1 bash ./deployment/build_from_zero.sh`；集成镜像无缓存构建使用 `docker compose -f deployment/integration/compose.yaml build --no-cache`。

PX4/Gazebo 和两个 ROS 1 算法基础镜像分别来自相邻的 `catora-px4-gazebo` 与 `swarm-formation-px4-gazebo` 复现工程。对应 README 记录了构建步骤。

## 从零准备上游源码

平台镜像使用三个锁定版本的上游仓库。若只需准备源码，执行：

```bash
cd aerostack2-integration
bash ./deployment/prepare_sources.sh
```

脚本使用 `git submodule update --init --recursive` 初始化所有子模块；其中 AS2 构建使用以下锁定版本：

```text
aerostack2             19c397309c6a41b279e6b551818ec12605cfcecc
as2_platform_pixhawk   fcba062219935acef69dfa009c8be376c707ee8a
px4_msgs               86d8239e962f6939e05c3737784f60c02fa884db
```

子模块按外层仓库记录的 commit 检出；执行前应保留子模块中的本地修改，不使用强制覆盖选项。需要代理时，在运行脚本前设置 `HTTP_PROXY`、`HTTPS_PROXY` 和 Git 的代理配置。

## 构建与启动

在 Linux 仿真机执行：

```bash
cd aerostack2-integration
docker compose -f deployment/integration/compose.yaml build
docker compose -f deployment/integration/compose.yaml up -d --no-build
docker compose -f deployment/integration/compose.yaml ps
```

应看到以下六个服务处于 `Up`：

```text
planner-as2-seven
planner-px4-seven
planner-clock
planner-runtime
planner-cat-ora
planner-swarm-formation
```

Compose 使用 host network、`ROS_DOMAIN_ID=80` 和 Fast DDS UDP。Gazebo 原生时钟经 `planner-clock` 发布为 ROS 2 `/clock`，AS2 与 Planner runtime 均使用仿真时间。

已有镜像时，可以用一条命令完成干净启动、七机顺序起飞、Planner 启动和终态验收：

```bash
bash ./deployment/integration/run_planner.sh cat_ora
bash ./deployment/integration/run_planner.sh swarm_formation
```

脚本每次会先停止本工程的旧容器，因此两个命令应串行执行。失败时会自动汇总六个容器的末尾日志。

## 七机起飞

等待 PX4、AS2 和定位 topic 就绪后执行：

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  exec fleet_takeoff.py \
    --vehicles drone0 drone1 drone2 drone3 drone4 drone5 drone6 \
    --height 1.5 --speed 0.5 --timeout 90
'
```

脚本等待全部定位后，按机顺序执行 Arm、Offboard 和 Takeoff。七机高负载下不要并发发送 Takeoff action。

## 运行 CAT-ORA

CAT-ORA 示例使用 `drone0..drone3`。其余已起飞车辆继续由各自 AS2 控制器保持位置。

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  ros2 service call /planner/control \
    uav_planner_interfaces/srv/ControlPlanner \
    "{command: 0, task_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/cat_ora_four_px4.yaml, deployment_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/deployment_four_px4.yaml}"
'
```

CAT-ORA 会优化目标分配，但公共任务契约采用固定逐机目标。adapter 会核对每条轨迹终点；若算法改变车辆与目标的对应关系，任务以 `FAILED/NO_SOLUTION` 结束，不会执行换配后的轨迹。

## 运行 Swarm-Formation

验收时先重置仿真并重新执行七机起飞，再启动七机任务：

```bash
docker compose -f deployment/integration/compose.yaml down
docker compose -f deployment/integration/compose.yaml up -d --no-build
```

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  ros2 service call /planner/control \
    uav_planner_interfaces/srv/ControlPlanner \
    "{command: 0, task_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/swarm_formation_seven_px4.yaml, deployment_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/deployment_seven_px4.yaml}"
'
```

Swarm adapter 会把滚动 `PolyTraj` 转成短时 `TimedTrajectorySegment`，并在发布前校验机群完整性和轨迹首点连续性。

当前 ROS 地图生成器使用 `obs_num=0`。这套验收覆盖真实 PX4/Gazebo 动力学和队形闭环，不代表虚拟点云障碍物已经与 Gazebo 碰撞体对齐。

## 状态与控制

查询当前任务：

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  ros2 service call /planner/control \
    uav_planner_interfaces/srv/ControlPlanner "{command: 4}"
'
```

命令值为 `START=0`、`PAUSE=1`、`RESUME=2`、`CANCEL=3`、`GET_STATUS=4`。状态值为 `REJECTED=0`、`RUNNING=1`、`PAUSED=2`、`SUCCEEDED=3`、`FAILED=4`、`CANCELED=5`。

暂停、取消、Planner 失败、轨迹超时和任务超时时，框架会丢弃旧轨迹并发布当前实际位置的机群悬停参考。失败后不会自动降落或解除武装。

## 测试

`deployment/runtime/Dockerfile` 在镜像构建阶段编译全部五个 ROS 2 包，并运行：

- Planner framework：22 项
- CAT-ORA adapter：3 项
- Swarm-Formation adapter：5 项
- AS2 setpoint adapter：2 项

单独运行依赖无关测试：

```bash
python3 -m unittest discover -s src/planner/framework/tests -v
```

设计与验收边界见 [接口与配置设计](docs/项目简介.md#part-2)，最终构建和冷启动证据见 [最终验证](log/resolved/FINAL_VERIFICATION.md)。

完整操作流程、任务配置说明和排障方法见 [使用指南](docs/项目简介.md#part-2)。三份主文档见本文的 [docs 文档目录](#docs-文档目录)。

## 重组后的源码与产物

目录说明见 [项目目录说明](docs/项目简介.md#part-1)。克隆时使用 `git clone --recurse-submodules`。AS2 子模块位于 `as2/`，PX4 位于 `px4/`，官方仿真工程位于 `simulator/gazebo/`。算法源码位于 `src/planner/cat_ora/upstream/` 和 `src/planner/swarm_formation/upstream/`，算法镜像构建会从这里重新编译。

PX4 启动使用本仓库 `px4/`，构建目录挂载 `log/px4-build`，仿真缓存为 `log/px4-sim-cache`，运行日志为 `log/px4-runtime`。新环境需先准备 PX4 SITL 构建产物；本轮迁移保留已有相同 commit 的构建产物，不代表重新编译了 PX4。
