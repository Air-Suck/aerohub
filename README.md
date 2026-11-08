# AeroHub

围绕 AS2 Runtime、算法 Library 与用户任务组织的无人机开发仓库。
当前已通过 PC AirSim/PX4 1.18 → RoboPi AS2 的单机仿真闭环。

## 交付与运行入口

**OHOS / RoboPi 使用前提：先完成官方[设备开发环境搭建和开发指南](https://gitcode.com/openharmony-robot/docs/blob/main/device-dev/%E8%AE%BE%E5%A4%87%E5%BC%80%E5%8F%91%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA%E5%92%8C%E5%BC%80%E5%8F%91%E6%8C%87%E5%8D%97.md)，再执行本仓库的项目适配、编译和部署步骤。**

交付包含 `flight_board` 和 `onboard_computer` 两个镜像。用户已有 RoboPi，
开发流程以 RoboPi 为最终运行与验收目标；是否使用 `onboard_computer` 做 PC 开发可自行选择。
两份镜像当前定版为 **v0.1.0**，准确镜像 ID 与验收范围见[发布清单](deployment/releases/README.md)。
官方 builder 从官方指南所指网站获取，SDK 由用户自行检查准备。

## 除源码外，需要准备什么

| 资源 | 获取与检查 | 是否必须 |
| --- | --- | --- |
| Linux amd64 PC | 安装 Git、Docker Engine / Compose、Python 3 和 PyYAML；准备可运行 AirSim 的 GPU、驱动及图形环境 | 必须，承载仿真与交叉编译 |
| `flight_board` 镜像 | 从交付包加载固定镜像，按[镜像说明](deployment/flight_board/README.md#使用固定镜像)校验 | 必须，提供 PX4 / DDS Agent / 传感器 / clock |
| `onboard_computer` 镜像 | 从交付包加载；用于 PC 上的 AS2 / Library 开发及部分混合 Demo，见[开发环境](deployment/onboard_computer/README.md) | PC 开发可选；具体 Demo 依赖以其 README 为准 |
| 官方 OHOS builder | 按[官方指南](deployment/oh-ros-build/README.md#前置条件)从网站获取；本仓库直接调用，不自行构建派生 builder | 必须，用于交叉编译 |
| OHOS SDK 与目标依赖 | 自行确认已有匹配 SDK、目标 ROS Humble install 和第三方依赖 out；核对 AArch64 OHOS / Python 3.12，按[准备说明](deployment/oh-ros-build/README.md#首次初始化项目工作区)检查与初始化 | 必须；只下载 builder 不足以完成编译 |
| RoboPi 与 HDC | 确认板端系统及 ROS 环境匹配，PC 可通过 HDC 访问；配置设备 ID 与 HDC 路径 | 必须，Library 必须完成板端验证 |
| PC ↔ RoboPi 网络 | 准备有线 DDS 通信、可达 IP、对应网卡及时间同步；填写 `.env` | 必须，见[RoboPi 说明](deployment/robopi/README.md) |
| AirSim 场景 | 自行到 [AirSim 仓库 Releases](https://github.com/microsoft/AirSim/releases) 下载对应 Linux 场景；当前 NH Demo 需要 AirSimNH，解压后按[资源说明](config/airsim/README.md)填写路径 | 必须，场景不随源码或镜像交付 |

选择与 Demo 匹配的场景，下载其他场景或版本不等于该 Demo 已验证。
SDK、场景及个人设备路径不提交到仓库。

## 首次使用与开发顺序

1. 准备上表资源，加载交付的 `flight_board`；需要 PC 开发或混合 Demo 时再加载 `onboard_computer`。
2. 初始化 submodule，按模板配置 `.env` 与 `config/resources.local.yaml`。
3. 按[OHOS 工作区准备说明](deployment/oh-ros-build/README.md#首次初始化项目工作区)确认 SDK / 目标依赖、准备隔离工作区，并执行 `bash deployment/oh-ros-build/run.sh check`。
4. 按[RoboPi 说明](deployment/robopi/README.md)构建、部署并验证 AS2 基础链路。
5. 选择 [Demo](demos/README.md)，迁移或修改 Library，完成对应的板端接口验证和端到端验收，保留结果与实际源码版本。

**RoboPi 验证是 Library 接入的必经步骤。** PC 构建、单元测试及 PC 仿真可用于开发，
不能代替板端验收。板端导入或启动检查也不能代替 Demo 的端到端结果。
SUPER 已新增独立 PC 镜像与 FollowTimedPath 场景入口；板端统一部署仍缺失，PC 结果不能代替 RoboPi 验收。

## 算法迁移分析

迁移新算法时，使用项目技能 [$algorithm-migration](.agents/skills/algorithm-migration/SKILL.md)，提供算法源码位置、需要保留的能力和目标运行环境。例如：

```text
使用 $algorithm-migration 分析 <算法源码路径> 如何接入本项目，
保留 <所需能力>，目标为 PC onboard_computer 和 RoboPi。
```

技能先核查源码和当前 AS2 接口，交付迁移流程表、分层架构图、各层新增/复用/修改清单、依赖及接口契约，并安排分阶段验证。
报告默认保存在 `docs/migrations/<算法名>/migration.md`；分析报告与实际迁移、验收状态分别记录。

## 详细运行入口

- [容器配方与文档](deployment/README.md)：两个交付镜像、官方开发环境前提及项目适配。
- [PC 机载计算开发环境](deployment/onboard_computer/README.md)：AS2、算法 Library 与应用的开发及仿真验证。
- [RoboPi 运行](deployment/robopi/README.md)：配置、构建、部署、启动、验证与收尾。
- [AirSim 场景](config/airsim/README.md)：binary 资源、场景 profile 与 settings。
- [Library 注册表](libraries.yaml)：已收录算法与当前接入状态。
- [端到端 Demo](demos/README.md)：各 Library 的完整调用、场景、运行拓扑与验收入口。
- [SUPER forest8](demos/super_forest/README.md)：新 PC FollowTimedPath 长程场景、失败限制，以及历史 PC → RoboPi 单点桥记录。
- [CAT-ORA 空旷场地应用](demos/catora_open_field/README.md)：PC AirSimNH 四机起飞、队形变化、到点和降落。

## 分层与目录

| 层 / 目录 | 职责 |
| --- | --- |
| 用户任务与 `scripts/` | 场景选择、任务编排及构建运行入口 |
| `demos/` | 各 Library 的端到端示例、场景、任务、运行入口与验收记录 |
| `libraries/` | CAT-ORA、Swarm-Formation 的算法专用适配、代理与部署 |
| `src/planner/framework/`、`src/planner/interfaces/` | 公共规划契约、生命周期及 Library → AS2 输出桥 |
| `runtime/` | 共用 AS2 启动器、AirSim 传感器与时钟接入 |
| `thirdparty/` | 活动 AS2 / Pixhawk fork submodule |
| `config/` | 场景、AS2、DDS 与外部资源配置 |
| `deployment/` | flight_board 仿真底座、onboard_computer 开发环境、官方 OHOS builder 调用及 RoboPi 部署 |
| `artifacts/` | 本机运行记录、构建缓存与目标产物，不进入交付镜像 |

CAT-ORA 已在 PC AirSimNH 的指定空旷路口通过四机 Planner 飞行应用验收。
后续 RoboPi 验收包括 [CAT-ORA 四机及混合部署](demos/catora_open_field/README.md)
和 [Swarm-Formation 三机真实点云混合部署](demos/swarm_open_field/README.md)；
性能负载、失败样例及适用边界分别保留在应用目录。
各 Library 的可运行组合与验收边界见 [Demo 导航](demos/README.md)；PC 验收与 RoboPi 验收分别记录。

## 源码与本机配置

```bash
bash scripts/prepare_sources.sh
```

该入口只初始化缺失的活动 submodule，不重置本地 fork 修改。
源码版本、依赖镜像和修改约定见 [thirdparty/README.md](thirdparty/README.md)。

首次按模板创建 `.env` 与 `config/resources.local.yaml`；已有文件不要覆盖。
`.env` 保存设备、网络与构建路径，资源 YAML 保存场景 binary 位置，两者都被 Git 忽略。
OHOS 操作以完成[官方开发环境准备](deployment/oh-ros-build/README.md#前置条件)为前提，
随后按本仓库说明准备隔离 SDK、目标依赖和源码工作区。不要求 RoboPi 运行 Docker。

## 当前单机运行流程

已加载 flight_board tar、完成板端部署与本机配置后：

```bash
bash scripts/robopi.sh check
bash scripts/robopi.sh up
bash scripts/robopi.sh verify
# 仿真飞行：解锁、起飞、到点、悬停、降落、解除解锁
bash scripts/robopi.sh verify --flight
bash scripts/robopi.sh down
```

PC 运行 AirSim、PX4/Agent 及 Humble 传感器/clock；RoboPi 原生运行 AS2。
临时网络与时间校准命令、资源需求、验证范围见 [RoboPi 运行说明](deployment/robopi/README.md)。

## 测试与历史

```bash
PYTHONPATH=src/planner/framework python3 -m unittest discover -s src/planner/framework/tests -v
PYTHONPATH=runtime/airsim_sensors python3 -m unittest discover -s runtime/airsim_sensors/tests -v
```

旧 Gazebo/PX4 1.17 部署已移出当前分支，参考 main 的历史提交，
见 [旧部署参考版本](docs/history/README.md#旧部署参考版本)。
规划消息、算法实现和设计资料继续保留；完整索引见 [文档导航](docs/README.md)。
