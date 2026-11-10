# AeroHub

AeroHub 基于 Aerostack2 构建无人机开源能力的适配、部署和运行框架。项目通过 Offline Agent 将外部算法整理为符合 AS2 接口、生命周期和部署约束的组件，并由 Mission 层的 Online Agent 根据能力卡片完成任务拆解与能力调用。

当前统一复现与验收平台为 **AirSim + PX4 SITL**。仓库中的 Gazebo 工程仅作为锁定的上游参考，不作为后续复现工作的默认平台。

## 架构

运行时目标分层为：

```text
Application
    ↓
Mission / Online Agent
    ↓
Tool
    ↓
基础 Behavior
    ↓
状态估计 / 环境感知 / 运动控制等基础能力
    ↓
AS2 Aerial Platform
    ↓
PX4 SITL + AirSim
```

Tool 不必经过运动 Behavior 才能读取状态、传感器或地图。State Estimator 负责飞行器自身状态，Perception 负责环境感知、建图和环境表示。通信、生命周期和资源仲裁贯穿各层。完整设计与接口说明见[项目简介](docs/项目简介.md)。

## 当前 AirSim / RoboPi 入口

全仓库的 Library、算法、适配组件及 Demo 统一采用四个运行角色的职责约定；完整职责见[项目简介](docs/项目简介.md#deployment-roles)。

| 角色 | 当前对应 |
| --- | --- |
| 仿真器 | PC 宿主 AirSim，提供环境、动力学和仿真真值 |
| 飞控板 | `flight_board` 容器中的 PX4 SITL；同时包装传感器/时钟桥 |
| 算力板 | RoboPi 原生运行算法、导航输入、Adapter、AS2 与 FollowTimedPath |
| 工作站 | 固定 ROS 桌面镜像中的 `mission.py`，通过 ROS 调用已就绪的算力板能力 |

该约定适用于 CAT-ORA、Swarm-Formation、SUPER、状态估计与环境感知等组件。角色可在 PC 仿真中共置，实际部署位置和验收状态由各项目单独声明。SUPER 是当前实例，不代表其他项目已迁移或复验。

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
- 主线 AS2 依赖仍在 `as2/aerostack2/` 和 `as2/as2_platform_pixhawk/`；AS2 核心直接锁定 AeroHub fork，Pixhawk 的 PX4 1.18 适配在构建副本应用兼容补丁。
- SUPER 实际源码与 CAT-ORA 一致：`src/planner/super/ros2_adapter/native/`，AS2 适配在 `bridge/`，`upstream/` 子模块只作参考。运行产物进入被忽略的 `log/airsim/`。

已有 Skill 集中在 `skill/`；历史运行证据保留在忽略的 `log/pr-history/`，复现过程记录在 `skill/temp/`。`docs/` 只跟踪三份主文档及其图片。AS2 直接使用 AeroHub fork 子模块并锁定到 `f387e0d`；构建流程不再对官方 AS2 副本重复应用同等补丁。

主线依赖准备入口仍为 `bash deployment/prepare_sources.sh`；AirSim / SUPER 使用 `bash tools/robopi/prepare_sources.sh`。
两者都需从仓库根目录执行后续构建命令。AirSim 构建还依赖 PR 指定的基础镜像与外部仿真资源；目录迁移不表示它们已在当前机器上完成闭环验收。
迁移说明与验证边界见[项目简介](docs/项目简介.md#pr-目录适配)。

## 上游来源、修改范围与许可

本项目依赖并致谢以下上游项目：

| 上游 | 本工程中的用途 | 锁定版本 |
| --- | --- | --- |
| [Aerostack2（AeroHub fork）](https://github.com/Popc0rn7/aerostack2-aerohub) | 状态估计、运动控制、运动 Behavior 及公共接口 | `f387e0d733a5102808508bcbec1b9eca2c29d1f0` |
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

| 目录 | 用途 |
| --- | --- |
| `src/` | 自建代码，按 `planner/`、`behavior/`、`state_estimator/`、`perception/` 分类 |
| `as2/` | Aerostack2、平台插件和消息依赖 |
| `px4/` | 锁定版本的 PX4-Autopilot |
| `simulator/` | AirSim 集成代码及 Gazebo 上游参考工程 |
| `deployment/` | 镜像、运行拓扑、AirSim Demo 和设备部署入口 |
| `skill/` | 复现过程中提炼的可复用 Skill；临时记录位于被忽略的 `skill/temp/` |
| `cards/` | 按能力类型组织的能力卡片 |
| `tools/` | 构建、部署、诊断和辅助工具 |
| `vendor/` | 有明确来源与许可证的第三方依赖或快照 |
| `docs/` | 项目、AS2 和贡献文档 |
| `log/` | 被 Git 忽略的测试证据与运行产物 |

## 初始化

克隆仓库后初始化锁定依赖：

```bash
git submodule update --init --recursive
```

RoboPi 和 AirSim 开发环境优先使用项目准备脚本：

```bash
bash tools/robopi/prepare_sources.sh
```

不要在主仓库中直接修改上游依赖。确需修改时，应先说明原因、来源和维护方式，并将修改放在明确管理的 fork 或本项目扩展包中。

## 构建、运行与验证

当前入口按用途划分：

- [交付与开发环境](deployment/README.md)：镜像、RoboPi、AirSim 和源码准备顺序。
- [AirSim Demo](deployment/airsim/demos/README.md)：各算法的运行拓扑、前置检查和端到端验收入口。
- [机载开发环境](deployment/onboard_computer/README.md)：PC AS2 和算法开发镜像。
- [飞控仿真底座](deployment/flight_board/README.md)：PX4 SITL、DDS、传感器与时钟。
- [RoboPi 部署](deployment/robopi/README.md)：板端构建、部署、启动和诊断。

验证结论必须注明平台、版本、拓扑和覆盖范围。静态检查、合成输入冒烟、容器启动、AirSim 闭环和真机验收是不同证据等级，不能互相替代。当前开发主机具备 NVIDIA GPU；依赖 GPU 的算法必须完成真实 GPU 运行测试。只有预先确定的适用测试全部执行并通过、证据完整时才能声明复现成功；失败、跳过、未执行或缺证据均表示复现尚未成功。

## 文档

- [项目简介](docs/项目简介.md)：目录、目标架构、Planner 接口和当前实现入口。
- [AS2 简介](docs/as2简介.md)：原生 Aerostack2 架构、组件和接口。
- [开发者贡献指导](docs/开发者贡献指导.md)：扩展方式、目录规则、Docker 构建与验收流程。

## 上游来源与许可

主要上游依赖包括：

| 上游 | 本项目中的用途 |
| --- | --- |
| [Aerostack2](https://github.com/aerostack2/aerostack2) | 飞行执行框架、Behavior、状态估计与公共接口 |
| [as2_platform_pixhawk](https://github.com/aerostack2/as2_platform_pixhawk) | AS2 与 PX4 的平台适配 |
| [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | 飞控与 SITL |
| [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) | 激光、视觉、惯性状态估计与建图 |
| [PIN-SLAM](https://github.com/PRBonn/PIN_SLAM) | 激光状态估计与隐式神经点建图 |
| [CAT-ORA](https://github.com/ctu-mrs/catora) | 多机轨迹规划 |
| [Swarm-Formation](https://github.com/ZJU-FAST-Lab/Swarm-Formation) | 多机编队规划 |

上游仓库和精确提交通过 `.gitmodules` 及外层 gitlink 锁定。AirSim 构建对 AS2 与 Pixhawk 的兼容修改以补丁形式应用，来源和修改边界见 [AS2 补丁说明](deployment/platform/patches/as2-pr-provenance.md)与 [Pixhawk 补丁说明](deployment/platform/patches/px4-v1.18-provenance.md)。引入新上游代码时必须保留许可证和来源记录，不得把第三方代码误标为本项目原创实现。

本仓库可能同时包含多种许可证的软件，使用和再分发时需分别遵守其许可条款。仓库根目录目前没有一份覆盖全部自建代码的统一 `LICENSE`；各 ROS 包的许可证声明也不能覆盖第三方源码，发布源码、镜像或二进制前必须按实际包含的组件完成许可证核查。
