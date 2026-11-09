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

验证结论必须注明平台、版本、拓扑和覆盖范围。静态检查、合成输入冒烟、容器启动、AirSim 闭环和真机验收是不同证据等级，不能互相替代。需要 GPU 的测试在没有可用 GPU 时应明确标记为未执行。

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
| [CAT-ORA](https://github.com/ctu-mrs/catora) | 多机轨迹规划 |
| [Swarm-Formation](https://github.com/ZJU-FAST-Lab/Swarm-Formation) | 多机编队规划 |

上游仓库和精确提交通过 `.gitmodules` 及外层 gitlink 锁定。AirSim 构建对 AS2 与 Pixhawk 的兼容修改以补丁形式应用，来源和修改边界见 [AS2 补丁说明](deployment/platform/patches/as2-pr-provenance.md)与 [Pixhawk 补丁说明](deployment/platform/patches/px4-v1.18-provenance.md)。引入新上游代码时必须保留许可证和来源记录，不得把第三方代码误标为本项目原创实现。

本仓库可能同时包含多种许可证的软件，使用和再分发时需分别遵守其许可条款。仓库根目录目前没有一份覆盖全部自建代码的统一 `LICENSE`；各 ROS 包的许可证声明也不能覆盖第三方源码，发布源码、镜像或二进制前必须按实际包含的组件完成许可证核查。
