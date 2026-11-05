# AeroHub

围绕 AS2 Runtime、算法 Library 与用户任务组织的无人机开发仓库。
当前已通过 PC AirSim/PX4 1.18 → RoboPi AS2 的单机仿真闭环。

## 交付与运行入口

**OHOS / RoboPi 使用前提：先完成官方[设备开发环境搭建和开发指南](https://gitcode.com/openharmony-robot/docs/blob/main/device-dev/%E8%AE%BE%E5%A4%87%E5%BC%80%E5%8F%91%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA%E5%92%8C%E5%BC%80%E5%8F%91%E6%8C%87%E5%8D%97.md)，再执行本仓库的项目适配、编译和部署步骤。**

交付的 tar 是 PC 仿真运行镜像 `flight_board`。OHOS 开发环境和官方 builder
由用户按上述指南准备，再按[项目适配说明](deployment/oh-ros-build/README.md)接入。

- [容器配方与文档](deployment/README.md)：flight_board tar、官方开发环境前提及项目适配。
- [RoboPi 运行](deployment/robopi/README.md)：配置、构建、部署、启动、验证与收尾。
- [AirSim 场景](config/airsim/README.md)：binary 资源、场景 profile 与 settings。
- [Library 注册表](libraries.yaml)：已收录算法与当前接入状态。
- [CAT-ORA 空旷场地应用](apps/catora_open_field/README.md)：PC AirSimNH 四机起飞、队形变化、到点和降落。

## 分层与目录

| 层 / 目录 | 职责 |
| --- | --- |
| 用户任务与 `scripts/` | 场景选择、任务编排及构建运行入口 |
| `apps/` | 可重复运行的应用任务及对应场景、部署和验收记录 |
| `libraries/` | CAT-ORA、Swarm-Formation 的算法专用适配、代理与部署 |
| `planner_framework/`、`planner_interfaces/` | 公共规划契约、生命周期及 Library → AS2 输出桥 |
| `runtime/` | 共用 AS2 启动器、AirSim 传感器与时钟接入 |
| `thirdparty/` | 活动 AS2 / Pixhawk fork submodule |
| `config/` | 场景、AS2、DDS 与外部资源配置 |
| `deployment/` | flight_board 镜像说明、官方 OHOS builder 调用及 RoboPi 部署 |
| `artifacts/` | 本机运行记录、构建缓存与目标产物，不进入交付镜像 |

CAT-ORA 已在 PC AirSimNH 的指定空旷路口通过四机 Planner 飞行应用验收。
Swarm-Formation 实景接入仍待完成；PC 验收不代表算法已在 RoboPi 上运行。

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
PYTHONPATH=planner_framework python3 -m unittest discover -s planner_framework/tests -v
PYTHONPATH=runtime/airsim_sensors python3 -m unittest discover -s runtime/airsim_sensors/tests -v
```

旧 Gazebo/PX4 1.17 部署已移出当前分支，参考 main 的历史提交，
见 [旧部署参考版本](docs/history/README.md#旧部署参考版本)。
规划消息、算法实现和设计资料继续保留；完整索引见 [文档导航](docs/README.md)。
