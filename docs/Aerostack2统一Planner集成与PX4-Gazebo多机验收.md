# Aerostack2 统一 Planner 第一阶段

> 本文保留 Planner 设计与 main 历史集成参考。Gazebo 构建、启动和验收命令不属于当前分支；对应源码见[旧部署参考版本](history/README.md#旧部署参考版本)。当前运行入口见[部署导航](../deployment/README.md)。

## 当前任务

在 `F:\UAV\aerostack2-integration` 中，将 CAT-ORA 和 Swarm-Formation 作为可替换的 Planner 模块接入 Aerostack2，并使用同一套任务配置、项目内 Planner 生命周期、轨迹契约、Output Gateway 和 AS2/PX4 执行链路完成多机仿真验收。

第一阶段只负责两个 Planner 的注册、选择、运行和 PX4 SITL/Gazebo 验收，为后续 Model Hub 与 Mission Agent 提供基础。

## 已完成

- CAT-ORA 原版、四机 PX4/Gazebo 闭环及统一 Planner 冷启动验收通过。
- Swarm-Formation 原版、单机、七机 PX4/Gazebo 闭环及统一 Planner 冷启动验收通过。
- 建立本地 Planner Registry、manifest schema、默认参数模板、任务覆盖和 resolved config。
- 实现统一 `TimedTrajectorySegment`、机群原子校验、插值、过期与乱序拒绝、任务隔离和逐机 PX4 原点转换。
- 实现 `START/PAUSE/RESUME/CANCEL/GET_STATUS` 生命周期及统一结果接口。
- 实现基于真实 AS2/PX4 位置的完成判定：所有车辆同时进入容差并保持 1 秒。
- 实现 `trajectory_timeout`、只累计运行时间的 `task_timeout`，以及暂停、取消和失败后的当前位置悬停。
- 完成 CAT-ORA ROS1 代理、Swarm-Formation ROS1 代理、两个 ROS2 adapter 和公共 AS2 setpoint adapter。
- 建立六服务 Compose 部署：七机 PX4/Gazebo、七机 AS2、仿真时钟、统一运行时和两个 Planner 服务。
- 修复 Fast DDS 跨容器共享内存、PX4 多实例目录、AS2 QoS、Swarm 演示航点、七机并发起飞等问题。
- 最终 32 项测试通过：Planner framework 22 项、CAT-ORA adapter 3 项、Swarm adapter 5 项、AS2 setpoint adapter 2 项。
- 形成第一阶段设计文档、使用指南和最终验证记录。

## 验收结果

- CAT-ORA 冷启动任务 `d7e1f160-b258-42f3-b8d7-27289c45199a`：`SUCCEEDED/NONE`。
- Swarm-Formation 冷启动任务 `7aaa394c-6b50-4f03-929e-d2946ee11761`：`SUCCEEDED/NONE`。
- 已验证第二次 START 拒绝、PAUSE/RESUME、CANCEL、`NO_SOLUTION`、`TRAJECTORY_TIMEOUT`、`TASK_TIMEOUT`、`INVALID_TRAJECTORY_SET` 和失败后悬停。
- 最终 runtime 镜像：`sha256:a6abc86173a82d9d8df7e62a5e4b032590fb9a7a03720aa4c074c5c34f25ed27`。
- 最终构建、起飞、resolved config 和 Planner 日志保存在 `aerostack2-integration/artifacts/resolved/final-*`。

## 有效结论

- CAT-ORA 与 Swarm-Formation 都按相同的 Planner 行为语义运行。任务通过 `planner.id + 精确 version` 选择其中一个，同一任务不并行运行两个 Planner，也不在线热切换。当前控制面是项目自建 `/planner/control` 服务，尚未封装为 Aerostack2 原生 `BehaviorServer<ActionT>`。
- Mission 后续负责编排 Arm、Takeoff、Planner、Hover、Land 和 Disarm。单机是 `vehicles[]` 只有一项的机群特例。
- Model Hub 后续负责 Planner 的获取、注册、版本、硬件匹配和部署。第一阶段只有本地目录 Registry。
- Planner 参数采用“默认模板 + 任务局部覆盖”。未知参数报错，每次启动保存 resolved config。
- 一条 `TimedTrajectorySegment` 包含任务机群中全部无人机轨迹。CAT-ORA 输出有限长轨迹，Swarm adapter 输出滚动短时轨迹，两者字段和时间语义一致。
- 仿真统一使用 Gazebo/ROS `/clock`。Planner 契约采用机群共享 PX4 `LOCAL_NED`，不依赖 ROS `frame_id` 或 TF。
- 失败后框架悬停，但不会自动 Land 或 Disarm。独立 Hover Behavior 留给后续 Mission 层。
- 最终默认完成参数为 `position_tolerance=0.5 m`、`hold_duration=1.0 s`、`task_timeout=120 s`，任务可以覆盖。

## 固定版本

- Aerostack2：`19c397309c6a41b279e6b551818ec12605cfcecc`
- `as2_platform_pixhawk`：`fcba062219935acef69dfa009c8be376c707ee8a`
- `px4_msgs release/1.17`：`86d8239e962f6939e05c3737784f60c02fa884db`
- PX4 v1.17.0：`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- Micro-XRCE-DDS-Agent v2.4.2：`57d086216d01ec43121845d385894a25987f8a2c`
- CAT-ORA：`4bc41f593eebde65dd74108a97e80430a145dfa0`
- Swarm-Formation：`967a4bdfae949e994691f8ffc87dbb0147cbebb7`

## 卡住的问题

无第一阶段阻塞项。Handoff 中曾记录远端六个 Compose 服务仍在运行，但该状态没有在收尾时重新核实。

## 后续任务

1. 基于现有 Registry 开发 Model Hub 控制面，增加远程获取、版本管理、硬件匹配和部署。
2. 开发 Mission Agent，组合 Arm、Takeoff、Planner、Hover、Land 和 Disarm Behavior。
3. 单独开展真机、国产操作系统和物理一致障碍物场景验收。

以下内容不属于第一阶段：Planner 并行协作或热切换、自动选优、算法核心 ROS2 重写、完整远程 Model Hub、Mission Agent 自动拆解任务、真机与国产操作系统验收。

## 踩过的坑

- ROS1 Noetic 与 ROS2 Humble 使用独立容器和显式接口，不在宿主混装。
- host network 不代表容器共享 IPC。Fast DDS 跨容器通信需设置 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。
- 多机 PX4 必须使用独立实例号、system id、工作目录和参数文件，避免共享 `parameters.bson`。
- Arm、Offboard service 成功只说明命令发出，仍需从 PX4 状态 topic 核验实际状态。当前可靠顺序为 Arm、持续 setpoint、Offboard、Takeoff。
- AS2 位姿发布使用 `BEST_EFFORT`，adapter 必须采用 sensor-data QoS。
- AS2 位姿是逐机局部 ENU，送入共享 NED Planner 前必须结合 `px4_origin_in_fleet_ned` 转换。
- Swarm 上游 launch 自带多个演示航点，集成镜像必须把 `point_num` 固定为 1。
- 七机 Gazebo 高负载下并发起飞不稳定，`fleet_takeoff.py` 的顺序起飞是当前可重复基线。
- 多机 PX4 ULog 曾增长到约 74 GB，后续部署必须设置日志限制或轮转。
- Planner manifest 的 CPU 和内存需求尚未实测，不应填写估算值。
- CAT-ORA 仓库不同位置的许可证信息不一致，对外分发前需要确认许可。

## 相关文档与路径

- 集成工程：`F:\UAV\aerostack2-integration`
- RKOH 工程：`/home/noah/code/UAV/aerostack2-integration`
- 使用指南：`F:\UAV\docs\Aerostack2统一Planner使用指南.md`
- 第一阶段设计：`F:\UAV\docs\Aerostack2统一Planner第一阶段接口与配置设计.md`
- 最终验证：`F:\UAV\aerostack2-integration\artifacts\resolved\FINAL_VERIFICATION.md`
- 前序复现归档：`F:\UAV\docs\Swarm-Formation与CAT-ORA的PX4-Gazebo动力学复现及Aerostack2集成评估.md`
- CAT-ORA 部署：`F:\UAV\catora-px4-gazebo\README.md`
- Swarm-Formation 部署：`F:\UAV\swarm-formation-px4-gazebo\README.md`
- SSH：`ssh RKOH_Remote`
