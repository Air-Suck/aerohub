# Aerostack2 统一 Planner 第一阶段接口与配置设计

## 1. 目标

第一阶段在同一套 Aerostack2 系统中接入 CAT-ORA 和 Swarm-Formation。两者都是 `planner` 类型模块，任务通过配置选择其中一个，每次只运行一个 Planner。

两个 Planner 共用以下边界：

- 相同的任务配置结构
- 相同的项目内 Planner 生命周期和结果格式
- 相同的 `TimedTrajectorySegment` 输出结构
- 相同的 Output Gateway 和 AS2/PX4 执行链路
- 相同的 PX4 `LOCAL_NED` 坐标和时间语义

算法内部结构不要求相同。CAT-ORA 保持集中式批量规划，Swarm-Formation 保持持续滚动规划，差异由各自 adapter 处理。

本文沿用“Planner Behavior”表示机群规划行为及其生命周期语义。第一阶段实际接口是项目自建的 `/planner/control` 服务，不是 Aerostack2 的 `as2_behavior::BehaviorServer<ActionT>`。原生 AS2 Behavior 还需要自定义 Planner Action，并实现 `on_activate`、`on_modify`、`on_deactivate`、`on_pause`、`on_resume`、`on_run` 和 `on_execution_end`。现有 adapter、Registry、轨迹契约和 Output Gateway 可以在该封装中继续使用。

## 2. 第一阶段范围

第一阶段包含：

1. 本地 Planner Registry
2. Planner manifest、默认参数模板和参数 schema
3. 任务配置解析、参数覆盖和 resolved config 保存
4. 统一 Planner 生命周期
5. CAT-ORA adapter 和 Swarm-Formation adapter
6. `TimedTrajectorySegment`
7. Output Gateway
8. AS2 → PX4 v1.17.0 → Gazebo Harmonic 执行链路
9. 正常流程、暂停恢复、取消和失败路径测试
10. 从停止状态开始可复现的部署文档

第一阶段不包含：

- 两个 Planner 并行协作或运行中热切换
- 自动选择最优 Planner
- 远程模型仓库、自动下载和完整 Model Hub 服务
- 自动硬件适配
- 将算法核心重写成 ROS2
- 真机或国产操作系统验收
- Mission Agent 自动拆解复杂任务

## 3. 系统结构

```text
Task YAML
   │
   ▼
Mission
   │  START / PAUSE / RESUME / CANCEL / GET_STATUS
   ▼
Planner framework
   │
   ├── Local Planner Registry
   │      └── manifest + defaults + schema
   │
   └── Selected Planner Adapter
          ├── CAT-ORA
          └── Swarm-Formation
                    │
                    ▼
          TimedTrajectorySegment
                    │
                    ▼
             Output Gateway
                    │
                    ▼
        Aerostack2 motion/controller
                    │
                    ▼
                 PX4 SITL
                    │
                    ▼
             Gazebo Harmonic
```

后续 Mission 还会编排 Arm、Takeoff、Hover、Land 和 Disarm Behavior。Planner 只负责从当前空中状态生成到目标位置的轨迹，不负责起飞和降落。第一阶段尚未实现独立 Mission Agent。

## 4. 组件职责

### 4.1 Mission

- 读取任务配置
- 生成本次运行唯一的 `task_id`
- 请求 Registry 解析 Planner
- 启动并控制 Behavior
- 根据车辆实际状态判断任务是否完成
- 聚合机群级结果
- Planner 失败时启动 Hover Behavior

### 4.2 Local Planner Registry

- 按 `planner.id + planner.version` 查找唯一 manifest
- 加载默认参数和参数 schema
- 校验能力、平台和资源声明
- 不负责远程下载、评分或自动选择

第一阶段可用目录和文件完成 Registry，不需要数据库或常驻服务。

### 4.3 Planner framework

- 提供统一的启动、暂停、恢复、取消和状态接口
- 管理一个任务机群中的全部无人机
- 启动所选 Planner 和 adapter
- 将 Planner 或 adapter 的错误转换为统一状态和原因码
- 不解释算法内部消息
- 不直接处理 PX4 坐标原点转换

### 4.4 Planner Adapter

- 将统一任务目标转换为算法原生输入
- 将算法原生输出转换为 `TimedTrajectorySegment`
- 将 ENU 等算法坐标转换为机群共享 `LOCAL_NED`
- 保证同一 `task_id` 下的 `generated_at` 严格递增
- 补齐轨迹时间结构，但不得伪造未提供的速度、加速度或 yaw

CAT-ORA adapter 直接使用逐机目标，输出较长的有限轨迹段。

Swarm-Formation adapter 根据逐机目标计算编队中心和相对偏移，并从 `traj_utils/PolyTraj` 采样短时滚动轨迹段。首期不从单点 `PositionCommand` 外推未来轨迹。

### 4.5 Output Gateway

- 校验整条机群轨迹消息
- 拒绝非当前任务、重复、乱序或过期消息
- 按仿真时钟采样当前参考
- 将机群共享 `LOCAL_NED` 转换为各 PX4 的本地 NED
- 将未声明有效的可选字段映射为 PX4 `NaN`
- 发现轨迹供给中断时停止使用旧轨迹并维持安全参考
- 向 Planner framework 报告失败原因

### 4.6 Hover Behavior

第一阶段由 Planner framework 在暂停、取消、超时或失败时丢弃旧轨迹，并向所有仍可控的空中车辆发送基于当前实际位置的悬停参考。当前任务随后进入对应终态，不自动执行 Land 或 Disarm。独立 Hover Behavior 及其由 Mission 接管的流程留到后续 Mission 层实现。

## 5. 任务配置

用户编写的任务文件采用最小结构：

```yaml
task:
  name: formation_demo_01

planner:
  id: cat_ora
  version: 4bc41f593eebde65dd74108a97e80430a145dfa0
  parameters: {}

vehicles:
  - uav1
  - uav2

targets:
  uav1: {x: 0.0, y: 0.0, z: -3.0}
  uav2: {x: 2.0, y: 0.0, z: -3.0}

completion:
  position_tolerance: 0.5
  hold_duration: 1.0
  task_timeout: 120.0
```

约束：

- `task.name` 可重复使用，不承担运行隔离职责。
- Mission 每次启动生成新的 `task_id`。
- 暂停和恢复沿用原 `task_id`。
- 取消后再次启动必须生成新的 `task_id`。
- 算法专属参数只能放在 `planner.parameters`。
- 用户任务文件不重复填写固定坐标约定、镜像、topic 或 PX4 原点。

### 5.1 参数解析

解析顺序：

1. Registry 定位精确 Planner 版本。
2. 加载该版本的默认参数模板。
3. 递归合并 `planner.parameters`。
4. 使用 schema 校验最终参数。
5. 补充运行时信息并保存 resolved task config。

合并规则：

- YAML 映射递归合并
- 标量直接覆盖
- 数组整体替换
- 未在 schema 中声明的键直接报错
- 不实现数组逐项合并或第三层优先级

### 5.2 启动前校验

Mission 在启动 Planner 和解锁车辆前原子校验：

- `vehicles` 至少包含一项，且没有重复值
- `targets` 与 `vehicles` 一一对应
- 坐标和数值不含 `NaN` 或 `Inf`
- Planner 的 id 和精确版本存在
- 参数覆盖符合 schema
- Planner 支持当前机群规模和 `TimedTrajectorySegment`
- 每架无人机都有部署配置和 PX4 原点映射

任何一项失败，整项任务进入 `REJECTED`，原因为 `INVALID_TASK`。系统不启动 Planner，也不发送飞行指令。

## 6. Planner manifest

两个 Planner 使用同一 manifest schema。下例只表示结构：

```yaml
id: cat_ora
version: 4bc41f593eebde65dd74108a97e80430a145dfa0

runtime:
  image: uav/cat-ora:4bc41f5
  command: []
  adapter: cat_ora_adapter

parameters:
  defaults: cat_ora.defaults.yaml
  schema: cat_ora.schema.yaml

capabilities:
  planning_mode: batch
  min_vehicles: 1
  max_vehicles: 4
  required_inputs:
    - vehicle_state

output:
  type: TimedTrajectorySegment
  interface_version: 1

resources:
  cpu: 4
  memory_mb: 4096
  gpu_required: false

platforms:
  - linux/amd64
```

第一阶段不加入评分、远程地址、自动下载和许可证管理字段。资源值必须来自实际测量或已有复现结果，不能用占位估计值进入正式 manifest。

## 7. Resolved task config

resolved config 是一次运行的完整记录，至少包含：

- `task.name` 和本次运行唯一的 `task_id`
- Planner id、精确版本和 manifest 路径
- 默认参数与任务覆盖合并后的最终参数
- 参与车辆和逐机目标
- 完成判据
- 固定的 `LOCAL_NED` 坐标约定
- 每架无人机的 `px4_origin_in_fleet_ned`
- 容器镜像、启动命令、namespace 和 topic 映射
- 仿真时钟设置

它用于复现和排错，不由用户手工维护。

## 8. `TimedTrajectorySegment`

这是 Planner/Adapter 发给 Output Gateway 的中立逻辑契约。实现可以映射为 ROS2 消息，但契约本身不依赖 ROS 的 `Header.frame_id`。

```text
TimedTrajectorySegment
  interface_version
  task_id
  generated_at
  valid_from
  is_final
  vehicle_trajectories[]

VehicleTrajectory
  vehicle_id
  valid_fields[]
  points[]

TimedTrajectoryPoint
  time_from_start
  position {x, y, z}
  velocity {x, y, z}       optional
  acceleration {x, y, z}   optional
  yaw                       optional
```

### 8.1 消息粒度

- 一条消息只对应一个 `task_id` 和一个任务机群。
- `vehicle_trajectories[]` 必须完整覆盖 resolved config 中的全部参与车辆。
- 不允许缺失、重复或出现任务外车辆。
- 任一车辆轨迹无效时，Gateway 拒绝整条消息，不允许部分执行。

### 8.2 时间语义

- 仿真阶段所有节点使用 Gazebo/ROS `/clock`。
- `generated_at` 表示该版本轨迹的生成时间，用于判断重复和乱序。
- `valid_from` 是全机群共享的绝对执行基时间。
- `time_from_start` 是相对 `valid_from` 的偏移。
- 每架无人机的 `time_from_start` 必须非负并严格递增。
- 各机点数可以不同，但最后一点的相对时间必须一致。
- 提前结束的车辆由 adapter 补终点保持点。

同一 `task_id` 下：

- 更晚的 `generated_at` 替换旧段
- 相同的 `generated_at` 视为重复并忽略
- 更早的 `generated_at` 视为乱序并拒绝
- 首期不增加 `segment_id`

消息到达时已经没有任何可执行覆盖范围，Gateway 直接拒绝。非最终段耗尽后仍没有后继有效段，开始累计 `trajectory_timeout`。

### 8.3 最终段

`is_final=true` 只表示 Planner 不再提供后继轨迹，不代表无人机已经到达目标。最终段执行完毕后，Gateway 保持终点，Mission 继续等待完成判据或 `task_timeout`。

### 8.4 必填和可选字段

- `time_from_start` 和 position 必填。
- velocity、acceleration 和 yaw 可选。
- `valid_fields` 声明哪些可选字段可信。
- Gateway 将未声明有效的 PX4 setpoint 字段写为 `NaN`。
- adapter 不得为了填满消息而伪造数值。

## 9. 坐标约定

统一 Planner 契约完全采用 PX4 语义：

- 世界坐标为任务级机群共享 `LOCAL_NED`
- `x` 指北，`y` 指东，`z` 指下
- 机体系采用 FRD
- yaw 单位为弧度，范围为 `[-π, π]`
- `yaw=0` 指向北，`yaw=π/2` 指向东

任务配置定义一个机群共享原点。`TimedTrajectorySegment` 通过 `task_id` 继承 resolved config 中的坐标定义，不携带自由文本 `frame_id`。

仿真阶段以 Gazebo 中的出生位姿为唯一来源，生成每架无人机的 `px4_origin_in_fleet_ned`。各坐标轴保持同向时，Gateway 使用：

```text
p_px4_i = p_fleet - px4_origin_in_fleet_ned[i]
```

解析后的原点映射必须写入 resolved config。第一阶段不依赖 ROS TF 完成这项转换。

## 10. Planner 公共接口

### 10.1 命令

```text
START
PAUSE
RESUME
CANCEL
GET_STATUS
```

### 10.2 状态

```text
REJECTED
RUNNING
PAUSED
SUCCEEDED
FAILED
CANCELED
```

`REJECTED`、`SUCCEEDED`、`FAILED` 和 `CANCELED` 是终态。只有 `PAUSED` 可以恢复为 `RUNNING`。不定义 `STOPPED`。

### 10.3 状态与原因码

状态只表达生命周期，具体原因单独记录。第一阶段至少需要：

```text
NONE
INVALID_TASK
INVALID_TRAJECTORY_SET
NO_SOLUTION
TRAJECTORY_TIMEOUT
TASK_TIMEOUT
```

### 10.4 BehaviorResult

所有机群 Planner 任务共用一个结果结构：

```text
BehaviorResult
  behavior_type
  task_id
  overall_status
  failure_reason
  vehicle_results[]

VehicleResult
  vehicle_id
  status
  failure_reason
  detail
```

单机是 `vehicles[]` 只有一项的机群特例，不使用另一套 API。所有车辆满足任务完成条件后，`overall_status` 才能进入 `SUCCEEDED`。任一车辆发生不可恢复故障，机群级状态进入 `FAILED`，逐机结果保留诊断信息。

## 11. 生命周期规则

### 11.1 正常执行

```text
任务校验
  → Arm Behavior
  → Takeoff Behavior
  → Planner Behavior
  → Land Behavior
  → Disarm Behavior
```

Mission 负责排序和错误处理，各 Behavior 负责自身动作。

### 11.2 完成判据

- 所有参与无人机必须同时进入各自目标的三维位置容差。
- 整个机群必须连续保持 `hold_duration`。
- 任一无人机离开容差，保持计时归零。
- 默认值为 `position_tolerance=0.5 m`、`hold_duration=1.0 s`、`task_timeout=120 s`。
- Mission 使用 AS2/PX4 实际位置反馈判断，不使用 Planner 轨迹值判断。

### 11.3 暂停和恢复

暂停时：

- 保留任务和 resolved config
- 停止生成和接受新轨迹
- 丢弃暂停前尚未执行的轨迹
- 启动悬停参考
- 冻结 `task_timeout`
- 停用轨迹新鲜度检查

恢复时：

- 沿用原 `task_id`
- 基于最新实际状态重新规划
- 等待一条新的有效轨迹
- 禁止续播暂停前轨迹

Planner 明确返回无解时进入 `FAILED/NO_SOLUTION`。Planner 没有响应时由 `trajectory_timeout` 收口为 `FAILED/TRAJECTORY_TIMEOUT`。

### 11.4 取消

- 任务进入 `CANCELED`。
- Planner 停止运行。
- Gateway 拒绝该 `task_id` 的后续消息。
- Planner framework 发送当前位置悬停参考。
- 已取消任务不能恢复，继续运动必须启动新任务。

### 11.5 超时和失败

| 条件 | Behavior 状态 | 原因码 | 安全动作 |
| --- | --- | --- | --- |
| 启动前配置或能力无效 | `REJECTED` | `INVALID_TASK` | 不发送飞行指令 |
| Planner 明确无解 | `FAILED` | `NO_SOLUTION` | 当前位置悬停 |
| 非最终轨迹供给中断 | `FAILED` | `TRAJECTORY_TIMEOUT` | 当前位置悬停 |
| 到达目标超过任务总时限 | `FAILED` | `TASK_TIMEOUT` | 当前位置悬停 |
| 机群轨迹集合无效 | `FAILED` | `INVALID_TRAJECTORY_SET` | 拒绝整段并保持当前位置 |
| 用户主动取消 | `CANCELED` | `NONE` | 当前位置悬停 |

`trajectory_timeout` 是所有 Planner 共用的参数名，各 Planner 默认模板可以设置不同值，任务也可以覆盖。它只监控 Planner 本应继续提供轨迹时的供给中断。车辆能否在规定时间内到达目标只由 `task_timeout` 判断。

## 12. 第一阶段测试与验收

### 12.1 配置与契约测试

- 模板与任务覆盖合并正确
- 数组整体替换
- 未知参数被拒绝
- resolved config 可重复生成并保存
- `task_id` 隔离旧任务消息
- 轨迹完整性、时间顺序、重复、乱序和过期校验
- `LOCAL_NED` 到逐机 PX4 本地 NED 的转换
- 未声明可选字段映射为 `NaN`
- Behavior 状态转换和原因码

### 12.2 平台链路

单机完成以下闭环：

```text
AS2 Arm/Takeoff/Follow reference/Land
  → as2_platform_pixhawk
  → PX4 v1.17.0 SITL
  → Gazebo Harmonic
```

验收时同时核对 ROS2 状态、PX4 模式和 Gazebo 刚体位置，不能只看 topic 或可视化标记。

### 12.3 CAT-ORA

- 通过任务配置选择 CAT-ORA
- 四机目标正确转换
- 输出统一 `TimedTrajectorySegment`
- 四路 PX4 和 Gazebo 刚体真实运动
- 全机群满足目标误差、机间安全约束和连续稳定 1 秒

### 12.4 Swarm-Formation

- 通过同一任务结构选择 Swarm-Formation
- 逐机目标转换为中心和相对队形
- adapter 从 `PolyTraj` 生成短时滚动段
- 原生地图输入、滚动重规划和队形运动进入同一执行链
- 全机群满足目标误差和连续稳定 1 秒

当前复现基线的 ROS 地图生成器使用 `obs_num=0`，因此本阶段验证的是 Planner 接口和真实 PX4/Gazebo 动力学闭环，不把虚拟点云障碍物与 Gazebo 碰撞体一致性列入验收。物理一致的障碍物场景属于后续高速穿梭 demo。

### 12.5 异常路径

- 暂停后旧轨迹不能恢复执行
- 取消后迟到消息被拒绝
- 显式无解进入 `FAILED/NO_SOLUTION`
- 轨迹断流进入 `FAILED/TRAJECTORY_TIMEOUT`
- 任务超时进入 `FAILED/TASK_TIMEOUT`
- 无效机群轨迹整段拒绝
- 失败后当前位置悬停，且不会自动降落

### 12.6 交付

- 从停止状态完成冷启动复验
- Docker/Compose 和所有版本固定
- 保存构建日志、测试结果和 resolved config
- CAT-ORA、Swarm-Formation 和统一集成工程均有部署文档

## 13. 实施顺序

1. 完成单机 AS2 → PX4 v1.17.0 → Gazebo 闭环。
2. 落地任务 schema、manifest schema 和本地 Registry。
3. 实现 Behavior 公共状态与 resolved config。
4. 实现 `TimedTrajectorySegment` 和 Output Gateway。
5. 接入 CAT-ORA 并完成四机验收。
6. 接入 Swarm-Formation 并完成多机验收。
7. 补齐异常路径、冷启动和部署文档。

## 14. 当前实现映射

截至 2026-10-05，第一阶段实现与验收结果如下：

- 五个 ROS 2 包已经由 `deployment/runtime/Dockerfile` 构建；最终交付 runtime 镜像为 `sha256:a6abc86173a82d9d8df7e62a5e4b032590fb9a7a03720aa4c074c5c34f25ed27`。
- Planner framework 22 项、CAT-ORA adapter 3 项、Swarm-Formation adapter 5 项、AS2 setpoint adapter 2 项测试全部通过。
- `deployment/integration/compose.yaml` 可从停止状态启动七机 PX4/Gazebo、七机 AS2、ROS2 `/clock`、统一 runtime 和两个 ROS1 Planner。六个服务均在最终复验中保持运行。
- Gazebo 时钟经最小代理发布到 ROS2 `/clock`，实测约 100 Hz；resolved config 保存 `use_sim_time: true` 和 `clock_topic: /clock`。
- CAT-ORA adapter 会把 ROS1 有限轨迹转换为原子四机 `TimedTrajectorySegment`，并拒绝改变固定逐机目标归属的 CAT-ORA assignment。最终冷启动任务 `d7e1f160-b258-42f3-b8d7-27289c45199a` 返回 `SUCCEEDED/NONE`。
- Swarm-Formation adapter 会把原生七机 `PolyTraj` 转为短时滚动段，校验机群完整性、首点连续性并隔离旧任务轨迹。最终冷启动任务 `7aaa394c-6b50-4f03-929e-d2946ee11761` 返回 `SUCCEEDED/NONE`。
- 两次动力学冷启动运行于行为代码相同的 `1337ee…` 镜像。其后的 `a6abc8…` 只修正 manifest 镜像元数据并增加 schema 校验；该镜像重新通过全部 32 项测试，且已在运行容器中分别解析 CAT-ORA 四机和 Swarm-Formation 七机任务，得到正确的集成镜像名。
- Output Gateway、AS2 setpoint adapter、PX4 v1.17.0 SITL 和 Gazebo Harmonic 组成两个 Planner 共用的执行链。完成判定使用 AS2/PX4 实际位置，要求全机群进入逐机目标容差并连续稳定 1 秒。
- 已真实验证二次 START 保护、PAUSE/RESUME、CANCEL、`NO_SOLUTION`、`TRAJECTORY_TIMEOUT`、`TASK_TIMEOUT`、无效机群轨迹整段拒绝和失败后当前位置悬停。任务超时后抽测四机 3 秒位移约为 `0.018–0.053 m`。
- resolved config、Planner/output 日志和起飞日志保存到 `artifacts/resolved`。根 README 给出 Compose、起飞、两个 Planner、状态查询和安全语义的完整命令。

第一阶段仍不包含独立 Mission Agent、独立 Hover Behavior、Planner 热切换、远程 Model Hub、真机、国产操作系统和物理一致障碍物场景；这些属于后续阶段。
