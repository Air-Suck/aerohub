> 历史快照：2026-11-03 合并前版本，仅用于追溯。当前说明统一维护在[第一阶段接口与配置设计](../../Aerostack2统一Planner第一阶段接口与配置设计.md)，快照中的旧结论和命令不作为当前使用依据。

# 统一 Planner 框架：集成设计与使用

本文面向需要理解、运行或扩展当前工程的开发者。内容依据 2026-11-03 本地源码核对；历史动力学验收结果见[最终验证记录](../../../artifacts/resolved/FINAL_VERIFICATION.md)，本次文档编写没有重新执行仿真。

Aerostack2 的分层与执行链介绍见 [Aerostack2 架构介绍](../../Aerostack2架构介绍.md)。

## 1. 为什么要统一 Planner

CAT-ORA 和 Swarm-Formation 的内部工作方式不同：前者集中计算有限轨迹，后者持续输出滚动轨迹。如果上层任务直接调用各自原生接口，就需要分别处理参数、坐标、时间、状态、超时和控制输出。

本工程把这些公共问题提取为框架，保留算法原有实现，并用 adapter 消化差异。用户通过任务配置选择 Planner；执行侧始终接收相同的机群轨迹契约。

| 公共部分 | 统一后的职责 |
| --- | --- |
| Registry | 按算法 id 和精确版本定位 manifest、参数模板和 schema |
| 任务解析 | 校验车辆、目标和部署映射，合并参数并保存 resolved config |
| 生命周期 | 启动、暂停、恢复、取消、状态查询与失败收口 |
| 轨迹契约 | 全机群、带时间信息的 `TimedTrajectorySegment` |
| Output Gateway | 整段校验、插值、过期处理及逐机原点转换 |
| AS2 输出适配 | 将参考送入各机 Aerostack2 执行链 |
| 完成判定 | 使用实际位置确认整个机群完成任务 |

当前是“单个活跃任务选择一个 Planner”。ROS 1 算法容器由 Compose 预先启动；framework 启动的是对应 ROS 2 adapter 子进程，并不根据 manifest 自动创建或调度容器。

## 2. 组件关系与接口

```mermaid
flowchart TB
    TASK[任务 YAML + 部署 YAML] --> FW[Planner framework / Registry / 生命周期]
    FW --> ADAPTER[所选 ROS 2 Planner adapter]
    ADAPTER <-->|TCP / JSON| PROXY[ROS 1 proxy + 原生算法]
    ADAPTER -->|/planner/trajectory| GW[Output Gateway：framework 内部]
    GW -->|/planner/setpoints| OUT[AS2 setpoint adapter]
    OUT --> AS2[逐机 Aerostack2 执行链]
    AS2 --> PX4[PX4 SITL + Gazebo]
    AS2 -->|实际位姿| FW
    AS2 -->|实际位姿| ADAPTER
    FW --> STATUS[/planner/status]
```

Gateway 是 framework 内部逻辑，不是单独的 Compose 服务。统一接口的 ROS 包名为 `uav_planner_interfaces`，源码目录名为 `planner_interfaces`。

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/planner/control` | `ControlPlanner` service | 控制任务和查询状态 |
| `/planner/trajectory` | `TimedTrajectorySegment` topic | adapter 输出全机群轨迹 |
| `/planner/failure` | `PlannerFailure` topic | adapter 报告失败 |
| `/planner/setpoints` | `FleetSetpoint` topic | Gateway 输出本次采样的逐机参考 |
| `/planner/status` | `BehaviorResult` topic | 任务状态；不是固定频率的持续遥测 |
| `/<vehicle>/self_localization/pose` | `PoseStamped` topic | 实际位置反馈 |

控制接口采用项目自建 Behavior 语义，尚不是 AS2 原生 Action Server。`BehaviorResult` 消息虽然定义了 `vehicle_results`，当前节点主要填写机群状态、任务 id 和原因码，未填充逐机结果数组。

### 2.1 接口数据总览

下文以实际源码为准区分磁盘文件、ROS 消息和 socket 数据。ROS 数据用 YAML 或字段树展示，JSON 用于 TCP 代理；展示格式不表示所有接口都使用 JSON。

| 发送方 → 接收方 | 通信形式 | 数据结构 | 关键语义 |
| --- | --- | --- | --- |
| 使用者 → framework | ROS 2 service | `ControlPlanner.Request` | 命令及容器内配置文件路径 |
| Registry → framework | Python 字典 | resolved config | 生成 task_id，合并参数并附带部署信息 |
| framework → 两类 adapter | YAML 文件 + 进程参数 | resolved config | `resolved_config:=文件路径`；不是通过 ROS topic 发送配置 |
| AS2 → framework、Planner adapter、输出 adapter | ROS 2 topic | `geometry_msgs/PoseStamped` | 逐机局部 ENU 位姿 |
| CAT-ORA adapter ↔ ROS 1 proxy | TCP 28500，逐行 JSON | 起终点及参数 ↔ 逐机轨迹点数组 | 机群共享 ENU，车辆靠数组顺序对应 |
| Swarm adapter ↔ ROS 1 proxy | TCP 28501，逐行 JSON | 机群状态及编队中心 ↔ 逐机轨迹点数组 | 固定 drone0…drone6，机群共享 ENU |
| ROS 1 proxy ↔ 原生算法 | ROS 1 service / topics | CAT-ORA 服务、Odometry、PoseStamped、PolyTraj | 保留各算法原生接口 |
| Planner adapter → framework/Gateway | ROS 2 topic | `TimedTrajectorySegment` | 完整机群、共享 NED、明确执行时间 |
| Gateway → AS2 输出 adapter | ROS 2 topic | `FleetSetpoint` | 当前采样值，已转为逐机本地 NED |
| 输出 adapter → AS2 | ROS 2 service + topics | `SetControlMode`、`PoseStamped`、`TwistStamped` | 本地 ENU 位置、yaw 和速度限制 |
| Planner adapter → framework | ROS 2 topic | `PlannerFailure` | task_id、原因码、诊断文本 |
| framework → 使用者 | ROS 2 service response / topic | `BehaviorResult` | 任务状态与失败原因 |
| Gazebo 时钟代理 → ROS 2 时钟桥 → 节点 | UDP 28502 → ROS 2 topic | 12 字节时间包 → `rosgraph_msgs/Clock` | 仿真时间基准 |

### 2.2 配置文件与进程启动接口

任务 YAML 描述“做什么”，部署 YAML 描述“车辆在哪里、状态从哪里读”。完整任务示例见第 4 节。部署文件中的单机条目如下，实际文件必须完整包含任务所选车辆：

```yaml
drone0:
  px4_origin_in_fleet_ned: [-4.5, -12.0, 0.0]
  namespace: /drone0
  topics:
    pose: /drone0/self_localization/pose
```

`px4_origin_in_fleet_ned` 是该机局部原点在共享 NED 中的位置，单位米。namespace 和 topic 为 ROS 名称。部分旧示例还含 `trajectory_setpoint`，当前 AS2 adapter 不读取它，而是根据 namespace 构造 `motion_reference` topic。

Registry 的输出先作为 Python 字典返回，framework 再补充时间配置并写成 YAML。主要字段为：

```text
resolved config
├── task
│   ├── name: string
│   └── task_id: string                 # 每次 START 生成的 UUID
├── planner
│   ├── id: string
│   ├── version: string                 # 精确版本
│   ├── manifest: string                # manifest 绝对路径
│   ├── runtime: {image, adapter, command[]}
│   └── parameters: mapping             # 默认值与任务覆盖合并后的结果
├── vehicles: string[]                  # 有序车辆列表
├── targets: {vehicle_id: {x, y, z}}     # 共享 NED，米
├── completion: {position_tolerance, hold_duration, task_timeout}
├── coordinates: {frame: LOCAL_NED, body_frame: FRD}
├── deployment: {vehicle_id: 部署条目}
└── time: {use_sim_time: true, clock_topic: /clock}
```

framework 将文件路径填入 manifest 的 `runtime.command`，以子进程启动所选 Planner adapter；同时启动 AS2 输出 adapter。两者均通过 `resolved_config` 参数读取同一份配置，并通过独立的 ROS 参数 `use_sim_time` 设置时钟。

暂停时停止的是 ROS 2 Planner adapter 子进程，不是 Compose 中常驻的 ROS 1 算法服务。恢复时重新启动 adapter，重新读取相同任务的 resolved config。

### 2.3 使用者与 framework：控制和结果

`/planner/control` 的服务定义等价于：

```text
Request
  uint8 command                       # START=0 / PAUSE=1 / RESUME=2 / CANCEL=3 / GET_STATUS=4
  string task_config_path             # START 时使用，runtime 容器内路径
  string deployment_config_path       # START 时使用，runtime 容器内路径
---
Response
  bool accepted                       # 本次控制命令是否被接受
  string message                      # 控制响应或拒绝原因
  BehaviorResult result
    string behavior_type              # 当前为 "planner"
    string task_id
    uint8 overall_status              # REJECTED=0…CANCELED=5
    string failure_reason             # NONE、INVALID_TASK、TASK_TIMEOUT 等
    VehicleResult[] vehicle_results
      string vehicle_id
      uint8 status
      string failure_reason
      string detail
```

例如成功接受 START 后，返回值可表示为：

```yaml
accepted: true
message: ok
result:
  behavior_type: planner
  task_id: "示例任务UUID"
  overall_status: 1
  failure_reason: NONE
  vehicle_results: []
```

这里是 `RUNNING`，不是 `SUCCEEDED`。当前 `vehicle_results` 为空；详细错误通常写日志，不能依赖它返回逐机诊断。`/planner/status` 使用同一个 `BehaviorResult` 类型，节点在控制请求和部分终态变化时发布，不保证固定周期推送。

### 2.4 AS2 与上层模块：实际位姿反馈

每架飞机通过部署配置指定的 topic 发布 `geometry_msgs/msg/PoseStamped`：

```text
header
  stamp: {sec, nanosec}
  frame_id: string
pose
  position: {x, y, z}                  # 逐机局部 ENU，米
  orientation: {x, y, z, w}            # 四元数
```

framework 使用 position 加上逐机原点，转换为共享 NED 后执行完成判定；两个 Planner adapter 将它转换为共享 ENU 后交给算法代理；AS2 输出 adapter 从 orientation 提取当前 yaw，供缺省 yaw 时使用。

这些订阅采用 sensor-data QoS，适配 AS2 的 BEST_EFFORT 位姿发布。当前代码依据配置约定坐标，没有根据任意 `frame_id` 自动做 TF 转换，也没有使用消息 stamp 检查位置新鲜度。因此接入新的定位源时，需同时保证坐标与数据时效。

### 2.5 ROS 2 adapter 与 ROS 1 算法代理：TCP/JSON

两个代理均使用 UTF-8 的换行分隔 JSON：每次新建 TCP 连接，发送一行请求（末尾 `\n`），接收一行响应。下面 JSON 为易读而分行展示，在线路上由代码序列化成一行。位置单位为米、时间参数为秒。

#### CAT-ORA：批量规划接口

默认地址 `127.0.0.1:28500`。请求示例（两机，仅说明数据格式，不是四机验收任务）：

```json
{
  "starts_enu": [[0.0, 0.0, 1.5], [0.0, 2.0, 1.5]],
  "goals_enu": [[1.0, 0.0, 1.5], [1.0, 2.0, 1.5]],
  "trajectory_dt": 0.2,
  "max_velocity": 1.2,
  "max_acceleration": 0.6
}
```

起点和终点数组均按 resolved config 的 `vehicles` 顺序排列，坐标为机群共享 ENU。请求中没有 vehicle_id、task_id 或绝对时间戳。

代理将字段映射到 ROS 1 服务 `/mrs_formation_reshaping/get_reshaping_trajectories`，类型为 `mrs_formation_reshaping/SrvGetReshapingTrajectories`：

| JSON 字段 | 原生请求字段 |
| --- | --- |
| `starts_enu` | `initial_configurations`，Point 数组 |
| `goals_enu` | `goal_configurations`，Point 数组 |
| `trajectory_dt` | `trajectory_dt` |
| `max_velocity` | `max_velocity`，米/秒 |
| `max_acceleration` | `max_acceleration`，米/秒² |
| 代理固定值 | `publish_visualization=false` |

原生响应中的 `success`、`message` 和 `trajectories[].waypoints` 被转换为以下 JSON。成功响应结构示例（中间采样点未展示，不能作为实际轨迹执行）：

```json
{
  "ok": true,
  "trajectories_enu": [
    [[0.0, 0.0, 1.5], [1.0, 0.0, 1.5]],
    [[0.0, 2.0, 1.5], [1.0, 2.0, 1.5]]
  ]
}
```

`trajectories_enu[车辆索引][点索引][坐标轴]` 是三层数组，不含速度、加速度、yaw 或逐点时间。adapter 以 `点索引 × trajectory_dt` 补齐时间，较短轨迹重复末点以统一结束时间，再核对终点是否仍属于原车辆目标。

无解时响应 `{"ok": false, "reason": "NO_SOLUTION", "error": "诊断文本"}`；普通异常响应 `{"ok": false, "error": "诊断文本"}`。前者转为统一失败消息，后者当前记录日志并重试，最终可能由轨迹超时收口。

#### Swarm-Formation：状态更新与滚动采样接口

默认地址 `127.0.0.1:28501`。请求必须按顺序包含 `drone0` 至 `drone6`：

```json
{
  "task_id": "示例任务UUID",
  "vehicles": [
    {"id": "drone0", "position_enu": [-12.0, -4.5, 1.5]},
    {"id": "drone1", "position_enu": [-12.0, -1.5, 1.5]},
    {"id": "drone2", "position_enu": [-12.0, 1.5, 1.5]},
    {"id": "drone3", "position_enu": [-12.0, 4.5, 1.5]},
    {"id": "drone4", "position_enu": [-15.0, -3.0, 1.5]},
    {"id": "drone5", "position_enu": [-15.0, 0.0, 1.5]},
    {"id": "drone6", "position_enu": [-15.0, 3.0, 1.5]}
  ],
  "goal_center_enu": [-4.0, 0.0, 1.5],
  "sample_dt": 0.1,
  "horizon": 1.0,
  "lead": 0.2
}
```

`sample_dt` 是采样间隔，`horizon` 是窗口长度，`lead` 是相对代理当前时间的提前量。默认情况下每条轨迹返回 11 个位置点。`update_period=0.2` 控制 ROS 2 adapter 发请求的周期，不是请求里的字段。

当前 adapter 内置 `RELATIVE_ENU` 七机队形：逐机目标减去固定相对偏移后，应得到一致中心（偏差不超过 0.05 米）。请求只传这个中心，不传任意队形偏移数组。因此更换任务目标不能自动定义任意新队形。

代理与 ROS 1 算法之间使用：

| 方向 | Topic / 类型 | 实际处理的数据 |
| --- | --- | --- |
| proxy → 算法 | `/drone_i_visual_slam/odom`，`nav_msgs/Odometry` | 共享 ENU position、当前 ROS 1 stamp、`frame_id=world`；orientation 固定单位四元数，速度未由请求提供 |
| proxy → 算法 | `/traj_start_trigger`，`geometry_msgs/PoseStamped` | 共享 ENU 编队中心，latched 发布 |
| 算法 → proxy | `/drone_i_planning/trajectory`，`traj_utils/PolyTraj` | 读取 `drone_id`、`start_time`、`duration[]`、`order`、`coef_x/y/z[]` 并采样 |
| 算法 → proxy → 算法 | `/broadcast_traj_from_planner` → `/broadcast_traj_to_planner`，`PolyTraj` | 转发机间规划轨迹 |

代理对分段多项式在目标时刻求值，返回与 CAT-ORA 相同的 `{"ok": true, "trajectories_enu": ...}` 三层数组，外层必须是七条轨迹。尚未收齐时返回 `{"ok": false, "reason": "WAITING_FOR_TRAJECTORIES"}`，异常时返回 `{"ok": false, "reason": "ERROR", "error": "诊断文本"}`。

`WAITING_FOR_TRAJECTORIES` 不立即终止任务；持续无有效轨迹由 framework 的超时机制处理。proxy 仅在 task_id 变化时清空缓存并重新发布中心目标。暂停恢复沿用 task_id，因此 adapter 重启不等于 ROS 1 算法完整重置。

时间边界也需要区分：proxy 用 ROS 1 当前时间和 PolyTraj 的 start_time 采样；响应不携带该采样基时间。ROS 2 adapter 收到响应后，用自己的仿真时钟重新设置 `generated_at` 和 `valid_from`。当前协议没有显式进行两端时钟或传输延迟校正。

### 2.6 Planner adapter 与 Gateway：统一轨迹消息

`/planner/trajectory` 使用 `uav_planner_interfaces/msg/TimedTrajectorySegment`。实际 ROS 消息的字段层次为：

```text
uint32 interface_version                 # 当前为 1
string task_id
builtin_interfaces/Time generated_at      # {sec: int32, nanosec: uint32}
builtin_interfaces/Time valid_from        # 全机群共享执行基时间
bool is_final
VehicleTrajectory[] vehicle_trajectories
  string vehicle_id
  uint8 valid_fields                      # 位掩码，不是字符串数组
  TimedTrajectoryPoint[] points
    builtin_interfaces/Duration time_from_start
    geometry_msgs/Point position          # {x, y, z}，共享 NED，米
    geometry_msgs/Vector3 velocity         # {x, y, z}，米/秒
    geometry_msgs/Vector3 acceleration     # {x, y, z}，米/秒²
    float64 yaw                           # 弧度，NED yaw
```

`valid_fields` 的位值为 `VELOCITY=1`、`ACCELERATION=2`、`YAW=4`，组合时按位或，例如 `5` 表示速度和 yaw 有效。位置和相对时间始终必需，不靠位掩码声明。

下面是单机契约示例，方便展示完整字段；多机时必须增加所有参与车辆，不能只发送一个车辆条目：

```yaml
interface_version: 1
task_id: "示例任务UUID"
generated_at: {sec: 100, nanosec: 0}
valid_from: {sec: 103, nanosec: 0}
is_final: true
vehicle_trajectories:
  - vehicle_id: drone0
    valid_fields: 0
    points:
      - time_from_start: {sec: 0, nanosec: 0}
        position: {x: -4.5, y: -12.0, z: -1.5}
        velocity: {x: 0.0, y: 0.0, z: 0.0}
        acceleration: {x: 0.0, y: 0.0, z: 0.0}
        yaw: 0.0
      - time_from_start: {sec: 1, nanosec: 0}
        position: {x: -4.5, y: -11.0, z: -1.5}
        velocity: {x: 0.0, y: 0.0, z: 0.0}
        acceleration: {x: 0.0, y: 0.0, z: 0.0}
        yaw: 0.0
```

两个现有 adapter 都发送 `valid_fields=0`。ROS 消息中可选字段仍物理存在且默认是零，但这些零不能解释为有效的零速度或零 yaw。转换到 Gateway 内部对象后，位掩码变为字段名称集合；未声明的字段不会参与有效数据采样。

轨迹 publisher/subscriber 使用 `RELIABLE + TRANSIENT_LOCAL`、depth 1；迟加入的订阅者可能收到最后一段，所以仍必须检查 task_id、时间覆盖和版本顺序。其它整段校验规则见第 5 节。

### 2.7 Gateway 与 AS2 输出 adapter：采样参考

`/planner/setpoints` 的 `FleetSetpoint` 为：

```text
string task_id
builtin_interfaces/Time sampled_at
VehicleSetpoint[] setpoints
  string vehicle_id
  geometry_msgs/Point position            # 逐机本地 NED，已减去该机原点
  geometry_msgs/Vector3 velocity
  geometry_msgs/Vector3 acceleration
  float64 yaw
```

与轨迹段不同，这是一帧参考，没有未来点数组，也没有 valid_fields；不可信的可选值使用 NaN。按上面的轨迹，在仿真时间 103.5 秒采样，若 drone0 原点为 `[-4.5, -12.0, 0.0]`，输出示例为：

```yaml
task_id: "示例任务UUID"
sampled_at: {sec: 103, nanosec: 500000000}
setpoints:
  - vehicle_id: drone0
    position: {x: 0.0, y: 0.5, z: -1.5}
    velocity: {x: .nan, y: .nan, z: .nan}
    acceleration: {x: .nan, y: .nan, z: .nan}
    yaw: .nan
```

`.nan` 是此处 YAML 的展示写法；ROS 线上是浮点 NaN，不是字符串，也不是标准 JSON 数据。消息 QoS 使用默认可靠、volatile、depth 10。输出 adapter 检查 task_id 和车辆集合，等待全机群控制模式及当前 yaw 就绪后再发布。

### 2.8 AS2 输出 adapter 与执行层：控制模式和运动参考

每架飞机先调用 `/<namespace>/controller/set_control_mode`，服务类型 `as2_msgs/srv/SetControlMode`。当前设置请求中 `control_mode.control_mode=POSITION`、`control_mode.yaw_mode=YAW_ANGLE`，以响应 `success` 判断模式是否就绪。

之后发布两个 ROS topic：

| Topic | 类型 | 填写方式 |
| --- | --- | --- |
| `/<namespace>/motion_reference/pose` | `geometry_msgs/PoseStamped` | header.stamp 使用 sampled_at；frame_id 为 `<namespace>/odom`；position 为本地 ENU；orientation 为 yaw 四元数 |
| `/<namespace>/motion_reference/twist` | `geometry_msgs/TwistStamped` | 同一 header；linear.x/y/z 均写入速度限制；其余分量默认零 |

上例本地 NED `[0, 0.5, -1.5]` 转换后，AS2 的 position 为 `[0.5, 0, 1.5]`。如果 yaw 有效，先转换 `yaw_enu=wrap(π/2-yaw_ned)`，再令四元数 `z=sin(yaw_enu/2)`、`w=cos(yaw_enu/2)`；yaw 为 NaN 则采用最新位姿的当前 yaw。

速度限制读取 `planner.parameters.max_velocity`，缺省为 1.0。当前适配层不把 FleetSetpoint 的 velocity、acceleration 原样传给控制器。AS2 再经控制器和 Pixhawk platform 转换为 PX4 消息，Planner 不直接发布 PX4 `/fmu/in/trajectory_setpoint`。

### 2.9 失败与时钟接口

adapter 发布 `/planner/failure`，类型 `PlannerFailure`：

```yaml
task_id: "示例任务UUID"
reason: INVALID_TRAJECTORY_SET
detail: "Swarm-Formation returned an incomplete fleet trajectory"
```

framework 只处理当前 RUNNING 任务对应的失败消息，并更新状态、停止 Planner adapter、尝试发布当前位置悬停。`reason` 为字符串原因码，`detail` 为日志诊断文本；通用代理通信错误并非都立即转换成该消息，具体行为见 2.5 节。

时钟不通过上述 JSON 代理传递。PX4 容器中的 `gazebo_clock_proxy.py` 读取 Gazebo `/clock`，向 `127.0.0.1:28502` 发送 UDP 包：

```text
struct.pack("!qI", sec, nanosec)
├── sec:     8 字节有符号整数，网络字节序
└── nanosec: 4 字节无符号整数，网络字节序
总长：12 字节
```

ROS 2 时钟桥解包后发布 `/clock`，类型 `rosgraph_msgs/msg/Clock`，字段为 `clock: {sec, nanosec}`。framework 的超时、轨迹时间和位置参考 stamp 采用这套 ROS 2 仿真时间；Shell 验收脚本的截止时间仍是宿主时间。

## 3. 两个算法如何接入

| 项目 | CAT-ORA | Swarm-Formation |
| --- | --- | --- |
| `planner.id` | `cat_ora` | `swarm_formation` |
| 原生运行环境 | ROS 1 Noetic | ROS 1 Noetic |
| ROS 1 接入方式 | 代理调用规划服务 | 代理接收状态与目标，并读取 `PolyTraj` |
| 当前代理端口 | `127.0.0.1:28500` | `127.0.0.1:28501` |
| 输出方式 | 一段有限轨迹，`is_final=true` | 短时滚动段，`is_final=false` |
| manifest 声明机数 | 1–16 | 固定 7 |
| 统一框架历史冷启动验收 | 4 机 | 7 机 |

两边通过 TCP/JSON 显式传递数据，不依赖通用 `ros1_bridge`。这使 ROS 1 算法与 ROS 2 执行环境保持分离；跨机器部署仍需要另外设计网络、代理地址和故障策略，当前 loopback 配置不等于已完成板端部署。

CAT-ORA 原算法可以重新分配目标，但公共任务契约固定“车辆 → 目标”的对应关系。adapter 校验终点归属，换配不符合约定时报告 `NO_SOLUTION`，不会执行换配轨迹。

Swarm adapter 根据内置七机相对偏移校验逐机目标并计算中心，将中心交给代理；代理对原生多项式轨迹采样，adapter 再封装为统一短段。在发布前检查完整机群及首点与实际位置的距离。默认 `max_start_distance=1.5 m`。当前接口不支持通过任务 YAML 任意重定义队形偏移。

两个 adapter 当前都只声明位置有效，`valid_fields=0`。轨迹消息中的速度、加速度和 yaw 是扩展能力，不能据此认为现有算法已提供这些字段。

## 4. 配置为何分成四类

| 配置 | 谁维护 | 内容 |
| --- | --- | --- |
| 任务 YAML | 任务使用者 | 算法及版本、车辆、逐机目标、完成条件、参数覆盖 |
| 部署 YAML | 部署人员 | namespace、位姿 topic、逐机原点 |
| Registry 文件 | 算法集成人员 | manifest、默认参数、参数 schema |
| resolved config | 框架生成 | 本次唯一 task_id、合并参数、部署与运行信息 |

解析顺序为：定位精确版本 → 加载默认参数 → 合并任务覆盖 → schema 与任务校验 → 生成 task_id → 保存 resolved config → 启动 adapters。

映射递归合并，数组整体替换；未知算法参数按 schema 拒绝。`vehicles`、`targets`、部署 YAML 的车辆集合必须完全一致。四机任务不能直接传入包含七机的部署文件。

Registry 校验 manifest 的结构与机数范围，但尚未执行宿主 CPU 架构匹配、资源预留或硬件自动调度。`platforms`、`resources` 和 `runtime.image` 当前主要是声明信息。

以现有 CAT-ORA 四机样例为例：

```yaml
task:
  name: cat_ora_four_px4
planner:
  id: cat_ora
  version: 4bc41f593eebde65dd74108a97e80430a145dfa0
  parameters:
    trajectory_timeout: 30.0
vehicles: [drone0, drone1, drone2, drone3]
targets:
  drone0: {x: -6.0, y: -4.0, z: -1.5}
  drone1: {x: -2.0, y: -4.0, z: -1.5}
  drone2: {x: 2.0, y: -4.0, z: -1.5}
  drone3: {x: 6.0, y: -4.0, z: -1.5}
completion:
  position_tolerance: 0.5
  hold_duration: 1.0
  task_timeout: 120.0
```

`completion` 三个字段必须显式填写。当前 CAT-ORA 示例任务总超时为 120 秒，Swarm 七机示例为 180 秒。

## 5. 轨迹契约、坐标和时间

一条 `TimedTrajectorySegment` 包含：接口版本、任务 id、生成时间、执行基时间、是否最终段，以及所有参与车辆的轨迹。每架飞机包含 vehicle_id、有效字段位掩码和带相对时间的轨迹点。

Gateway 整段检查以下条件：

- 车辆集合完整且无重复；任一车辆无效即拒绝整段。
- 位置和时间为有限数；逐机点时间非负且严格递增。
- 各机最后一个点的相对时间一致。
- 轨迹仍有可执行时间覆盖。
- 同任务生成时间有序；通过有效性检查后，重复版本忽略，倒序版本拒绝。

旧任务消息在节点入口被过滤。当前任务内的无效轨迹会触发 `FAILED/INVALID_TRAJECTORY_SET`，不能理解为“丢掉坏段后继续飞”。

框架默认按 50 Hz 采样并进行线性插值。轨迹基时间之前取首点；轨迹结束后取终点。最终段不再等待后继轨迹；非最终段结束后超过 `trajectory_timeout` 才触发断流失败。任务是否成功独立依据实际位置判定。

坐标转换分两步：

```text
逐机本地 NED = 机群共享 NED - px4_origin_in_fleet_ned
AS2 本地 ENU = (本地 NED.y, 本地 NED.x, -本地 NED.z)
```

任务高度 1.5 米通常写为 `z: -1.5`。修改 Gazebo 出生位置时必须同步更新部署原点。

未声明有效的可选字段在 Gateway 输出中为 NaN。当前 AS2 adapter 只消费位置和可用 yaw；缺省 yaw 使用当前朝向，`motion_reference/twist` 写入速度限制，不传递轨迹速度或加速度前馈。

## 6. 生命周期与完成条件

| 命令 | 值 | 当前行为 |
| --- | ---: | --- |
| START | 0 | 校验任务、生成新 task_id、启动 adapters；不负责起飞 |
| PAUSE | 1 | 停止 Planner adapter、清除旧轨迹、发布悬停参考、冻结任务计时 |
| RESUME | 2 | 沿用 task_id，重启 adapter，从实际位置重新规划 |
| CANCEL | 3 | 清除轨迹、停止 Planner adapter、悬停并进入不可恢复的终态 |
| GET_STATUS | 4 | 获取当前状态 |

状态值为 `REJECTED=0`、`RUNNING=1`、`PAUSED=2`、`SUCCEEDED=3`、`FAILED=4`、`CANCELED=5`。已有活跃任务时再次 START 会被拒绝。服务返回 `accepted=true` 仅表示命令被接受，不代表飞行任务成功。

完成条件是所有参与车辆同时进入各自目标的三维容差，并连续保持 `hold_duration`。任一车辆离开容差则保持计时重置。`task_timeout` 只累计运行期间的仿真时间。

常见失败原因包括 `NO_SOLUTION`、`TRAJECTORY_TIMEOUT`、`TASK_TIMEOUT`、`INVALID_TRAJECTORY_SET`、`PLANNER_ERROR` 和 `OUTPUT_ADAPTER_ERROR`。失败后不会自动 Land 或 Disarm。

当前位置悬停依赖完整位置缓存和可用输出链。当前节点没有独立的位姿新鲜度 watchdog，不能把它当作已覆盖定位丢失、跨机断链和框架崩溃的完整故障保护。

## 7. 使用前准备

以下部署命令在 Linux 仿真主机、工程根目录执行。当前 macOS 文档工作区不是已验证的仿真运行环境。

基础镜像 `uav/px4-gazebo:v1.17.0-harmonic`、`uav/catora:noetic-upstream`、`uav/swarm-formation:noetic-px4` 需按相邻复现工程准备：[CAT-ORA 工程](../../../../03-catora-px4-gazebo/README.md)、[Swarm 工程](../../../../04-swarm-formation-px4-gazebo/README.md)。

随后准备锁定上游源码、构建 AS2 平台及集成镜像：

```bash
bash deployment/build_from_zero.sh
```

该脚本不替代相邻工程的基础镜像构建。`NO_CACHE=1` 当前只传递给平台镜像的 `docker build`；要让集成镜像也无缓存构建，应显式执行：

```bash
NO_CACHE=1 bash deployment/build_from_zero.sh
docker compose -f deployment/integration/compose.yaml build --no-cache
```

目录整理后，Compose 中默认的旧 `swarm-formation-px4-gazebo` 路径尚未改名。若仿真机采用当前编号目录布局，先设置：

```bash
export PX4_SOURCE_DIR="$(cd ../04-swarm-formation-px4-gazebo && pwd)/PX4-Autopilot"
export PX4_SIM_CACHE_DIR="$(cd ../04-swarm-formation-px4-gazebo && pwd)/.px4-sim-cache"
test -f "$PX4_SOURCE_DIR/Makefile"
test -d "$PX4_SIM_CACHE_DIR"
docker compose -f deployment/integration/compose.yaml config --quiet
```

如果源码与缓存位于别处，改为仿真机上的真实绝对路径。目录检查失败时，先完成相邻工程的源码与仿真资源准备；仅创建空目录不能代替准备工作。

## 8. 一键运行现有示例

已有镜像并完成路径设置后，选择一个命令执行：

```bash
bash deployment/integration/run_planner.sh cat_ora
```

```bash
bash deployment/integration/run_planner.sh swarm_formation
```

脚本会停止本工程旧容器，启动六服务，等待控制服务，顺序完成七机起飞，启动所选任务，再轮询到成功或失败。两次验收必须串行运行，每次都会重置仿真。

CAT-ORA 示例使用四机，其余三机不参与任务；Swarm 示例使用七机。成功时输出 `PASS: ... completed with SUCCEEDED/NONE`。脚本成功后不会自动关闭仿真，停止命令为：

```bash
docker compose -f deployment/integration/compose.yaml down
```

## 9. 手动启动与任务控制

需要观察中间过程或使用自定义任务时，先在宿主启动服务：

```bash
docker compose -f deployment/integration/compose.yaml up -d --no-build
docker compose -f deployment/integration/compose.yaml ps
docker exec -it planner-runtime bash
```

接下来的命令均在 `planner-runtime` 容器内执行：

```bash
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /opt/uav_planner_ws/install/setup.bash

fleet_takeoff.py --vehicles drone0 drone1 drone2 drone3 drone4 drone5 drone6 \
  --height 1.5 --speed 0.5 --timeout 90

ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner \
  '{command: 0, task_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/cat_ora_four_px4.yaml, deployment_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/deployment_four_px4.yaml}'
```

Swarm 对应任务文件为 `swarm_formation_seven_px4.yaml`，部署文件为 `deployment_seven_px4.yaml`，均在同一 examples 目录。验收另一算法时退出容器、重置 Compose 并重新起飞。

查询和控制命令：

```bash
# 查询当前任务
ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner '{command: 4}'
# 暂停
ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner '{command: 1}'
# 从当前位置重新规划并恢复
ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner '{command: 2}'
# 取消
ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner '{command: 3}'
```

### 自定义任务

从[现有示例](../../../planner_framework/examples/cat_ora_four_px4.yaml)复制任务，修改目标、完成条件或允许的算法参数；选择车辆集合完全一致的部署文件。容器读取的是容器内路径。

例如在宿主执行：

```bash
docker cp ./my_task.yaml planner-runtime:/tmp/my_task.yaml
docker cp ./my_deployment.yaml planner-runtime:/tmp/my_deployment.yaml
```

然后在已加载环境的容器终端执行：

```bash
ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner \
  '{command: 0, task_config_path: /tmp/my_task.yaml, deployment_config_path: /tmp/my_deployment.yaml}'
```

复制到 `/tmp` 的文件在容器重建后不会保留。修改源码中的 examples 或 Registry 后，如继续使用镜像安装目录里的配置，需要重新构建 runtime 镜像。

## 10. 验证与排障

宿主 `artifacts/resolved/` 挂载到容器 `/var/lib/uav-planner/resolved/`。成功启动的任务生成 `<task_id>.yaml`、`<task_id>.planner.log`、`<task_id>.output.log`。暂停恢复会重新打开同名 Planner 日志，当前实现不保证保留恢复前的完整日志。

| 现象 | 优先检查 |
| --- | --- |
| 服务存在但没有定位数据 | `/clock`、AS2 子节点、QoS、ROS_DOMAIN_ID 和 Fast DDS UDP 设置 |
| `INVALID_TASK` | 车辆、目标、部署集合是否完全一致；版本与参数是否符合 Registry |
| `NO_SOLUTION` | 原算法失败信息，或 CAT-ORA 是否改变目标分配 |
| `TRAJECTORY_TIMEOUT` | proxy 连通性、算法首段耗时、滚动轨迹覆盖时间及仿真时钟 |
| `INVALID_TRAJECTORY_SET` | 机群完整性、点时间、段过期与倒序、Swarm 首点连续性 |
| `TASK_TIMEOUT` | 实际位置误差、共享原点映射、控制模式、任务总时限 |
| 容器是 Up 但任务失败 | ROS 服务、子进程日志和 PX4 实际状态；Up 只证明容器主进程存在 |

常用宿主日志命令：

```bash
docker logs --tail 100 planner-runtime
docker logs --tail 100 planner-cat-ora
docker logs --tail 100 planner-swarm-formation
docker logs --tail 100 planner-as2-seven
docker logs --tail 100 planner-px4-seven
```

运行时镜像构建会执行四组测试，历史记录共 32 项通过。纯框架测试也可以在已安装其 Python 依赖的环境运行：

```bash
python3 -m unittest discover -s planner_framework/tests -v
```

现有动力学冷启动证据使用 `1337ee…` runtime 镜像；最终 `a6abc8…` 镜像修改 manifest 元数据和校验逻辑后重跑了测试与 Registry 解析，不能写成该最终镜像已原样重跑全部冷启动。

## 11. 后续接入新 Planner

新增算法可以沿用以下边界：

1. 在 Registry 中添加精确版本目录、manifest、默认参数和 schema。
2. 实现读取 resolved config 的 ROS 2 adapter，接收实际状态、调用算法，输出统一轨迹或失败原因。
3. 保证车辆集合、坐标、时间、目标归属和暂停恢复语义一致。
4. 将 adapter 加入 runtime 构建；如有独立算法服务，补充其镜像和部署配置。
5. 测试配置拒绝、轨迹校验、断流、暂停恢复与取消，再做实际 PX4/Gazebo 闭环验收。

当前框架没有完整远程 Model Hub，也没有自动下载、硬件匹配或运行位置选择。封装原生 AS2 Behavior 时，可复用现有 Registry、adapter 和 Gateway，但需另行实现 Action 接口及生命周期映射。

## 12. 实现入口

- [Registry 与配置校验](../../../planner_framework/uav_planner_framework/registry.py)
- [ROS 节点与进程管理](../../../planner_framework/uav_planner_framework/ros_node.py)
- [轨迹契约与 Output Gateway](../../../planner_framework/uav_planner_framework/trajectory.py)
- [完成判定](../../../planner_framework/uav_planner_framework/completion.py)
- [CAT-ORA adapter](../../../cat_ora_adapter/uav_cat_ora_adapter/node.py)、[ROS 1 proxy](../../../adapters/cat_ora/ros1_proxy.py)
- [Swarm adapter](../../../swarm_formation_adapter/uav_swarm_formation_adapter/node.py)、[ROS 1 proxy](../../../adapters/swarm_formation/ros1_proxy.py)
- [AS2 setpoint adapter](../../../as2_setpoint_adapter/uav_as2_setpoint_adapter/node.py)
- [一键验收脚本](../../../deployment/integration/run_planner.sh)
- [详细接口设计](../../Aerostack2统一Planner第一阶段接口与配置设计.md)、[完整操作指南](../../Aerostack2统一Planner使用指南.md)

[返回文档导航](../../README.md)
