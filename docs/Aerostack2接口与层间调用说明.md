# Aerostack2 原生接口与层间调用说明

本文根据 Aerostack2 官方架构文档和本项目锁定的 AS2 源码整理，说明原生 AS2 各层的职责、调用方向和主要 ROS 2 接口。本文只描述原生 AS2；本项目的 Planner、适配器和 `FollowTimedPath` 扩展见[统一 Planner 第一阶段接口与配置设计](Aerostack2统一Planner第一阶段接口与配置设计.md)。

## 1. 原生分层

```text
Application / Mission Control
        │  Python API、Behavior Tree、Behavior Action
        ▼
Behaviors
        │  Motion Reference、状态查询、生命周期反馈
        ▼
Motion Controller        State Estimator
        │  actuator commands       ▲ sensor measurements
        ▼                           │
Aerial Platform ────────────────┘
        │
        ▼
真实飞行器、PX4、DJI 或仿真平台
```

官方架构还把 ROS 2 消息定义称为进程间通信层，把应急通道作为所有模块都可访问的公共通道。上述组件通常是并行运行的：Behavior 被调用时，状态估计、控制器和平台接口不会因此停止。

## 2. Mission/Application 到 Behavior

Mission Control 不直接生成电机命令。它通过 Behavior 的 Action Client、Python API 或行为树节点提交一个动作目标，Behavior Server 负责动作的生命周期和执行反馈。

常见调用关系如下：

```text
Python API / Behavior Tree
        │ Action Goal
        ▼
Behavior Server
        ├─ Feedback：执行进度或当前状态
        ├─ Result：成功、失败或取消结果
        └─ 状态控制：pause、resume、modify、cancel（按 Behavior 支持情况提供）
```

典型原生运动 Action 包括 `as2_msgs/action/Takeoff`、`GoToWaypoint`、`Land` 和 `FollowPath`。不同 Behavior 的 Goal 字段不同，不能把所有 Behavior 当成同一种通用 Action；应以对应的 `as2_msgs/action/*.action` 定义和插件实现为准。

Behavior 内部一般使用 `as2_behavior::BehaviorServer<ActionT>` 管理目标、状态、反馈和结束条件，再调用具体插件完成动作。Behavior 可以是运动控制、轨迹生成、路径规划或感知能力，官方并未把规划能力排除在 Behavior 之外。

## 3. Behavior 到 Motion Reference

Behavior 将动作目标转换为运动参考，发布给 Motion Controller。AS2 的运动参考按控制模式使用不同消息：

| 参考 | 典型内容 | 用途 |
| --- | --- | --- |
| Position | 位置、姿态/航向 | 位置或航向控制 |
| Twist | 线速度、角速度 | 速度或速度加航向速度控制 |
| Trajectory | 时间化位置、速度等 | 轨迹控制 |

主题命名空间通常为：

```text
/<drone_namespace>/motion_reference/pose
/<drone_namespace>/motion_reference/twist
/<drone_namespace>/motion_reference/trajectory
```

具体 Behavior 不一定直接创建 ROS Publisher；例如原生运动 Behavior 使用 `PositionMotion` 等 Motion Reference Handler，把目标和控制模式封装后发布。参考消息必须带时间戳和坐标系，控制器负责按配置转换到所需坐标系。

## 4. Motion Controller 到 Aerial Platform

Motion Controller 读取运动参考和当前状态，运行配置好的控制插件，生成平台可以执行的指令：

```text
motion_reference/* + self_localization/*
              ↓
Motion Controller / controller plugin
              ↓
/<drone_namespace>/actuator_commands/*
```

官方定义的 actuator command 类型包括 position、velocity 和 thrust；姿态模式把姿态放在 position command 中、把推力放在 thrust command 中，Acro 模式把角速度放在 velocity command 中并配合 thrust。控制模式决定实际使用哪些字段。

因此，Behavior 通常不需要知道 PX4 或 DJI 的协议，也不直接控制电机。它表达“要达到什么运动”，Motion Controller 决定“如何由误差生成控制量”。

## 5. Aerial Platform 到飞控和传感器

Aerial Platform 是 AS2 与具体飞行器或仿真平台之间的适配层：接收 Motion Controller 的 actuator command，将其转换为 PX4、DJI 等平台接口；同时读取平台反馈并发布到 AS2。

平台向 AS2 的标准数据路径包括：

```text
平台传感器 / 飞控反馈
        ↓
/<drone_namespace>/sensor_measurements/<sensor_name>
        ↓
State Estimator
        ↓
/<drone_namespace>/self_localization/pose
/<drone_namespace>/self_localization/twist
```

官方 Pixhawk 接口示例包括：

| 数据 | ROS 2 类型 |
| --- | --- |
| `sensor_measurements/odom` | `nav_msgs/msg/Odometry` |
| `sensor_measurements/imu` | `sensor_msgs/msg/Imu` |
| `sensor_measurements/battery` | `sensor_msgs/msg/BatteryState` |
| `sensor_measurements/gps` | `sensor_msgs/msg/NavSatFix` |

平台适配器提供的是统一接口，不保证每个字段都是传感器芯片未经处理的原始值。相机、激光雷达等外部传感器可以通过各自驱动接入；是否经过飞控、是否参与飞控内部融合，取决于平台和部署配置。

## 6. State Estimator 的输入和输出

State Estimator 使用 `sensor_measurements/*` 和可能的外部定位数据，融合或整理出 AS2 统一的机器人状态。它的输出主要是位姿、速度和 TF 坐标变换，供 Motion Controller、Behavior 和应用使用。

```text
sensor_measurements/imu、odom、gps、外部视觉里程计……
                              ↓
                       State Estimator plugin
                              ↓
                 self_localization/pose、twist 和 TF
```

平台内部已经运行 PX4 EKF 等估计器时，AS2 可以把平台输出的里程计作为输入，也可以根据配置接入外部定位。AS2 State Estimator 与 PX4 内部估计器是不同组件；是否重复融合必须根据具体插件和系统配置判断。

## 7. 应急通道

Emergency 是跨模块的 ROS 2 通道。任意相关模块可以发布告警或应急消息，其他模块根据严重级别和自身职责响应。它不替代 Behavior 的 Action Result，也不自动决定完整任务恢复流程；任务级恢复通常由 Mission Control 编排。

## 8. 一个原生执行例子

以 Python 应用请求“起飞到 1.5 米”为例：

1. 应用创建 `Takeoff` Action Client，并发送高度、航向等 Goal。
2. `TakeoffBehavior` 接收 Goal，选择已配置的起飞插件并进入运行状态。
3. Behavior 通过 Motion Reference Handler 发布目标位置和速度限制。
4. Motion Controller 读取目标参考和 State Estimator 的当前位置，计算控制输出。
5. Aerial Platform 将控制输出转换成 PX4/DJI 或仿真平台指令。
6. 平台返回 IMU、里程计等数据；State Estimator 更新状态并发布位姿和速度。
7. Behavior 根据当前位置、误差、超时和平台状态判断是否完成，向应用发送 Feedback 和最终 Result。

```text
Takeoff Goal
   → Takeoff Behavior
   → Motion Reference
   → Motion Controller
   → Aerial Platform
   → 飞控/仿真
   → 传感器反馈
   → State Estimator
   └────────────────────→ Behavior 完成判定
```

## 9. 接口边界总结

| 上层 | 下层 | 主要接口 | 传递的语义 |
| --- | --- | --- | --- |
| Application/Mission | Behavior | ROS 2 Action、Python API、行为树节点 | 动作目标、反馈、结果、取消 |
| Behavior | Motion Controller | Motion Reference topics | 位置、速度、轨迹等运动意图 |
| State Estimator | Behavior/Controller | 位姿、速度、TF、状态 topics | 当前状态和坐标变换 |
| Motion Controller | Platform | Actuator command topics | 平台可执行的控制指令 |
| Platform | State Estimator | Sensor measurement topics | IMU、里程计、GPS、电池等测量 |
| 所有模块 | 所有相关模块 | Emergency topic | 告警和应急信息 |

最重要的调用边界是：**Mission/Application 调用 Behavior，Behavior 发布运动参考，Motion Controller 生成执行命令，Platform 负责具体飞行器接口；传感器反馈经过 State Estimator 返回上层。** AS2 原生架构中的 Planner 如果以 Behavior 形式存在，也应遵循这条链路，而不是绕过 Behavior 直接向平台发送控制命令。

## 10. 依据

- [官方 Architecture](https://aerostack2.github.io/_01_aerostack2_concepts/architecture/index.html)
- [官方进程间通信](https://aerostack2.github.io/_01_aerostack2_concepts/as2_communications/index.html)
- [官方 Behaviors 概念](https://aerostack2.github.io/_01_aerostack2_concepts/behaviors/index.html)
- [官方 Motion Controller](https://aerostack2.github.io/_01_aerostack2_concepts/motion_controller/index.html)
- [官方 Pixhawk 平台接口](https://aerostack2.github.io/_03_aerial_platforms/_pixhawk/index.html)
