# Aerostack2 架构介绍：从起飞、到点到降落

我们用一架无人机执行一次完整任务，沿着请求和反馈的流向理解原生 Aerostack2：

> 在无障碍区域，从地面起飞到 1.5 米，保持高度向东飞 5 米，然后原地降落。

这里的动作逻辑依据上游真实插件实现，任务数值用于讲解，不是本次实际飞行的验收记录。源码基准为 `19c397309c6a41b279e6b551818ec12605cfcecc`。本文只介绍原生架构；项目扩展集中在[统一 Planner 主文档](Aerostack2统一Planner第一阶段接口与配置设计.md)。

## 1. 先把场景搭起来：飞机还没动，哪些程序已经在运行？

选择一条具体执行路线：Python 任务应用调用原生运动 Behavior，AS2 通过 Pixhawk platform 连接 PX4 SITL，Gazebo 模拟飞机运动。真机时可以将 SITL 和 Gazebo 换成实际飞控与机体，但仍需重新验证平台配置。

飞机名为 `drone0`。为便于计算，设用于任务说明的 ENU 参考系原点位于起飞地面，x 朝东、y 朝北、z 朝上，初始位置为 `(0, 0, 0)`。目标是 `(5, 0, 1.5)`。实际部署必须使用存在的参考系和 TF，不应直接假定所有平台原点都在地面。

在发送任务之前，下面这些角色已经就绪：

| 角色 | 本例选用的实现 | 正在等待或处理什么 |
| --- | --- | --- |
| 任务应用 | Python 应用，可以借助 `as2_python_api` | 等待状态就绪，准备按顺序调用动作 |
| 起飞、到点、降落服务端 | 原生运动 Behavior | 等待各自的 Action Goal |
| 行为的具体执行策略 | `takeoff_plugin_position`、`go_to_plugin_position`、`land_plugin_speed` | 决定各个动作如何发参考、如何判断完成 |
| 运动控制器 | `as2_motion_controller` 与配置的控制插件 | 接收参考和实际状态，输出平台支持的控制命令 |
| 状态估计 | `as2_state_estimator`，例如 `raw_odometry` | 将平台里程计组织成 AS2 定位输出和坐标关系 |
| 平台适配 | `as2_platform_pixhawk` | 对接 PX4 状态、模式和控制消息 |
| 飞控与物理世界 | PX4 SITL + Gazebo | 执行飞控算法、模拟机体及传感器 |

**这些角色是并行运行的。** 不是先运行 Behavior、关闭它，再启动控制器。任务进入 GoTo 阶段时，定位、控制和飞控循环仍在持续运行。

## 2. 任务应用先准备飞行，再发起 Takeoff

任务程序负责顺序：只有起飞成功，才允许进入到点；只有到点成功，才进入降落。它可以写成下面的概念流程。此处是伪代码，不是某一版本 Python API 的可复制调用签名：

```text
等待平台连接、定位和所需服务就绪
按平台要求完成解锁与外部控制模式准备，并确认实际状态

发送 Takeoff(height=1.5, speed=0.5)
等待 Action 结束并检查 takeoff_success
若失败：进入预先定义的处置分支，不启动 GoTo

发送 GoTo(target=(5, 0, 1.5), max_speed=1.0, yaw=保持当前朝向)
等待 Action 结束并检查 go_to_success
若失败：进入预先定义的处置分支，不盲目继续

发送 Land(下降速度参数)
等待 Action 结束并检查 land_success
确认平台实际落地状态，按平台流程完成解除武装
```

解锁和进入 Offboard 是平台操作，不应一概当成所有平台通用的同一种 Behavior。PX4 的 Offboard 还要求有效外部参考持续存在，准备顺序由平台实现和部署流程决定。一次服务返回成功，也不等于飞控已经进入期望状态。

应用现在向 Takeoff Behavior 发出 `as2_msgs/action/Takeoff` Goal：

```yaml
takeoff_height: 1.5
takeoff_speed: 0.5
```

这条 Goal 的意思是“完成一次起飞动作”，并不是“立即把飞机高度改成 1.5 米”。Action 首先告诉调用者目标是否被接受，执行期间继续返回 Feedback，最后返回 Result。

Python API 的作用是在应用端封装这些调用。它不是新的控制器，也不是必须穿过的服务端；应用也可以直接使用 ROS 2 Action client。

## 3. Takeoff Behavior 把任务目标变成运动参考

Takeoff Behavior 使用 `BehaviorServer` 管理目标与执行状态，具体起飞策略由插件实现。本例的 `takeoff_plugin_position` 在激活时读取实际位置和朝向：

```text
当前位置 = (0, 0, 0)
目标位置 = (当前 x, 当前 y, 当前 z + takeoff_height)
         = (0, 0, 1.5)
目标 yaw = 当前 yaw
```

因此，这个插件的 `takeoff_height` 是相对激活时高度的增量，而不是任意世界参考系中的绝对 z。

在后续运行周期中，插件调用 `PositionMotion.sendPositionCommandWithYawAngle(...)`，传入目标位置、朝向和速度限制。

`PositionMotion` 属于 `as2_motion_reference_handlers`，是 Behavior 使用的辅助对象，**不一定是单独运行的节点**。它封装运动参考发布与控制模式处理。对于这里的位置命令，参考通过 `motion_reference/pose`、`motion_reference/twist` 等接口送给控制器；Twist 在这一位置调用路径中承担速度限制的作用。

到这一步，Behavior 已经把“起飞 1.5 米”转成了具体的运动意图，但飞机尚需靠后面的控制闭环真正升高。

## 4. 控制器、平台与 PX4 让这个参考变成实际运动

控制器读到两组不同的数据：

```text
期望位置：(0, 0, 1.5)     ← Behavior 发布的运动参考
实际位置：(0, 0, 0.2)     ← State Estimator 返回的定位
```

控制器根据控制模式和插件，将参考与反馈转换为平台可接受的命令。使用位置到速度的控制路径时，它可以根据位置误差产生速度命令；使用其他插件或 bypass 模式时，分工会不同。不能简单理解为“AS2 总是计算全部电机控制”。

`as2_platform_pixhawk` 接到命令后，处理 AS2 与 PX4 的消息、坐标及模式差异，将其送给 PX4。例如 AS2 常用 ENU/FLU，而 PX4 常用 NED/FRD。平台不能把两者的同名 x、y、z 分量直接照抄。

PX4 根据收到的控制级别执行剩余飞控闭环。在 SITL 中，飞控输出驱动 Gazebo 中的机体，Gazebo 模拟运动和传感器；真机上对应的是飞控、执行器和真实机体。

这段关系可以画成一条简单链：

```text
Takeoff Behavior 内的位置参考处理器
        │ 目标位置、yaw、速度限制
        ▼
Motion Controller
        │ 当前模式下的平台命令
        ▼
Pixhawk Platform
        │ PX4 消息及模式适配
        ▼
PX4 → Gazebo 中的机体
```

`motion_reference` 表达“期望怎么运动”，`actuator_command` 是控制器送往平台的命令接口族。后者虽然名称含 actuator，也不意味着总是电机 PWM。

## 5. 实际高度返回后，谁判断“起飞完成”？

飞机上升后，新的实际状态沿另一条路径返回：

```text
Gazebo 的运动与传感器反馈 → PX4 状态估计
        ↓
Pixhawk Platform 发布里程计等测量
        ↓
State Estimator（本例可使用 raw_odometry）
        ↓
self_localization/pose、速度及 TF
        ├── Motion Controller：继续修正运动
        └── Takeoff Behavior：更新反馈并判断是否完成
```

`raw_odometry` 接入已有的里程计结果，不意味着 AS2 又运行了一套视觉 SLAM。若换成动捕、外部视觉里程计或真值数据，需要配置对应来源、插件与坐标关系。

本例的起飞插件有定位信息后，检查目标起飞高度与反馈中的实际起飞高度之差是否小于 `takeoff_threshold`。满足条件时填写 `takeoff_success=true` 并结束 Action；成功收尾时，它还会尝试发送最终位置保持参考。

应用在执行期间看到的反馈包括 `actual_takeoff_height` 和 `actual_takeoff_speed`。例如高度从 0.2 米、0.8 米逐渐接近 1.5 米；这些数字是说明性示例，实际反馈来自定位。

所以这里同时有三件不同的事：

- **控制器**持续让飞机跟踪参考；
- **Behavior**根据自己的完成条件决定这个动作何时结束；
- **任务应用**看到结果后决定下一步做什么。

Action 目标被接受、位置参考被发布、动作执行成功，是三个不同阶段。

## 6. 起飞完成后，向东飞 5 米发生了什么？

应用收到起飞成功结果，向 GoTo Behavior 发送 `as2_msgs/action/GoToWaypoint` Goal。它包含：

| 字段 | 本例含义 |
| --- | --- |
| `target_pose` | 带 frame_id 和时间戳的 PointStamped，目标为 `(5, 0, 1.5)` |
| `max_speed` | 1.0 米/秒 |
| `yaw` | 使用 KEEP_YAW，保留当前朝向 |

目标必须明确属于哪个坐标系，并具备到行为所用参考系的变换。本文的示例原点是为解释数值所作的约定，不是一个自动存在的 TF frame。

本例选用的 `go_to_plugin_position` 会计算所需 yaw，随后周期调用位置参考处理器，发送目标位置及最大速度。控制器、平台和 PX4 继续执行刚才那条闭环，只是参考从 `(0, 0, 1.5)` 换成 `(5, 0, 1.5)`。

**这个具体 GoTo 插件没有先调用避障路径规划器，也没有必然生成一条完整的时间化轨迹。** 它直接发送目标位置，底层控制链使无人机趋近目标。所以本例选择无障碍区域。此前将 GoTo 泛化描述为“自动生成路径或调用规划”并不适用于这条具体实现。

执行过程中，GoTo Action 返回 `actual_distance_to_goal` 和 `actual_speed`。例如实际位置到达 `(4.8, 0, 1.5)` 时，几何距离约为 0.2 米；是否算完成还取决于配置的 `go_to_threshold`。插件在有定位且距离小于阈值时返回 `go_to_success=true`。

这里不应套用其他项目的“保持 1 秒”“全机群同时满足容差”等条件；原生动作按自身实现判断完成。

## 7. 最后降落：同一个框架可以执行另一种参考

应用收到 GoTo 成功结果，再调用 Land Behavior。本例的 `land_plugin_speed` 采用速度策略，与前两个位置策略不同：

1. 激活时记录初始高度，并准备降落相关判据。
2. 运行时通过 `SpeedMotion` 发布垂直下降速度参考，水平速度和 yaw 速度为零。
3. 控制器和平台按当前速度模式执行，实际位置和速度继续反馈。
4. 插件根据下降高度、实际垂直速度与持续时间判据判断动作结束。

在核对版本中，该插件要求从初始位置下降超过 `land_condition_height`，实际降落速度低于由 `land_speed_condition_percentage` 得到的阈值，并持续超过 `land_position_condition_time`。满足后返回 `land_success=true`。

因此，**不能把这个插件的结束判定简化为“PX4 报告落地，Behavior 就结束”**。实际任务仍应核对平台落地与解锁状态，再按部署策略收尾。也不能仅因 Land Action 成功，就假定所有平台都已自动解除武装。

到此为止，应用完成的是三个有结果的动作，而底层状态估计、控制器和平台始终提供共同的执行基础。换一种 Land 插件可以改变降落策略，而不必让任务应用重新实现控制循环。

## 8. 回过头看：这次任务为什么需要这些层？

这次执行可以归纳为两种同时发生的循环：

```text
较慢的任务循环：
发起动作 → 接收 Feedback → 检查 Result → 决定下一个动作

持续运行的运动闭环：
运动参考 → 控制器 → 平台 → 飞控/机体 → 实际状态 → 控制器
                                          └→ Behavior 的完成判定
```

应用不必知道位置误差如何变成飞控指令，Behavior 不必为每种飞控重新写一套协议，控制器不必决定任务先起飞还是先到点。分层的价值就在于把这些变化隔离开。

| 任务里看见的行为 | 对应的架构职责 |
| --- | --- |
| “起飞成功后向东飞” | 应用或行为树负责任务顺序 |
| “起飞到 1.5 米，持续返回高度” | Behavior 管理动作及其结果 |
| “发布目标位置、yaw 和限速” | Motion Reference Handler 封装运动参考接口 |
| “根据实际状态跟踪参考” | Motion Controller 与插件实现控制策略 |
| “把命令翻译成 PX4 消息” | Aerial Platform 隔离平台协议 |
| “把 PX4 里程计提供给上层” | State Estimator 组织定位数据和 TF |
| “真实机体或仿真机体发生运动” | 飞控与物理/仿真后端完成执行 |

这些职责依靠 `as2_msgs`、标准 ROS 消息以及 `as2_core` 的公共设施协作。模块划分是职责边界，不等于每个名称都对应一个独立进程。

## 9. 如果任务改变，架构在哪一处变化？

### 路上有障碍物

刚才的位置 GoTo 不够了。应用需要加入能够利用地图或感知的规划能力，先得到可行路径，再交给兼容的路径跟随或轨迹执行接口。原生上游提供路径规划和轨迹生成相关 Behavior，但需要明确选择并配置；不会因为发出了 GoTo Goal，就自动启用全部规划插件。

规划得到路径、轨迹生成把路径变成带时间的参考、控制器跟踪参考，是不同职责。已有时间化轨迹是否需要再经过轨迹生成，要看执行接口，不能把它们机械地全部串联。

### 想让 Agent 发起任务

Agent 可以接替最上方应用的决策角色，通过经过校验的工具接口调用起飞、到点、降落，并读取动作反馈与结果。Python API 或 ROS Action client 可以作为工具封装基础。

原生行为树本来就在 Behavior 上方提供任务编排；并非原生框架只有 Behavior 以下的部分。Agent 则是利用这些接口构建的上层应用，不是原生框架自动附带的大模型模块。目标检查、失败分支和动作互斥需要在任务系统中明确实现。

### 想换仿真后端或换成真机

本例使用 Pixhawk platform → PX4 SITL → Gazebo，也可以选择原生 Gazebo 平台或轻量多旋翼仿真平台。替换点主要是平台、状态来源与后端配置。上层动作接口可复用，但平台能力、坐标、参数和保护逻辑必须重新验证。

### 从一架变成多架

可以为 `drone0`、`drone1` 等 namespace 各部署执行链，再由上层协调任务。namespace 解决命名隔离；同步、避碰和共享坐标原点不是自动获得的能力。多机规划及其与现有工程的关系见[统一 Planner 主文档](Aerostack2统一Planner第一阶段接口与配置设计.md)。

## 10. 对照源码阅读这次执行

以下链接固定在本文核对版本。建议按 Takeoff 插件 → GoTo 插件 → Land 插件 → Behavior Server 的顺序阅读：先看每个动作实际做什么，再看公共生命周期如何承载它。

- [Takeoff 位置插件](https://github.com/aerostack2/aerostack2/blob/19c397309c6a41b279e6b551818ec12605cfcecc/as2_behaviors/as2_behaviors_motion/takeoff_behavior/plugins/takeoff_plugin_position.cpp)
- [GoTo 位置插件](https://github.com/aerostack2/aerostack2/blob/19c397309c6a41b279e6b551818ec12605cfcecc/as2_behaviors/as2_behaviors_motion/go_to_behavior/plugins/go_to_plugin_position.cpp)
- [Land 速度插件](https://github.com/aerostack2/aerostack2/blob/19c397309c6a41b279e6b551818ec12605cfcecc/as2_behaviors/as2_behaviors_motion/land_behavior/plugins/land_plugin_speed.cpp)
- [Behavior Server 实现](https://github.com/aerostack2/aerostack2/blob/19c397309c6a41b279e6b551818ec12605cfcecc/as2_behaviors/as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp)
- [原生 Action 与消息定义](https://github.com/aerostack2/aerostack2/tree/19c397309c6a41b279e6b551818ec12605cfcecc/as2_msgs)
- [运动参考辅助库](https://github.com/aerostack2/aerostack2/tree/19c397309c6a41b279e6b551818ec12605cfcecc/as2_motion_reference_handlers)
- [运动控制器](https://github.com/aerostack2/aerostack2/tree/19c397309c6a41b279e6b551818ec12605cfcecc/as2_motion_controller)
- [Python API](https://github.com/aerostack2/aerostack2/tree/19c397309c6a41b279e6b551818ec12605cfcecc/as2_python_api)
- [行为树](https://github.com/aerostack2/aerostack2/tree/19c397309c6a41b279e6b551818ec12605cfcecc/as2_behavior_tree)
- [Pixhawk 平台](https://github.com/aerostack2/as2_platform_pixhawk)

[返回文档导航](README.md)
