# EGO Planner v2 → AeroHub / AS2

独立 ROS 2 包 `uav_ego_planner_adapter`，复用 AeroHub 公共 `TimedTrajectorySegment`、`FleetFollowPathClient` 及扩展 `FollowPath/FollowTimedPath`。算法仍在 native，Adapter 不发布 Motion Reference，不取终点替代完整轨迹。

## 每机实例入口（2026-11-13）

同一份完整机群配置可显式选择一架飞机；每次 launch 仅启动该机原生 EGO 和该机 Adapter：

```bash
ros2 launch uav_ego_planner_adapter ego_as2.launch.py resolved_config:=/path/fleet.yaml vehicle:=drone0
ros2 launch uav_ego_planner_adapter ego_as2.launch.py resolved_config:=/path/fleet.yaml vehicle:=drone1
```

`vehicle` 是配置 `vehicles` 中的键；ROS 前缀来自对应 `deployment[vehicle].namespace`，不根据列表下标重新编号。原生 launch 仍要求 namespace 为 `drone<planner_id>`，且完整配置中的 ID 与 namespace 必须唯一。未知车辆会在启动前报错。

| 接口 | 每机模式（以 drone0 为例） | 不传 vehicle 的历史入口 |
| --- | --- | --- |
| 目标 GoalSet | `/drone0/ego/goal` | `/ego/goal` |
| 状态 String(JSON) | `/drone0/ego/status` | `/ego/status` |
| 取消 Trigger | `/drone0/ego/cancel` | `/ego/cancel` |
| 输出段 TimedTrajectorySegment | `/drone0/planner/trajectory` | `/planner/trajectory` |
| 失败 PlannerFailure | `/drone0/planner/failure` | `/planner/failure` |
| 原生输入 goal/odom/points | `/drone0/ego/native/{goal,odom,points}` | `/ego/<vehicle>/{goal,odom,points}` |
| 原生轨迹/心跳/停止 | `/drone0/ego_planner/...` | 相同 |
| AS2 执行/修改/停止 | `/drone0/follow_timed_path/...` | 相同 |

每机模式的 `FleetFollowPathClient` 只收到本机投影配置，提交的 segment 只有本机；取消、错误、完成、revision 和目标世代均为本实例所有。不会等待另一机目标，也不会取消另一机行为。状态附带 `vehicles` 与 `interface_scope`；已有状态字段和消息 ABI 不变。GoalSet 仍必须携带本机 planner ID，投错端点的目标会拒绝；没有自动桥接旧全局端点。

完整机群元数据另行只读保留。开启 `peer_body_mask_radius_m` 时，Adapter 订阅邻机原始 navigation/odom，用已知原点和原时间戳过滤邻机机体点云；不向邻机转发规划输入、不创建邻机执行客户端。邻机原始定位失效仍可能使本机点云过滤拒绝输入，继而触发本机输入超时，这是已有安全约束，不宣称所有传感器故障完全互不影响。

原生 `/ego_swarm/trajectory` MINCO 广播继续在同一 fleet/domain 共享，用于邻机避让；`/clock` 和 TF 仍遵循部署层的权威来源。每机模式不再提供集中式 Adapter 的公共交接时刻或整组取消，集体目标发布、整组停止与故障策略由任务/编排层显式实现。本次不新增此编排层，也不修改广播的时间和坐标语义。

不传 `vehicle` 保留集中式兼容入口；显式 vehicle 模式已在RoboPi单机回归。
不能同时给同一车辆启动两种实例。接口/冻结尾段测试在 `../tests/`，独立回归入口为
`deployment/airsim/demos/ego_planner_v2/tests/verify_terminal_hold.sh`。
真实板端、历史PC飞行与当前整理检查分别见[Demo验证摘要](../../../../../deployment/airsim/demos/ego_planner_v2/VALIDATION.md)。

## 运行边界

```text
navigation/odom + navigation/points
  → EGO 输入坐标适配 → 原生 Humble EGO
  → PolyTraj 连续约束校验 → TimedTrajectorySegment
  → AeroHub FleetFollowPathClient
  → 各机 FollowTimedPath Action / modify
  → 扩展 FollowPath → AS2 Motion Reference
```

平台、状态估计、注册点云及原生起飞另由部署层提供。本入口启动 Planner 与 Adapter；每机需已有扩展 FollowPath 宿主。收到真实 AS2 的 FLYING 状态及有效定位后才可能接受执行 Goal。当前工作区验证使用合成状态/点云和控制器替身，不自动启动 AirSim、PX4 或飞行。

## 输入契约

| 接口 | 内容 |
| --- | --- |
| `/<namespace>/navigation/odom` | pose 在固定 `<namespace>/odom` ENU；twist 在 `<namespace>/base_link` FLU；有效单位四元数、有限位置/速度、单调新鲜时间 |
| `/<namespace>/navigation/points` | 已注册到同一固定局部 ENU 的 XYZ 点云；无需再次旋转传感器外参，不伪造 ring/逐点时间 |
| `/ego/goal` | `quadrotor_msgs/GoalSet`；目标为全局 ENU，drone_id 由配置映射；ROS reliable/transient-local |
| `/ego/cancel` | Trigger：锁定当前任务并请求整组停止；结果确认见 `/ego/status`，新任务须重启 Adapter |

配置中的 `px4_origin_in_fleet_ned` 来自 AeroHub 部署约定。Adapter 对 odometry pose 和注册点云仅做已知固定原点平移，保留采样时间。Odometry 的 body twist、child_frame_id 和协方差保持标准语义；AS2 启动入口显式设置 native 的 `fsm.odom_twist_in_body_frame=true`，由真实 FSM 用同一里程计姿态将速度转为世界系。native 单独启动仍默认接收历史 world twist，避免破坏原对照接口。

框架共用轨迹格式为 fleet NED，Adapter 将原全局 ENU 的 PVA 显式转换为 NED，再由共用客户端转换至每机固定局部 ENU。不得把本例原点当作真实平台标定；实际数据源版本、时间映射和地图配准仍需仿真验收。

## 时间、约束与执行

每次给整个机群选择一个未来交接时刻，在**原 PolyTraj 的绝对时间函数**上取值，裁掉已经过去或无法及时交付的前缀，不平移源轨迹时间。采样保留分段边界、PVA 和纳秒级严格递增时间；端点与导数由同一多项式计算。AS2 使用已有五次 Hermite 插值执行。

全部车辆轨迹都到达其目标且端点 PVA 静止时才标记为 final；较早结束的车辆显式保持终点，让整个机群共享完成时刻。一般滚动段只提交各车共同覆盖的时域。邻机 MINCO 广播仍由原生节点直接处理。

Adapter 在转换前独立核查每段多项式的速度/加速度极值，拒绝格式错误、非有限、错误车辆、旧目标世代及超限输出；接受边界与 native 的配置及原 5% 可行性容差一致。Launch 核对两侧约束参数一致。当前示例阈值为 0.2415 m/s、0.3675 m/s²，未放宽此前门槛。

执行复用共用客户端的 Action、modify、Feedback、Result、取消和停止确认。输入/heartbeat 过期、无完整计划、执行拒绝、时钟倒退或超限时，取消本实例所管理的车辆并发送 mandatory_stop；历史入口是整组，每机入口只有所选车辆。状态中的 `stop_confirmed` 需要所管理车辆停止服务成功及控制器 HOVER/POSITION 回报；只有完成结果与停止确认齐全才报告 SUCCEEDED。此包未实现任务重启恢复、上层任务调度、Mission/Tool 或 Model Hub 组件注册。


构建、安装及运行命令统一见[Demo说明](../../../../../deployment/airsim/demos/ego_planner_v2/README.md)。
