# EGO-Planner-v2 接入

EGO 原生规划器输出五阶多项式，Adapter 保留原执行时刻、位置、速度和加速度，转换为
TimedTrajectorySegment，经已有 FollowTimedPath 扩展、AS2 控制器/平台执行。起飞与降落
由工作站任务单独调用原生 Takeoff/Land；Adapter 不直接发飞控设定点，也不负责自动降落。

## 源码与边界

- `ros2_adapter/native/`：六个实际构建包，来自 EGO-Planner-v2 提交
  `9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`，保留 GPL-3.0 许可证。
  ROS 1→Humble 移植、地图计数与话题参数化、首轮规划状态和轨迹可行性修复直接维护于此。
- `ros2_adapter/bridge/`：`uav_ego_planner_adapter`，包含每机接口、严格曲线转换与失效停止。
- `ros2_adapter/tests/`：原生输入输出、ROS 数值/归属检查与真实 Adapter/AS2 合成输入回归；
  小型冻结输入及来源哈希在 `tests/data/`，不依赖本地历史日志。
- [组件清单](component.yaml)、[能力卡片](../../../cards/planner/ego-planner-v2.yaml)、
  [Demo 与复验](../../../deployment/airsim/demos/ego_planner_v2/README.md)。

构建使用上述 native/bridge，不要求下载一份参考 ROS 1 源码。不将 `ego_planner`、
`traj_utils` 等同名包与 Swarm-Formation 放入同一源码发现、build 或 install 空间。

## 每机接口

launch 参数 `vehicle:=drone0` 只拥有所选车辆：

| 接口 | 类型／语义 |
| --- | --- |
| `/drone0/ego/goal` | `quadrotor_msgs/msg/GoalSet`，公共 ENU 目标，具体 frame 由配置校验 |
| `/drone0/ego/cancel` | `std_srvs/srv/Trigger`，请求停止；需继续等待状态 `stop_confirmed` |
| `/drone0/ego/status` | `std_msgs/msg/String` JSON，任务、revision、故障和停止确认 |
| `/drone0/ego/native/{odom,points}` | 仅该车辆的原生规划输入 |
| `/drone0/follow_timed_path` | 已有 `FollowTimedPath` Action，实际执行参考由 AS2 输出 |

空 vehicle 保留集中式兼容入口 `/ego/*`。PC 历史双机 Demo 使用该入口，取消一机时
按机群任务契约停止全队；每机模式只停止自身。不得同时启动两个模式争用同一执行端。
原生 MINCO 轨迹广播保留；本 PR 不包含 RoboPi＋PC 混合机群监管入口。

导航里程计和点云经 Adapter 转入公共 ENU，保持原时间戳与车辆原点；控制端按车辆
配置回到本地 ENU。转换不能把迟到轨迹整体推迟。最终静止段最多允许原生结束后
100 ms 的显式保持，必须包含原生末点、目标一致且已经静止；否则拒绝或停止。

## 验证状态

历史 PC 单机/双机仿真与 RoboPi 单机每机版本回归已通过，详见
[验证摘要](../../../deployment/airsim/demos/ego_planner_v2/VALIDATION.md)。本次目录整理的
构建/离线检查单独记录，不能冒充重新执行全部历史飞行。未覆盖真机、整机定位失效、
网络断开、板端双机及长期可靠性；混合双机继续在后续交付中完成。
