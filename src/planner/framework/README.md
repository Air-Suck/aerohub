# Planner 执行框架

默认 `execution_backend:=follow_path`：算法 Adapter 输出 TimedTrajectorySegment，
框架负责校验和机群调用协调；逐机 FollowTimedPath Behavior 负责采样与执行。

`execution_backend:=legacy_setpoint` 显式启用旧 FleetSetpoint 输出桥。
CAT-ORA PC Demo 可显式选择 follow_path；旧板端及 Swarm Demo 仍选择 legacy。
禁止同机同时开启两条执行链。

- `uav_planner_framework/`：配置、生命周期、校验、完成判定与 Action 客户端。
- `registry/`、`examples/`：算法 manifest、任务与部署配置。
- [公共接口](../interfaces/)：ROS 消息、服务和 Action。
- [FollowPath 扩展](../../behavior/follow_path/)：时间化轨迹执行。
- [旧输出桥](../../behavior/legacy_setpoint/)：仅用于显式旧链路。

AS2 启动器通过 `AS2_EXECUTION_BACKEND=follow_path` 选择扩展节点；未设置时保留
既有 Demo 的启动行为。使用新模式前必须构建安装扩展和接口包。
迁移状态与验证边界见 [交接文档](../../../skill/temp/pr-main-layout/history/migration.md)。

## 执行状态观察

新后端发布 `planner/execution_state`（std_msgs/String JSON），字段包含 task_id、
backend、status、failure_reason、prepared、stop_requested 与 stop_confirmed。
prepared 仅表示服务可发现；stop_confirmed 只有全机群停止响应成功及后续模式反馈
均满足时才整体置真，不能将规划服务 CANCEL 获准视为停止完成。
该话题用于应用观察，不替代 FollowTimedPath 的 Goal/Feedback/Result。
