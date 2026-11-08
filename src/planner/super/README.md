# SUPER 的时间化 Behavior 接入

算法仍来自锁定的 `thirdparty/SUPER-sim` fork；本目录只保存本仓库适配包。
原生输入、优化与主/备用轨迹计算不搬入 Behavior。当前验证对象是 PC forest8，
不表示 ROS/OHOS 算法部署或 RoboPi 闭环已完成。

## 执行链与接口

`super_planner` 输出 `/planning_cmd/poly_traj`（PolynomialTrajectory）。
`uav_super_adapter/super_timed_path_adapter` 将完整 committed 轨迹转换成
TimedTrajectorySegment，并复用机群客户端调用逐机 FollowTimedPath；
Adapter 不发布 motion_reference，不与旧 SUPER 单点桥同时启用。

SUPER 的 CmdTraj 在原生内部拼接主轨迹前缀和备用制动轨迹，导出的 committed
多项式已经包含该拼接结果。适配保留所有原生分段边界与整段未来覆盖，
不只截取主轨迹或终点。PositionCommand 的 flag=1/2 用于源状态和主/备用观测，
不会作为另一个运动参考源发布。单纯 HEART_BEAT 不重建或延长已缓存计划。

本任务选择固定接管航向，与旧 SUPER adapter 默认保持一致；没有实现原生
Yaw polynomial 约束适配，不宣称支持要求跟随航向轨迹的任务。

## 时间、坐标与结束

- 源使用固定局部 ENU `drone0/odom`。接口转换到统一局部 NED，再由既有客户端
  转回逐机 ENU；原点为零，不重复减去任务平移。frame/vehicle/namespace 须匹配。
- 保留源 generated_at 和原多项式执行 epoch，只裁掉已过去的前缀。
  默认在当前时刻后100ms执行原曲线上的对应后缀，不将曲线整体推迟。
- 原生七次曲线每20ms解析采样 PVA，并显式保留每个分段边界；Behavior 在节点间
  使用五次 Hermite 插值。它是近似表示，测试误差不是所有输入的统一误差上界。
- 过期或不足未来窗口的更新丢弃，既有计划的结束时间与过期保护不变。
  不允许给旧轨迹重盖时间戳充当新轨迹。
- 终点在任务 goal_tolerance 内且终端速度<=0.3m/s时可标记最终段；Behavior
  仍按自身0.25m误差及超时判断执行结束。任务还须检查Action成功、实际到点保持。
- 新鲜、递增的原生命令才续源健康状态；重复、错误frame/非法PVA不续期。
  执行中500ms没有新源命令则关闭执行；初次规划预算10s。
- `super_adapter/enable`（SetBool）是应用接管/退出入口。停用后需等待
  `super_adapter/state` 的 stop_confirmed，不能把服务响应直接视作飞机已停止。
- `super_adapter/state`（String JSON）包含任务、revision、complete、error、
  source_flags、late_updates、源曲线元数据与停止确认。曲线元数据是最近解析值，
  不是每个版本已接受的证明；结果与取消以Action和停止反馈为准。

## 构建与验证

完整PC场景入口见 `demos/super_forest/run.py`。镜像基于现有 onboard_computer，
原生算法在现有 ROS Humble desktop 镜像离线编译，运行层携带所需非ROS动态依赖。

数学测试可直接运行；真实ROS消息转换、重复/过期样本、备用命令及节点构造
须在新镜像中验证。C++ interpolation_test.cpp 使用实际Behavior采样器对照
七次测试曲线。真实Behavior终点模式及外部接管测试位于
`src/behavior/follow_path/tests/terminal_mode_test.py`。
