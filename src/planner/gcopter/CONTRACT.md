# GCOPTER Planner / Component 契约

当前任务方向：未知环境的 LiDAR 局部规划。全场景先验地图路线已移除。局部模式已实现独立前端和滚动轨迹更新，实际闭环程度见 VALIDATION；不沿用历史先验飞行作为当前目标证据。

## 局部规划接口

Function `gcopter_local_function` 直接消费 `navigation/points` 与 `navigation/odom`（同一固定车辆 ENU）；扫描使用原观测 stamp 配对至50ms以内的姿态，等待/丢弃未配对或过期数据。ROG-Map 私有副本进行概率射线更新、障碍膨胀和滑动；不加载 PCD 或场景真值。仅在显式 known-free 格中选取可达局部目标与路线，以实际膨胀障碍生成 FIRI 走廊，调用原 GCOPTER 优化器；优化后独立核查轨迹在已观测自由空间内。终端 V/A 为零以保留可停车轨迹。

任务目标采用显式 `goal_tolerance`，默认1.5m，与 SUPER forest 验收语义一致；精确目标体素未观测时，只能在容差内选择实际已观测、可达且通过完整轨迹检查的终点，并报告真实误差。该容差是局部导航完成语义，不放开未知空间或碰撞检查。

除规划核心外，局部地图与验收尽量对齐 SUPER forest：同一 ROG-Map 分辨率、1m占据膨胀、滑动/射线概率参数、LiDAR范围、速度方向yaw、1.5m到点与低速保持、AirSim碰撞事件及停止/降落证据。`deployment/gcopter/check_super_alignment.py` 固化这些共享项并在漂移时失败。GCOPTER自己的局部路线、FIRI走廊、MINCO/GCOPTER优化、滚动交接与停止策略是明确差异；SUPER探索/备份轨迹尚未实现，不通过对齐配置冒充等价。

调用 `gcopter/enable` 准备执行，`gcopter/plan_once` 启动局部规划循环。首次是明确的静止边界；后续从已获 FollowTimedPath 接受确认的旧轨迹提取未来 P/V/A 交接。TimedPath 保留 task_id、递增 revision、generated_at、valid_from、is_final 与全轨迹；adapter 的 `local_mode:=true` 保留规划时刻，通过已有 FollowTimedPath modify 接口更新。仅接近最终目标的终端段为 is_final。取消后不复用旧任务，重启并重新获取地图后开始下一任务。

LiDAR/状态断流、轨迹被新占据阻断、交接确认超时或可行轨迹耗尽时请求停止；普通单次求解失败可继续使用已有停车轨迹并尝试下一次。求解器尚不具备计算中硬取消；超期结果丢弃。首次实现不包含 SUPER 探索/备份优化器，不承诺复杂死胡同的全局探索完备性。

## 保留的快照批处理基线

首期：单机有限范围静态占据快照、批处理全局动力学轨迹。排除滚动、多机、动态障碍和自主建图。Function 只产计划，FollowTimedPath 执行，起降使用原生 Behavior。

核心输入：同一固定 ENU 米制起终 P/V/A（3×3 列矩阵）、已配准占据点。边界、分辨率、膨胀显式配置。边界外拒绝；未知空间沿用上游自由假设，仅允许明确覆盖的静态地图。零 V/A 必须是明确的静止边界，不冒充测量。机体系单帧点云不能改名作为全局地图。

Harness 必须冻结状态、目标世代与地图 epoch，拒绝 frame 不符、陈旧/未来状态、无效数据、并发；reset 清除旧计划。输出完整五次曲线 duration/P/V/A；采样含零点、每段边界和终点，纳秒严格递增；yaw 由调用方显式给定。generated_at 为完成时刻，valid_from 必须与边界状态一致；不靠改 now 掩盖迟到。静止起点执行前重新核对保持状态。

执行完成、取消与输出静默由 Behavior 反馈确认，算法完成不等于到达。不直接发布 Motion Reference；失败不重发旧计划。

运行前验证矩阵：空旷与障碍成功，起终占据、越界、NaN、无解墙、地图 reset；固定同走廊对照上游/目标 PVA 与 duration，绝对容差 1e-8；端点误差 1e-5、时间正且有限、1ms 约束采样；ROS 时间/frame/QoS/并发/恢复；真实 AirSim 输入及 FollowTimedPath 消费；有屏 AirSim+PX4 正常/取消/断流/恢复/清理。任何缺项均未完成，不下调门槛。

故障补充门（用例执行前声明）：取消与地图reset在运动开始1秒后注入；要求stop_confirmed、速度<0.3m/s，以及停止确认后2秒内trajectory参考计数不增加；随后原生Land并确认disarmed。状态断流、求解进程退出、重启恢复与资源冲突仍在矩阵中，不以这两项代替。

Forest 用户目标：LiDAR 输入链下的森林长路径约3m/s。完成判定沿用SUPER forest的1.5m到点和速度低于0.3m/s持续2秒，并要求无新增飞行碰撞、FollowTimedPath实际消费、停止确认、原生Land和disarmed。速度报告名义值、航段峰值和任务结果，不添加瞬时超调硬门。当前验收结果见VALIDATION。
