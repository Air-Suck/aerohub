# main 迁移交接

## 状态与权威

本文件记录正在实施的迁移，不是已完成声明。来源基线为
`verify/px4-airsim-as2` 的 `b3be98e`，目标架构基线为 main `b701e04`。
实施分支：`integration/main-behavior-migration`。

命名、目录和规划/执行职责遵循 main；AirSim/PX4 版本、配置、消息兼容、
启动、通信、传感器与 clock 以来源分支为权威。不能用 main 的 Gazebo 部署覆盖。
迁移 PC 镜像构建默认使用 onboard_computer:migration-main-behavior；可通过 MIGRATION_ONBOARD_IMAGE 指定独立标签，不覆盖冻结 v0.1.0。
Humble 算法、RoboPi/OHOS、AS2/Pixhawk 的 PVA 和失流保护必须保留并重新验证。

## 当前实施状态

- main 的独立 `uav_follow_path_execution` 包及无 ROS 时间线测试。
- `FollowTimedPath` Action、`TimedPath` 消息及 action_msgs 构建依赖。
- main 的机群 FollowPath 客户端已接通 framework；默认 follow_path，显式 legacy_setpoint 保留。
- Planner、接口和 Humble 算法已归位 src/planner，旧桥归位 src/behavior/legacy_setpoint。
- 加强停止响应与模式反馈检查，保留输入新鲜度、时钟保护及进程组回收。
- 缺失加速度保持 NaN，未指定 yaw 固定为接管时航向；轨迹模式取得后丢失会结束执行。
- PC 镜像配方加入执行 overlay；启动器支持显式 AS2_EXECUTION_BACKEND=follow_path。

现有 Demo 显式使用旧链路，应用编排尚未迁移。本次不宣称 Behavior
后端、ROS 构建、OHOS 或飞行已通过。未导入依赖旧 Gazebo 部署的验收驱动。

## 接手执行顺序

1. 检查工作树与子模块；部署文档和发布清单中的既有修改不得覆盖或混入迁移提交。
2. 按详细计划将 framework、interfaces、算法归入 main 的 src/planner 布局；
   同步 Docker COPY、colcon 路径、Registry、脚本与测试，不改 ROS 包名。
3. 在 main 的 as2 布局中使用来源分支 fork，整合必要兼容和保护；避免重复应用补丁。
4. 接通 FollowPath 客户端、执行结果与停止确认，保留显式 legacy 后端。
5. PC 和 OHOS 构建扩展包；逐机替换 FollowPath 节点，保留原生起飞降落。
6. CAT-ORA 先完成接口、PC 四机、RoboPi 与既有混合拓扑验收。
7. Swarm 和 SUPER 后续分别迁移，未完成前保持原入口且不得与新入口同时控制同机。

## 必须修正的集成问题

- main 客户端 quiescent 仅检查 Future 完成，不足以确认停止；须检查响应和执行状态。
- FollowPath 中缺失导数与零前馈、插值导数的语义须与现有 NaN 契约对齐。
- 无 yaw 时应捕获接管时局部航向，避免混用 earth 与 odom。
- Behavior 当前模式请求行为须结合恢复锁定验证，禁止模式丢失后反复抢占。
- 共同执行时间必须保留；迟到拒绝，不能通过重盖时间戳使过期计划复活。

## 验证与回退

当前验证：现有 onboard_computer 镜像内接口和扩展编译通过；框架 26 项（含真实 ROS 消息）及 CTest timeline 通过；CAT-ORA 6 项、Swarm 转换 9 项通过；板端打包布局 2 项通过。Swarm 12 项（含真实 traj_utils 多项式测试）在已有 Swarm 镜像中通过。
这些测试不证明板端加载或飞行兼容。AS2/Pixhawk 已归位 as2/ 并保持原 fork 提交。OHOS 源码准备、runtime 构建选包和部署打包已纳入扩展，尚未实际交叉编译或部署。Demo Action 编排和动力学验收尚待实施。
历史证据见各 demos 的 validation JSON 与 README，仅代表原运行组合。

回退时先停止新执行、确认无活动参考源，再切回原分支对应的镜像与板端安装。
禁止运行中叠加两套 AS2 install；冻结 v0.1.0 不重新标记为迁移构建。

完整顺序、验收门槛见 [迁移实施计划](docs/migration-plan.md)。
