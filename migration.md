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

CAT-ORA PC 应用已支持显式 CATORA_EXECUTION_BACKEND=follow_path，基于
onboard_computer:v0.1.0 的独立应用镜像已通过四机正常飞行场景。板端和 Swarm
应用仍为旧链路；SUPER 的独立 PC 新链路结果见本文末节。不能据此宣称 OHOS/RoboPi 迁移完成。未导入依赖旧 Gazebo 部署的验收驱动。
PC 复现方式和局限见 [应用说明](deployment/airsim/demos/catora_open_field/README.md#followtimedpath-迁移的-pc-验证入口)。

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
这些测试不证明板端加载或飞行兼容。AS2/Pixhawk 已归位 as2/ 并保持原 fork 提交。OHOS 源码准备、runtime 构建选包和部署打包已纳入扩展，尚未实际交叉编译或部署。CAT-ORA PC Action 编排及正常动力学场景已验证；板端编排、OHOS 编译与板端动力学验收尚待实施。
历史证据见各 demos 的 validation JSON 与 README，仅代表原运行组合。

回退时先停止新执行、确认无活动参考源，再切回原分支对应的镜像与板端安装。
禁止运行中叠加两套 AS2 install；冻结 v0.1.0 不重新标记为迁移构建。

完整顺序、验收门槛见 [迁移实施计划](docs/migration-plan.md)。

## RoboPi 就绪纠正

当前迁移不具备板端端到端就绪条件；实测工作区源码未同步、板端无执行扩展。
CAT-ORA/Swarm 的板端入口仍使用旧应用流程，SUPER 缺少统一环境入口。
详见 [RoboPi 就绪审计](docs/robopi-migration-readiness.md)。旧板端验证记录不能替代本轮验收。

## CAT-ORA PC 场景验证

基于现有 onboard_computer:v0.1.0 构建独立迁移应用镜像，FollowTimedPath 四机正常
与显式取消均已通过，结束时均确认降落解除解锁。正常场景最大定位终点误差
0.228 m、真值终点误差 0.131 m、最大采样跟踪误差 1.089 m；不是全程厘米级跟踪。
取消到全机群停止响应/模式确认约 0.126 s，不是制动距离或零速度保证。

保留一轮首轨迹前失败及其恢复记录；原失败未记录具体 failure_reason，根因未确认。
PC 首轨迹使用独立30秒冷启动预算，后续轨迹超时保持5秒。
原始证据、准确镜像 ID、源码哈希和失败限制见
[版本化摘要](deployment/airsim/demos/catora_open_field/migration-pc-validation.json)。
本结果不覆盖 OHOS/RoboPi、混合部署、Behavior 断流、Swarm 或 SUPER。

## SUPER PC 迁移结果

新增 src/planner/super/ros2_adapter 与 deployment/airsim/demos/super_forest 的独立PC构建、启动、
任务和分析入口；完整committed曲线（含备用拼接段）经 FollowTimedPath 执行。
源算法仍在原fork子模块，不重启或替换原有SUPER运行。Adapter固定航向，未实现yaw曲线约束。

指定 forest8 PC 场景已完成 Action 成功、0.162m到点保持、停止确认及降落解除解锁。
但X超观测边界6.96m、峰值速度8.36m/s，且前轮有未解释的碰撞失败；不宣称整体就绪。
RoboPi/OHOS、混合部署、受控备用切换与可靠性复验仍未完成。

为迁移补充的公共执行修复已单独提交：正常终点POSITION收敛不再误判接管；
客户端按当前Action反馈revision观察settling；运动参考保留真实采样时间。
Action消息未改，原生FollowPath仍保留，未修改AirSim/PX4算法或保护参数。

证据与失败限制见 [SUPER版本化摘要](deployment/airsim/demos/super_forest/migration-pc-validation.json)。

## verify 合并与推送交接

本轮整理交付到 `verify/px4-airsim-as2`，不直接合并 main；后续由 PR 管理员审阅合并。
提前实现的算法迁移 skill 已通过撤销提交排除，镜像发布文档已单独提交。

整理时在 `uav/apps-super-forest:migration-attempt7` 镜像内，以只读源码挂载、禁用网络的方式复核：
framework 27 项、SUPER 8 项全部通过，无跳过；真实 Mission/Adapter 构造，以及 C++ Behavior
正常终点收敛和外部模式接管测试通过。镜像 ID 为
`sha256:41da0729254202cc409c6a29b023e1f87f3c6e2b8f554e9b8421353a7113c0de`。
本轮未重跑飞行，PC 场景结论及未解决限制沿用上述版本化证据；RoboPi 仍未验收。

推送前已 fetch 核对：AS2 fork 的 `f387e0d733a5102808508bcbec1b9eca2c29d1f0`
比其远端 main 多一个本地提交，必须先发布依赖，再推主仓库。以下为待执行命令，本轮没有执行 push：

```bash
git -C as2/airsim/aerostack2 push origin HEAD:main
git push --recurse-submodules=check origin verify/px4-airsim-as2
```

第一条目标为 `Popc0rn7/aerostack2-aerohub` 的依赖 fork，不是本项目 main。
若推送时远端已有新提交，重新 fetch 并处理差异，不使用强制推送。
