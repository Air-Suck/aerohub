# IPC native ROS 2

来源为 `hku-mars/IPC`，锁定提交 `0f959ca7995e71a1fcc4942bd4a86869ab6ee975`；原始内核文件哈希见 `upstream-manifest.json`。保留几何库版权与许可文本；上游根目录未找到统一许可证，不为整个算法重新声明许可证。

算法运行在 PC 算力侧：真实 `navigation/odom`、注册到 earth 的 `navigation/points` 与既有 `ControlSession` → A*/Floyd → EMVP 安全走廊 → MPC → 平坦输出转换 → `ControlFrame`。既有 FollowReference 的 external_acro 插件管理执行生命周期，经 AS2 控制器和 Pixhawk 平台送到 PX4。起降使用原生 Behavior；本包不实现 Mission、Tool 或 ROS 1 运行节点。

## 接口与范围

输入里程计的线速度为 body FLU，节点用当前姿态转为 earth；点云须已注册到 earth。会话绑定 session/revision，仅支持固定 yaw，逐轴速度限幅取Goal与只读`mpc.velocity_limit`的较小值（节点默认0.5m/s，本轮应用为4m/s）。输出机体 FLU 角速度 rad/s；推力通过本次悬停油门和 AS2 最大推力参数映射，不能视为实机牛顿标定。

`ipc/ready` 表示输入新鲜；`ipc/health` 记录求解状态与耗时；`ipc/path` 在路径更新时发布；`ipc/processed_points` 提供保留原采样时间的裁剪降采样输入。启动参数只读。仅 OSQP_SOLVED 可输出有效控制；失败、超龄、撤销会话均不能继续把旧解当新解执行。

控制与重规划分开：A* 在独立工作线程使用不可变输入和独立栅格，结果仅在同一会话且地图年龄不超过 0.5s 时安装。每次新扫描刷新历史地图并检查剩余路径，仅受阻、偏离或接近临时终点才重新搜索；MPC 始终使用当前地图构建走廊；独立发送线程在原计算时间的 ROS/墙钟 100ms 有效期内发送缓存，不刷新计算时间。它不是 100Hz 求解或 PX4 闭环保证。

每个会话在同一ROS时刻至多尝试一次新计算，避免timer排队在clock未更新时生成不同值但相同源时间的控制帧；重复发送保留原值与原年龄，时钟回退不产生新解。接续验证中200Hz clock的真实[4,0,1]短程严格映射通过。当前自由XYZ SUPER A→B名义3m/s长任务已在两次全新启动中完整通过，旧20m定高结果仅作历史。

后续核对确认86.38Hz是发送缓存观察下界。当前health追加原始求解/状态/点云stamp及到health前耗时；直接记录的真实短程2181次成功求解平均99.815仿真Hz，间隔p95=15ms、max=27ms。节点名义100Hz能力与硬实时期限分开，当前长任务直接成功求解平均99.854/99.666仿真Hz，求解仍有间隔抖动，传感器年龄和固件逐周期消费分别报告，见Demo的`validation-free-xyz-20261110.json`。

保留上游算法核心，但不是全部上游功能的等价迁移：当前单机、静态场景，保留最多十帧、两秒内注册扫描的局部静态历史；在线推力辨识、RC 接管和 ROS 1 起降未迁移。局部地图默认20×20×8m、高度可配置（当前应用0…12m earth绝对高度）、分辨率0.1m，远目标裁剪为局部目标后滚动更新。节点默认参考速度0.25m/s，这些是历史低速默认；当前应用参考3.6m/s、起步参考斜坡1m/s²、加速度3m/s²、jerk10m/s³、SFC余量0.55m。名义3m/s与实际速度另行验收。

## 数值与移植修正

移除内核的 ROS 1 日志、参数、计时与可视化依赖。修正无穷大常量、构造、C++17 比较器、边界和初始化；每次搜索重置已访问节点，覆盖连续二十次规划。占据起点仅可经0.6m内、整段满足既有SFC余量的连接接回自由格，保留真实首点，不清障碍格；无安全连接则拒绝。EMVP与SetFSC检查有限性，每轮开始清空部分走廊约束。OSQP 热启动保留原始/对偶变量和 rho；未收敛迭代只用于数值初值，不用于执行。

走廊约束进入 QP 前，仅删除在既有 jerk 盒约束内必然满足的行，保留其余约束与求解精度。真实森林失败 QP 经独立 LP/SLSQP 核对可行；删冗余前后最优目标与原约束违例一致。回归覆盖远处冗余平面和近处必须保留的平面。

## 构建和验收

从仓库根执行，复用共享镜像，在外部工作区补编译：

```bash
git clone --branch v0.6.2 --recurse-submodules https://github.com/osqp/osqp.git log/ipc/native-deps/osqp
git clone --branch v0.6.3 https://github.com/robotology/osqp-eigen.git log/ipc/native-deps/osqp-eigen
bash deployment/airsim/demos/ipc_control/build_overlay.sh
bash deployment/airsim/demos/ipc_navigation/build_core.sh
bash deployment/airsim/demos/ipc_navigation/build_node.sh
```

依赖锁定 OSQP `f9fc23d3436e4b17dd2cb95f70cfa1f37d122c24`、QDLDL `7d16b70a10a152682204d745d814b6eb63dc5cd2`、OsqpEigen `0f3f8c7271931333b70f77a4db7039d764834ffe`，构建入口核对源码提交。

`tests/node_smoke.py` 检查真实节点接口、非恒等姿态下速度转换、陈旧拒绝、会话修改与撤销；`tests/replay.py` 回放真实录包输入，不启动执行链或仿真。编译与这些测试不能替代飞行验收。具体运行结果及失败复盘见 [IPC validation](../../validation.md)，forest 入口为 `deployment/airsim/demos/ipc_navigation/run.py`。

## 与上游运行分支的对应

| 上游分支 | 当前native处理 |
| --- | --- |
| PathReplan连续搜索/Reset | 每轮Reset访问节点并有多轮回归 |
| 起点占据后的周边搜索 | 迁移初期遗漏，现改为有限范围且整段校验的连接，保留真实首点 |
| 终点占据后的周边搜索 | 只允许调整临时局部目标；最终请求目标占据时拒绝，不悄悄改终点 |
| 远目标裁剪及追加终点 | 裁剪到局部范围并滚动重规划；不追加地图外未经检查的直线 |
| A*失败后当前点参考 | native报告规划失败；执行层以有效控制时限和Hover/降落收口 |
| 点云十帧历史 | 裁剪后的注册扫描累计最多十帧/两秒，合并降采样；诊断输出仍为单扫描，回放不重复累计地图 |
| MPC/安全走廊 | 保留模型、约束和参考选择；精确删除冗余约束，显式拒绝未收敛/非有限输出 |
| 固定yaw/RC/manual/起降 | 固定yaw绑定会话；RC与起降由现有平台/Behavior负责 |
| 在线推力模型估计 | 未迁移；使用接管前悬停油门与名义归一化映射 |

历史forest任务参考0.25m/s保留为证据；当前默认入口选择super-3mps-balanced-task.json，所有新飞行要求planar_path=false。

历史20m forest应用选择只读`planar_path=true`：A*在目标巡航高度平面搜索，MPC仍控制完整三维状态。建立平面路径时要求当前高度与目标平面相差≤0.25m，并检查、保留真实起点的连接。默认false保留自由三维路径；前序产物当时仅验证定高forest，当前自由XYZ长任务结果见最新摘要。搜索膨胀0.75m、路径复查0.55m、SFC0.55m分别保留职责，不能合并为一个“避障半径”。

历史固定高度forest已按原基准在两次独立全新启动中通过，详见[版本化验收](../../../../../deployment/airsim/demos/ipc_navigation/validation-20261110.json)。

原走廊距离时序QP失败时，可在当前状态仍位于初始有限凸走廊内的条件下，以该走廊约束所有预测步重新求解，保留全部运动硬约束；仅新OSQP_SOLVED解可执行。回归拒绝已出走廊或无法刹停的状态，原始primary失败矩阵留证，health单列conservative_corridor。参考起步斜坡避免瞬时升到巡航推进尺度。这些是项目明确增加的适配行为，不冒充上游原样实现。

当前自由XYZ名义3m/s、约238m森林在两次相同产物/配置的全新启动中完整通过，含停止、原生降落和严格源→AS2→PX4数值验证，见[新验收摘要](../../../../../deployment/airsim/demos/ipc_navigation/validation-free-xyz-20261110.json)。未验收突发/动态障碍、风/外力扰动、RoboPi、真机或硬10ms期限。
