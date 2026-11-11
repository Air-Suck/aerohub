# Swarm-Formation 混合部署验收

机群规模、唯一板端 UAV、AirSim 车辆与端口由
deployment/swarm/configs/swarm_open_field.yaml 决定。RoboPi 只运行一个 UAV
计算节点，其余 UAV 在 PC 上分别由独立监管线程拥有独立 Docker 进程。

## 混合入口的当前边界

run_robopi.py 与 run_role_matrix.py 是本次接入共享配置的混合入口。hybrid_plan.py
从同一份配置生成 AirSim settings、PX4 实例/系统 ID/Agent 端口、fleet、deployment、
任务车辆与目标、容器名和角色归属；run_robopi.py 只消费这些产物。配置中
nodes.*.launch_command 与 ground_station.launch_command 供共享编排器使用，本入口不执行它们。

- 机数：Swarm 原生 launch、setDesiredFormation 与 adapter 都只接受 3..7 架，且名称须为
  drone0..droneN-1/Drone0..DroneN-1。共享拓扑能生成 N=2 或 N>7，但本入口会在接触
  RoboPi/Docker 之前拒绝。
- 目标队形：初始位置来自共享拓扑的环形布局（AirSim X/Y 即机群 NED 北/东，Z=0）。
  目标取原生六边形 relative_pos 的前 N 个顶点乘 swarm_scale（ENU），整体放在初始质心
  正北 6 m、高 3 m；默认三机原布局下与历史 task.yaml 的目标一致。task.yaml 的
  formation_scale 必须等于原生 swarm_scale。环形到六边形的自动重排仍未实现；当前配置按索引映射原生队形顶点。
- 角色：板端只运行一个 UAV 节点。每个 PC UAV 由一个 ComputeWorker 线程监管一个前台容器。
  中心 ros2_proxy 在中心所在机器上监听 0.0.0.0:28501。任务客户端（mission、Planner、
  swarm_formation_adapter）在 PC 的 ground 容器里；adapter 通过 SWARM_PROXY_HOST 连接中心。
  只读 audit 仍只在完整飞行时运行。2026-11-11 当前源码有屏回归已通过两种角色的完整飞行，证据在 `log/swarm-infra/role-flight-gap-fix-20261111/`。算法 worker planner 与 PC 中心 proxy 均已在真实轨迹执行后注入故障；两次均由 `OUTPUT_UNAVAILABLE` 使 Planner 进入 FAILED，三机确认 HOVER 后降落解除武装，证据分别在 `fault-worker-fixed-20261111/` 和 `fault-center-fixed-20261111/`。整台 compute 主机或容器失联属于 PX4 failsafe 边界，不等价于该进程故障矩阵。
已实现并有单测（deployment/swarm/tests/test_swarm_open_field.py，不需要 Docker、RoboPi、
ROS）：N=3/5/7 与非 0 板端索引的产物一致性、两种中心角色、N=2/8 拒绝、目标满足 adapter
队形检查、ENU/NED 方向、PC worker 线程/进程唯一、提前退出（含退出码 0）判失败、无法停止
的容器被强制删除、未启动时清理安全。launch_vehicles 默认参数回归在
catora_open_field/tests/test_sim_startup.py。

2026-11-11 将同一份 config 的 vehicle_count 改为 5 后，有屏 board_center 闭环通过，证据在 `log/swarm-infra/fleet5-flight-20261111b/`：板端 drone0，PC 四机各有独立线程和容器，planner_status 为 SUCCEEDED，五机均降落并解除武装，最小真实间距 3.02 m。该次使用按索引映射的五个原生队形顶点，没有实现队形重排。下方 2026-11-06 实测结果属于修改前的固定三机入口。

历史固定三机工具，未迁移到配置驱动，也不被混合入口调用：analyze.py（固定 drone0..2 与
compute/drone1..2 日志路径）、verify_parameters.py（固定三台 ego_planner 及 vehicle_count=3）、
纯 PC run.py 及其使用的静态 task.yaml 目标、deployment.yaml、fleet.yaml。混合入口只读取
task.yaml 的任务参数模板和 planner.yaml。

本应用使用 NH 真实 AirSim 场景、独立 PX4 和独立 AS2。
RoboPi 节点与 PC 节点各运行本机 Swarm 原生规划器、
GridMap/ESDF、导航注册、本机输入/output adapter 和 AS2。实时原生 PolyTraj 广播直接走 DDS UDP。
全局 JSON 采样代理、公共 Planner 及只读验收程序位于 RoboPi；公共 Planner 将 FleetSetpoint
分发到各机 adapter，由本机 AS2 执行。PC 容器共享 x86 宿主机与网卡，不等于三块 ARM 板。

AirSim、PX4/Agent、三路真实 LiDAR 采集与 `/clock` 在 PC。每路 LiDAR 独立使用本机采样时间，
导航节点查询本机 TF 注册到 odom，再由 Swarm 自身的固定原点桥变换到共享 ENU。
原点由地面静止状态下的 AirSim 真值与 AS2 定位校准，只在启动时传输一次并校验 SHA256。
AirSim 真值 API 仅用于标定和验收；起飞、轨迹、停止、降落都由 AS2/PX4 执行。

## 运行

测试当前 main 时可覆盖 Dockerfile 的基础镜像，使用本次编译的 AS2/PX4 环境：

```bash
docker build --network none --pull=false \
  --build-arg BASE_IMAGE=onboard_computer:main-e2e \
  -f deployment/airsim/demos/swarm_open_field/Dockerfile \
  -t uav/swarm-hybrid-compute:main-e2e .
```

该基础镜像须先从当前 main 构建，具备 Humble、PCL、AS2 和 PX4 消息依赖，记录镜像 ID。此命令只生成 PC 计算镜像，不提供 RoboPi 环境，也不将下方混合部署自动变为纯 PC 验收；启动配置仍须显式选择新镜像。

复用已验证的 `uav/catora-hybrid-compute:local` AS2 和现有本地 ROS/PCL 依赖，离线编译：

```bash
docker build --network none --pull=false -f deployment/airsim/demos/swarm_open_field/Dockerfile \
  -t uav/swarm-hybrid-compute:local .
bash tools/robopi/build_lib.sh swarm-formation
python3 deployment/airsim/demos/swarm_open_field/run_role_matrix.py --preflight-only \
  --output log/airsim/swarm-role-ground-new
python3 deployment/airsim/demos/swarm_open_field/run_role_matrix.py \
  --output log/airsim/swarm-role-flight-new
```

集中式验收必须通过 board_center 和 board_worker 两个场景，不能只运行其中一个。
前者在板端 UAV 启动中心协调进程；后者在配置选中的 PC UAV 容器启动中心协调进程。
role-matrix.json 汇总两次退出码和证据路径。单场景调试可直接执行 run_robopi.py
并显式传入 --coordination-scenario，但不构成完整的集中式验收。

故障收口矩阵：

```bash
DISPLAY=:3 AIRSIM_RENDER_MODE=windowed \
python3 deployment/airsim/demos/swarm_open_field/run_fault_matrix.py \
  --output log/swarm-infra/fault-matrix-new
```

完整的配置、运行、证据和安全边界说明见 [Swarm Infra 使用说明](../../../swarm/README.md)。

入口沿用 `.env` 的 HDC、网卡、目标地址与 `FLIGHT_BOARD_IMAGE`。
应用使用 ROS domain 91、RPC 42471/42473、PX4 TCP 4680 起（启动中继 4780 起）、
Agent 8901 起，各按机数递增；使用前确认这些端口及 RoboPi 无其他消费者。
运行期持有共享 RoboPi 操作锁，不重启或覆盖其他任务。
板端安装在 `/data/aerohub/lib/swarm-formation/install`，AS2 在 `/data/aerohub/as2/install`。

地面验证不解锁，不发运动目标：检查三路真实点云连续注册、三张地图健康、
三组原生轨迹广播的发布/订阅端点及逐机 ROS 服务往返。完整飞行另外检查滚动轨迹、
本机 AS2 控制模式、真值终点、实际机间距、碰撞以及全部降落并解除武装。

## 三机与七机的契约

- launch 显式配置 `vehicle_count=3`、各自 `local_vehicle` 和 `start_proxy=false`；
  保留默认七机入口。共享代理单实例并关闭重复广播转发。
- 原生编队图取原七机形状的前三个非共线顶点，保留图优化公式与权重；
  原生目标和公共 adapter 都显式使用 `formation_scale=2`。三机减少了求解规模，
  不能据此宣称七机压力验收通过。
- 真实导航模式采用目标消息中的高度；旧模式默认仍保留上游固定 0.5 m 的行为。
  本次飞行高度为 3 m；地图高度和虚拟天花板显式匹配飞行空间。
- proxy 返回多项式实际采样时间 `valid_from`；adapter 保留该时间，
  不在收到响应后重新加 lead，以免网络和调度延迟造成轨迹时间平移。
  速度与加速度从原生多项式解析求导，并随位置一起传入 AS2；超出轨迹末端的位置保持对应零导数。
- 飞行配置的 `wait_for_output=true` 要求三机各自唯一的 Swarm AS2 adapter 发布端（不计 AS2 起降行为预创建的端点）存在后才下发目标。
  之后仍检查轨迹起点连续性和输入新鲜度。
- Swarm 拥有自己的输入与 AS2 输出 adapter。输出采用公共转换函数，并在成功、
  失败、取消或暂停时直接请求本机 AS2 HOVER，等待服务响应及控制器反馈。

每个节点的 `metrics.jsonl` 将原生 Swarm、地图桥、导航注册、AS2 和协调/观测开销分组。
进程 CPU 的 100% 表示一个逻辑核心，host CPU 表示整机比例；PC host CPU 包含其他任务。
RSS 为进程相加，包含共享页重复计数。输入年龄含仿真时钟分发，ROS 服务 RTT 含调度和序列化。

任务超时采用 ROS/仿真时间（90 s）。外层墙钟保护为 180 s，避免负载使仿真慢于实时后，
验收程序在算法任务截止前提前取消。原始记录同时保留 wall/sim 时间，分析时报告实时因子。
原生 `Time stamp diff` 超过 250 ms 的广播会被算法拒收；该年龄以多项式起始时间计，
包含规划调度和回调排队，不能直接用作网络延迟。原生优化日志也使用 ROS 时间，
出现 `0 ms` 不表示求解没有消耗 CPU。

汇总入口（不修改原始证据）：

```bash
python3 deployment/airsim/demos/swarm_open_field/analyze.py --ground log/airsim/swarm-ground-new \
  --flight log/airsim/swarm-flight-new --output log/pr-history/deployment/airsim/demos/swarm_open_field/hybrid-validation.json
```

应用最小间距验收为 2 m，三台原生规划器均显式配置
`optimization.swarm_clearance=2.0`；原生默认 0.5 m 保持不变。该参数进入原生碰撞代价和检查，
不只是验收脚本的显示值。PC 的 `planner.yaml` 在启动时从同一应用配置只读加载，
板端配置随运行包校验，避免镜像烘焙的旧参数与板端不一致。
原生软约束不构成实际 2 m 间距的证明，仍以实测定位与真值分别验收。

## 实测结果（2026-11-06）

[完整验收](../../../../log/pr-history/deployment/airsim/demos/swarm_open_field/hybrid-validation.json) 对应 `log/airsim/swarm-hybrid-ground-04` 和
`log/airsim/swarm-hybrid-flight-04`；[默认 0.5 m 间距失败记录](../../../../log/pr-history/deployment/airsim/demos/swarm_open_field/default-clearance-validation.json)
保留在调整应用参数之前，没有用新结果覆盖原始失败日志。

| 指标 | 结果 |
| --- | --- |
| 任务与退出 | SUCCEEDED，三机悬停、降落并解除武装，无新碰撞 |
| 真值终点最大误差 | 0.524 m |
| 最小实际间距 | AirSim 真值 2.746 m；定位 2.916 m；采样参考 3.070 m |
| 原生规划与执行 | 各节点一个原生规划进程、五个 AS2 原生进程；111 条有效 PVA 段 |
| 原生多项式/广播数 | drone0/1/2 为 11/7/12，三台均实际滚动重规划 |
| 真实注册点云 | 约 7.75–7.96 Hz 墙钟；执行期输入和地图采样健康率 100% |
| 跨机服务 RTT（起飞前各 20 次） | 中位 7.95–8.83 ms，P95 约 11.3 ms |
| 实时因子 | 0.809；规划前定位与建图阶段也只有约 0.82 |
| RoboPi 整机 CPU | 中位 93.5%，最高 96.4%，实际轨迹窗口约 26.9 s |
| RoboPi 进程 RSS 合计 | 约 2.52 GiB，包含共享页重复计数 |

AS2 未出现轨迹拒收或断流，参考源年龄最大约 72 ms。墙钟下 setpoint 约
36.6–38.6 Hz，不应忽略 0.81 实时因子后直接归为网络丢包或算法算不动。
RoboPi 有一条原生广播起始时间差 269 ms，被原生 250 ms 检查拒收；仍需继续调查回调排队、
轨迹生成和通信各自的耗时，不能称通信零问题。

板端以单核 100% 计：原生 Swarm/建图约 90%，AS2 原生约 103%，导航注册约 27%，
固定地图桥约 11%，各类 adapter 合计约 225%，全局代理约 71%、公共采样约 86%，
验收进程约 103%。这个布局接近板端 CPU 上限；单机生产负载应另测移走全局验收/协调后的情况。
三台实际运行参数已通过只读 DDS 服务回读，记录见 `native-parameters.json`。

测试包括 12 项 Swarm 契约/解析采样检查、23 项公共框架回归、2 项 TCP 中继回归，
以及隔离默认七机代理的缺失/失效/过期健康门控与恢复。未测试七机真实场景、无线链路、
主动断网、注入丢包或长期运行；本轮通过只限于本配置与场景。
