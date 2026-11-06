# Forest 8 m/s：PC SUPER → RoboPi AS2 → PX4

这是 SUPER 的端到端样例：真实 LiDAR / AS2 定位 → SUPER → 单点轨迹 adapter
→ RoboPi AS2 → PX4 / AirSim。任务入口为 `forest8_as2_mission.py`，算法源码仍在
`thirdparty/SUPER-sim/`，公共执行桥仍在 `planner_framework/bridges/as2_setpoint/`。

## 运行前提与任务入口

当前提供任务脚本、场景 / 规划配置和一次完整飞行记录，尚未提供与 CAT-ORA、Swarm
一致的环境构建、部署及启动编排入口。`source-profile.yaml` 是原 LeUAV 配置快照，
不能直接作为本仓库的启动配置；下文的现场进程状态也只是历史记录。

运行任务前，需要在已加载 ROS 2 Humble、AS2、配套 PX4 消息、SUPER 消息、
公共轨迹桥及 AirSim sensor adapter 的 PC 环境中准备：

- 使用本目录 `settings.json` 启动匹配的 NH 场景、PX4 / Agent 和 `/clock`。
  此脚本固定访问本机 RPC 41472，使用 ROS domain 83 和 `drone0`。
- 启动 RoboPi AS2、PC 导航输入和 SUPER，使用本目录 `planner.yaml`。
  导航输入为 `/drone0/navigation/odom`、`/drone0/navigation/points`。
- 按[轨迹桥说明](../../planner_framework/bridges/as2_setpoint/README.md#super)
  提前启动 `super_as2_adapter_node`，保持禁用，使用 `drone0/odom` 和仿真时钟。
- 为每次任务准备独立输出目录，在执行环境中挂载为可写 `/evidence`；复制
  `source-task.json` 到该目录，并根据本次地面定位重新生成 `origin.json` 中的
  `task_to_odom_translation_m`。同步平移规划配置中的原点、虚拟地面和天花板。
  本目录的 `origin.json` 是历史测量，不能直接当作新一次运行的标定。

在上述 PC 环境中，从仓库根目录执行（会发送仿真起飞及运动指令）：

```bash
ROS_DOMAIN_ID=83 python3 demos/super_forest/forest8_as2_mission.py --ros-args \
  -r __ns:=/drone0 -p use_sim_time:=true -p frame_id:=drone0/odom
```

脚本将飞行采样写入 `/evidence/flight.jsonl`，结果写入 `/evidence/result.json`。
通过条件是 `success=true`，包括到点和降落解除解锁；不能仅以 Python 进程退出码
判断成功。任务异常时尝试悬停、禁用 adapter 并降落；如结果包含 `recovery_error`
或阶段为 `RECOVERY_REQUIRED`，必须检查仿真飞行状态再停止运行环境。
本入口不负责清理预先启动的场景、容器或板端进程。

## 历史验收（2026-11-05）

本轮任务已完成，AS2 正常起飞、执行 SUPER 长程目标、到点悬停、降落并解除解锁。
实测为一次成功飞行，不代表重复成功率验收。

## 找到的既有任务

- 原入口：`/home/wqy/uav/leuav/profiles/super.forest8-ab.yaml`。
- 任务：`cases/super/config/task.forest8-ab.json`，完整字节快照为 [source-task.json](source-task.json)。
- 原规划配置：`super.search150-forward250ms.yaml`；本轮有效配置为 [planner.yaml](planner.yaml)。
- 出生 NED `[42.6,160.11,0]`；B 为任务局部 ENU `[-37.57,231.76,6]`，直发单目标。
- 速度 8 m/s、加速度 8 m/s²；A* 150 ms、前视 250 ms、规划范围 20 m；起飞 2 m，悬停 6 s。
- 保留航段 300 仿真秒、任务 600 仿真秒、终点误差 ≤1.5 m 且速度 <0.3 m/s 持续 2 仿真秒的要求。
- 保留原 observe 策略；新航路碰撞、状态/时钟失效、轨迹模式丢失、任务超时中止并尝试 AS2 降落。

## 实测

| 项目 | 结果 |
| --- | --- |
| 到点误差（局部里程计） | 0.118 m |
| 实测速度峰值 | 8.460 m/s |
| 参考速度峰值 | 8.0008 m/s |
| 参考加速度峰值 | 7.9565 m/s² |
| 长航段时间 | 38.86 仿真秒 / 51.12 墙钟秒 |
| 观测路径长度 | 260.80 m |
| 路径均速 | 6.71 m/s（仿真时间） |
| 净推进均速 | 6.04 m/s（仿真时间） |
| 新航路碰撞 | 无 |
| AS2 降落与 PX4 Disarmed / landed=True | 通过 |
| 捕获 actuator 参考源时间戳 | 1654 条全部可匹配源参考 |

观测边界不是全段满足：X 最大 20.444 m，超过任务上界 20 m
约 0.444 m，约持续 1.20 仿真秒；任务监测器 12 次采样、
原始里程计 121 条记录。Y、Z 未越界。原任务为 observe，故记录而未中止；本轮不能宣称
满足严格 X 边界。峰值速度超过名义 8 m/s，同样按原 observe 策略保留，不隐藏或改阈值。

SUPER 捕获到的 trajectory_flag 全为 1；没有触发 flag=2，因此不把此轮当作备用轨迹切换验收。

## AS2 接入适配及现场状态

仅将任务局部 ENU 显式平移到 PX4 `drone0/odom`，平移量来自起飞前静止里程计：
`[-0.00018211569840786979, -0.0016402003820985556, 0.6906089782714844]`。
目标、虚拟地面/天花板和地图初始原点同步平移，保留原任务几何与规划参数；话题改为
AS2 的 `navigation/odom` / `navigation/points`，不运行旧 LeUAV PX4 控制桥。

保留已有相机开启与多线程传感器镜像；LiDAR1 接口名映射为 Lidar，物理参数为任务规定的
200,000 点/秒、16 线、50 m、360°、竖直 ±15°。相机保留原显式配置，并载入任务 CameraDefaults。
实际 PX4 `MPC_XY_VEL_MAX=16`、`SENS_GPS0_DELAY=200` 已核对。网络端口改为 RPC 41472、
仿真 TCP 4581，避开既有端口；算法/控制仍在 ROS domain 83，AS2 在原私有 RoboPi overlay。

本轮结束后实际 PX4 为 **Hold、Disarmed、landed=True**。SUPER / 导航 / 传感器与板端 AS2
保持运行，adapter 已禁用，没有活动任务参考。原停止的 SUPER 容器保留为
`aerohub-nh-super-inputs-pre-forest8`，配置快照在原始目录，未触碰其他部署或任务。

完整原始 DDS / SUPER 日志：`/tmp/aerohub-forest8-as2-20261105T141215Z`；主要证据见 [summary.json](summary.json)、
[mission.log](mission.log)、[px4-final-state.log](px4-final-state.log)。
