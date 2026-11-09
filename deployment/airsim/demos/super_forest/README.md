# SUPER forest8：PC 验收与历史 RoboPi 链路

四方职责主说明见[项目简介](../../../../docs/项目简介.md#deployment-roles)。
当前 `robopi-all` 的历史验证将 `mission.py` 也放在板端，尚未符合“工作站编排、
算力板提供就绪 ROS 能力”的目标；本页运行方式按其实际实现描述。

当前 PC 验收链为实际 AirSim LiDAR / PX4 状态 → SUPER → 时间化 Adapter →
FollowTimedPath → onboard_computer 中的 AS2 → PX4 SITL / AirSim。
历史 `forest8_as2_mission.py` 属于单点轨迹桥 → RoboPi AS2 的不同运行组合，
保留用于追溯，不与下方 PC 新链路混用。

## 新迁移链路的 PC 入口

验收当前源码时可显式选择本次构建的机载镜像与原生算法工具链，避免沿用旧发布镜像。以下命令只构建，不启动飞行；先从当前 main 构建并记录 `onboard_computer:main-e2e` 的 ID，同时准备 Dockerfile 使用的 `flight_board:v0.1.0`：

```bash
docker build --network none --pull=false \
  --build-arg BASE_IMAGE=onboard_computer:main-e2e \
  --build-arg NATIVE_BASE_IMAGE=onboard_computer:main-e2e \
  -f deployment/airsim/demos/super_forest/Dockerfile \
  -t uav/apps-super-forest:main-e2e .
```

`NATIVE_BASE_IMAGE` 必须提供 Humble、PCL 和算法编译依赖；`BASE_IMAGE` 必须提供兼容的 AS2/PX4 消息运行环境。镜像构建成功不代表真实传感器或飞行通过。运行时还须确认启动入口实际选用了该新镜像，而非默认标签。

本目录新增 `run.py`，基于现有 `onboard_computer:v0.1.0` 构建独立应用镜像，
使用完整 committed polynomial → FollowTimedPath → AS2 → PX4/AirSim。
当前旧 RoboPi 记录与下方 `forest8_as2_mission.py` 仍属于历史单点桥链路，不能互换验收。

```bash
python3 deployment/airsim/demos/super_forest/run.py build
python3 deployment/airsim/demos/super_forest/run.py run --output log/airsim/super-migration-pc-new
```

该命令会执行仿真解锁、起飞、约235m单目标航段、停止与降落。使用独立
ROS domain94、RPC43472、TCP4881/4981、DDS Agent8951及容器前缀
`aerohub-super-migration-`；不停止已有的 `aerohub-nh-super-*`。
先等待权威 PX4 的 Ready for takeoff，再通过真实命令ACK与平台状态确认解锁/Offboard。
不强制解锁，不关闭飞控预检，不更改原任务速度、加速度和observe策略。

本次任务从静止里程计重新测量局部原点，同步平移目标、地图初始原点和虚拟地面/天花板。
原生SUPER保持自身规划、重规划及备用轨迹；Adapter只转换执行请求，不发布运动参考。
成功须同时满足 FollowTimedPath 成功、原任务1.5m到点与低速保持2秒、停止确认及降落解除解锁。

Adapter说明与测试见 [SUPER接入](../../../../src/planner/super/README.md)。
结果写入新输出目录的 result.json；失败、镜像ID和源码哈希均保留。
这不是 RoboPi、混合部署或固定扰动下备用切换成功率验收。

### 原生启动边界修复

SUPER 原生源码中的 PlanFromRest 已直接修复：地图内未占据的实际起点
直接作为轨迹边界，避免先取 0.2m 栅格中心引入位置跳变；起点被占据或超出地图时
仍调用原最近非占据栅格搜索。实际算法源码受主仓库管理，位于
`src/planner/super/ros2_adapter/native/`，Docker 直接编译这里的三个 ROS 2 包。
`upstream/` 固定在原参考提交 `3bfa403`，只用于对照，不参与构建或运行。

启动修复曾提交到 SUPER fork 的 `148116e`，现已完整导入主仓库 native 目录；
后续修改直接在这里提交，无需先推送算法子仓。迁移逐文件核对与已通过 PC 验收的
源码一致，来源与导入哈希见 native/SOURCE.json；这不等于另一次飞行验收。
地图分辨率和 Behavior 的 0.15m 首点连续性门槛保持不变。

新布局提交 `3d8a431` 已在不提供 SUPER 参考源码的干净上下文中离线构建，镜像
`uav/apps-super-forest:repo-native-20261109` 通过框架27项、SUPER8项（无跳过）、
真实节点构造、原生首点及Behavior生命周期回归。原生规划器二进制SHA256与此前完成
forest8的验收版本完全相同；本次布局调整未重复飞行。准确镜像ID见版本化验收摘要。

随后已用该主仓库源码镜像快速复跑完整demo：起飞、航段、Action完成、到点保持、
停止确认、降落解除解锁均通过；到点误差0.285m，航段40.37仿真秒，观测越界0，
峰速8.47m/s（仍非严格8m/s限速通过）。本轮未观测到备用flag2，容器和场景已回收。
证据见摘要中的 `layout_demo_followup`，原始目录为
`log/airsim/super-native-layout-20261109/demo-quick-1/`。

本轮原镜像已复现首点偏移 0.1622m 导致接管拒绝，随后正常降落；隔离回归脚本
`tests/verify_native_start.py` 使用真实 SUPER 节点和明确的合成里程计/平面点云检查
首条原生多项式起点。该检查只是启动边界回归，不能代替完整 AirSim/PX4 飞行。

### 2026-11-09 合并后 PC 验收

基线 `a9ddf15` 的本机首轮在初始连续性检查处失败；加入 `dc4a8d9` 的原生启动
修复后，第二轮完成原 forest8 任务。完整版本、预设判据、失败、镜像 ID 和关键证据
哈希见 [版本化验收摘要](pc-acceptance-20261109.json)。这是修复后单次功能通过。

| 指标 | 本轮结果 |
| --- | --- |
| 实际执行环境 | 新构建的 onboard_computer 派生应用容器；PX4/传感器使用固定 flight_board |
| 接管、Action 完成、到点保持、停止确认 | 通过 |
| 降落解除解锁 | 通过；随后回收本轮容器和场景 |
| 到点误差 | 0.221 m，门槛 1.5 m；速度低于 0.3 m/s 保持 2 仿真秒 |
| 航段时长 / 观测路径 | 42.35 仿真秒 / 270.22 m |
| 观测边界 | 0 个越界样本；不作连续时间内无遗漏的保证 |
| 碰撞 | 本轮未检测到新碰撞 |
| 峰值速度 | 8.94 m/s；高于名义 8 m/s，严格限速未通过 |
| 原生计划与执行参考 | 记录 499 条计划、2538 条参考；2538 条参考均有源曲线候选 |
| 接口和回归 | 框架 27 项、SUPER 8 项无跳过；真实原生启动、节点构造、Behavior 生命周期及采样器检查通过 |

应用镜像为 `uav/apps-super-forest:super-main-pc-20261109-startfix`，ID
`sha256:b30487573240725517560a8a72ee4b60f00f5bc448cfb1a18a0792365f5d379b`。
机载基底由本轮官方 AS2/Pixhawk gitlink 与当前补丁重编，并非旧 v0.1.0 安装直接冒充。
固定 PX4 消息工具链和场景仍是明确的既有前置依赖，不宣称空机无外部资源复现。

原始证据保留在 `log/airsim/super-main-pc-20261109/`，包括两轮结果、DDS 连接与
真实传感器样本、计划/参考、构建与测试日志、`artifact-index.json` 和轨迹/速度图。
原始工件和镜像不随 Git 打包；上述精简摘要受版本管理。重复运行请显式设置
`SUPER_APP_IMAGE` 并使用新输出目录，不能覆盖首次失败。

本轮没有进行重复成功率、受控备用切换、独立占据起点逃逸、RoboPi/OHOS 或混合
部署验收；备用 flag=2 的 15 条消息不代表 15 次独立切换。PVA 候选最佳匹配也不证明
唯一 revision 绑定或所有输入的插值误差上界。

## 历史链路的运行前提与任务入口

旧入口保留任务脚本、场景 / 规划配置和一次完整飞行记录；其准备流程不等同于上方新增的PC镜像入口。`source-profile.yaml` 是原 LeUAV 配置快照，
不能直接作为本仓库的启动配置；下文的现场进程状态也只是历史记录。

运行任务前，需要在已加载 ROS 2 Humble、AS2、配套 PX4 消息、SUPER 消息、
公共轨迹桥及 AirSim sensor adapter 的 PC 环境中准备：

- 使用本目录 `settings.json` 启动匹配的 NH 场景、PX4 / Agent 和 `/clock`。
  此脚本固定访问本机 RPC 41472，使用 ROS domain 83 和 `drone0`。
- 启动 RoboPi AS2、PC 导航输入和 SUPER，使用本目录 `planner.yaml`。
  导航输入为 `/drone0/navigation/odom`、`/drone0/navigation/points`。
- 按[轨迹桥说明](../../../../src/behavior/legacy_setpoint/README.md#super)
  提前启动 `super_as2_adapter_node`，保持禁用，使用 `drone0/odom` 和仿真时钟。
- 为每次任务准备独立输出目录，在执行环境中挂载为可写 `/evidence`；复制
  `source-task.json` 到该目录，并根据本次地面定位重新生成 `origin.json` 中的
  `task_to_odom_translation_m`。同步平移规划配置中的原点、虚拟地面和天花板。
  本目录的 `origin.json` 是历史测量，不能直接当作新一次运行的标定。

在上述 PC 环境中，从仓库根目录执行（会发送仿真起飞及运动指令）：

```bash
ROS_DOMAIN_ID=83 python3 deployment/airsim/demos/super_forest/forest8_as2_mission.py --ros-args \
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

- 原入口：旧工程的 `profiles/super.forest8-ab.yaml`。
- 任务：`cases/super/config/task.forest8-ab.json`，完整字节快照为 [source-task.json](../../../../log/pr-history/deployment/airsim/demos/super_forest/source-task.json)。
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

完整原始 DDS / SUPER 日志：`/tmp/aerohub-forest8-as2-20261105T141215Z`；主要证据见 [summary.json](../../../../log/pr-history/deployment/airsim/demos/super_forest/summary.json)、
[mission.log](../../../../log/pr-history/deployment/airsim/demos/super_forest/mission.log)、[px4-final-state.log](../../../../log/pr-history/deployment/airsim/demos/super_forest/px4-final-state.log)。


## 新链路 PC 场景验证结果

固定镜像 `uav/apps-super-forest:migration-attempt7` 完成 forest8：Action 成功、
原任务到点保持、停止确认、降落解除解锁；到点误差0.162m，航段39.15仿真秒，
观测路径约262.61m，速度峰值8.36m/s。原生备用flag=2出现在12条命令消息中（Adapter去重后5条），不表示12次独立切换，也不是受控故障注入验收。

X最大26.96m，超过任务20m观测边界约6.96m，采样估计持续5.20秒。
按原observe策略记录并完成任务，不能称作严格边界通过或限速保证。
前轮有碰撞失败，根因未确认，不能据后轮成功宣称碰撞问题已根治。

原始计划与执行参考已保存；对接收的原生计划按参考时间做最佳匹配，2327条参考均有
候选曲线，最大P/V/A差约1.91e-6m、5.43e-6m/s、1.71e-3m/s²。
该分析包含插值与发布时序误差，不证明唯一revision绑定或所有输入的误差上界。

完整结果、失败轮次、准确版本和证据哈希见
[migration-pc-validation.json](../../../../log/pr-history/deployment/airsim/demos/super_forest/migration-pc-validation.json)。原始记录保存在
`log/airsim/super-migration-pc-attempt7/`，运行后可执行：

```bash
python3 deployment/airsim/demos/super_forest/analyze.py log/airsim/super-migration-pc-attempt7
```

本轮没有更新 RoboPi 安装，没有验证 OHOS 编译或板端执行。现有
`build_lib.sh`/`deploy_lib.sh` 的组件列表不包含SUPER，不应将PC镜像当作板端产物。

### PC 规划、RoboPi 执行候选入口

`run.py` 默认仍为 `--execution-location pc`。`robopi` 将 SUPER、导航输入和
Adapter 留在 PC，AS2 使用 `/data/aerohub/validation/super-follow-path/as2`。
不编译或部署 OHOS SUPER。配置沿用本机 `.env` 的 HDC、`PC_IP`、`ROBOPI_IP`；
两侧显式生成 DDS 网卡与 peer，domain 94、system ID 61。启动前检查独立端口。

```bash
bash tools/robopi/deploy_lib.sh --component as2 \
  --as2-root /data/aerohub/validation/super-follow-path/as2 \
  --prepare-only log/airsim/super-candidate.tar.gz
# 去掉 prepare-only 后安装候选；默认安装和 .env 保持原样。
python3 deployment/airsim/demos/super_forest/run.py build
python3 deployment/airsim/demos/super_forest/run.py run --execution-location robopi \
  --case ground --output log/airsim/super-ground-01
# ground 不解锁。通过后才执行 short；forest8 保留原任务参数。
python3 deployment/airsim/demos/super_forest/run.py run --execution-location robopi \
  --case short --output log/airsim/super-short-01
python3 deployment/airsim/demos/super_forest/run.py status --output log/airsim/super-short-01
python3 deployment/airsim/demos/super_forest/run.py stop --output log/airsim/super-short-01
```

`short` 使用独立 4 m 目标，只检查接口和恢复，不能替代 forest8。
`stop` 只在运行结果确认降落解除解锁、板端进程身份仍匹配时回收 AS2。
若任务恢复未确认，会保留板端控制、时钟与仿真并报错；此时先恢复并确认状态，
再清理。板端存活由编排监控，运动过程中仍由任务检查状态/时钟新鲜度和控制权。
异常用例分别选择 `--case short-cancel`、`short-input-loss`、`short-takeover`，
在真实轨迹开始后注入取消、SUPER 退出或控制器 HOVER 接管，检查停止反馈、
两秒内不再接受更新或发出轨迹参考，并执行降落解除解锁。用例实现仍须按实际结果
逐项验收，入口存在不能作为异常执行通过的证据。

早期混合拓扑结果见 [RoboPi 验证摘要](robopi-acceptance-20261109.json)：
首轮短程曾在 PX4 Offboard 切换时被拒绝，对时后 short-02 已完成短程闭环。
板端墙钟约慢 2.77 秒；即使消费同一 `/clock`，PX4 平台系统时钟时间戳仍要求
两端墙钟匹配。入口现在沿用既有运行时的 0.5 秒偏差门槛并记录偏差，不自动改时。
临时系统对时是设备级操作，需要明确授权；本轮经用户授权执行后通过同步回归，
不据此宣称持续同步可靠。

2026-11-09 用户授权临时对时后，short-02 已完成板端轨迹执行、停止、降落解除解锁，
到点误差约 0.292 m；这不证明长期时钟稳定。针对间歇漂移疑虑，新增运行中时差
采样（clock-offsets.jsonl）及超限后请求任务恢复的路径，仍使用 0.5 秒门槛。
此为编排层轮询保护，有检测及恢复延迟，不能保证墙钟跳变后的第一帧控制消息即被阻止。
运行中超限回归已用隔离测试验证，实际时钟故障下飞行恢复尚未验收。
更长时段的间歇漂移、时间服务行为与持续同步方案仍待验证。

RoboPi 间歇时钟漂移的模式与触发条件暂不确定，根因未定位。用户要求先记录，
不继续专项排查；随后明确恢复全板端 Demo。异常用例及 forest8 两轮仍未验收。

### SUPER 与执行链全部放到 RoboPi

`--execution-location robopi-all` 把原生 SUPER、导航输入、Adapter、Mission 和 AS2
都放在 RoboPi。PC 只运行 AirSim、模拟 PX4、传感器/时钟桥及验收协调，不启动规划
应用容器。`robopi` 仍表示旧的 PC 规划/板端执行拓扑，`pc` 默认值保持不变。

```bash
bash tools/robopi/build_lib.sh super
bash tools/robopi/deploy_lib.sh --component as2 --include-super \
  --as2-root /data/aerohub/validation/super-follow-path/as2
python3 deployment/airsim/demos/super_forest/run.py run --execution-location robopi-all \
  --case ground --output log/airsim/super-robopi/all-board-ground-new
python3 deployment/airsim/demos/super_forest/run.py run --execution-location robopi-all \
  --case short --output log/airsim/super-robopi/all-board-short-new
python3 deployment/airsim/demos/super_forest/run.py stop \
  --output log/airsim/super-robopi/all-board-short-new
```

每次使用全新输出目录。全板端 `stop` 向所属编排请求恢复，随后用 `status` 与
`result.json` 确认停止/降落；恢复未确认时保留独立中继和执行组件。
板端原始证据位于候选 `runtime/super-runs/<输出目录名>/`，结束后校验归档哈希并
回传到对应 PC `log/` 目录。进程身份、包哈希、板端资源和时差采样均随证据保留。

全板端模式使用专用 PC 网卡提供 AirSim RPC，解锁前从板端验证 ping；同一地址也
用于 PX4 启动中继。候选自带有来源和版权记录的 msgpack 纯 Python 实现，匹配 PCL
所需符号版本的 libpng 也只放候选内部，不替换板端系统库。

滚动更新等待当前首计划实际发布第一条轨迹参考后再开始，避免原生模式切换的
100 ms 阻塞窗口。旧参考不能解除新任务等待，原始轨迹绝对时间及拒绝阈值保留。

本轮全板端快速闭环结果见 [全板端验收摘要](all-board-acceptance-20261109.json)。
all-board-short-04 单次通过，到点误差约 0.205 m、执行 revision 28，完成停止确认和
降落解除解锁。前三轮失败分别涉及 RPC 回环监听、PX4/relay 地址及启动更新抢跑，
均保留证据；未将成功轮当作原 forest8 长任务或长期可靠性通过。
