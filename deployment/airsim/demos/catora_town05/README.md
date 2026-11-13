# CAT-ORA Town05 四机队形展示

配置已确定；已实现独立计算节点启动器 `compute.py` 和工作站四阶段客户端 `mission.py`，
一板三 PC 启动编排和真实 ROS 地面联调已通过；首次解锁尝试在起飞 Action 前被定位新鲜度保护中止，尚未完成 Town05 飞行验收。
`mission.py` 读取 `sequence.yaml` 并逐次提交三个 Planner task；`deployment-plan.yaml`
仍是部署规格，不是可直接启动的共享 swarm 配置。

选用 Town05 初始出生点沿 NED X 负方向约 50 米、停车场旁的四向公路交叉口。
2026-11-12 的真实俯视图显示路口中央没有车辆，建筑、树和路灯在道路外围。
场景采用本地资源 `airsim_town05`；更换场景包后应重新核查路面和坐标。

## 四阶段展示配置

四机在同一高度按“横排起飞 → 菱形 → 竖列 → 方形”展示，中心固定在
AirSim 世界 NED (-50, 0)，起飞高度 3 米，飞行目标 Z=-3 米。
横排沿 Y 轴排列，竖列沿 X 轴排列；从配置的相机方向看，分别对应左右排列和纵深排列。
横排直接通过原生 Takeoff 从四个出生点起飞建立，之后提交三个完整队形变换任务。
四机均到位后保持 4 秒再进入下一阶段，最后以原生 Land 降落并确认解除武装。

| 阶段 | 布局 | 尺寸 | 入口 |
| --- | --- | --- | --- |
| 1 横排起飞 | X=-50，Y=-9/-3/3/9 | 长 18 米、间距 6 米 | 四机原生 Takeoff |
| 2 菱形 | 中心周围四个轴向顶点 | 两条对角线均 16 米 | `task.yaml` |
| 3 竖列 | Y=0，X=-59/-53/-47/-41 | 长 18 米、间距 6 米 | `tasks/vertical_line.yaml` |
| 4 方形 | X=-56/-44，Y=-6/6 | 边长 12 米 | `tasks/square.yaml` |

`sequence.yaml` 保存逐机目标与顺序，`deployment.yaml` 的 PX4 原点与场景 settings
的横排出生点一致。速度上限 0.8 m/s、加速度上限 0.5 m/s²，最小机间距验收门槛 2 米。
CAT-ORA 不承担场景障碍物避让；静态队形间距通过不代表过渡轨迹安全。
Demo 显式设置 `goal_assignment: optimize`，允许原生算法自主分配槽位；通用默认仍为 `fixed`。原生算法的换配结果、共同时间基准、
整个轨迹段的机间距及路面净空需在规划及飞行中核验，不能用逐机 GoTo 替代。

## 一台 RoboPi 与三台 PC 节点

本 Demo 是集中式规划，中央服务仅运行一份，不是每架飞机运行一份 CAT-ORA。
`deployment-plan.yaml` 声明部署职责，尚不是共享 swarm 编排器可直接启动的配置。

| 职责 | 默认位置 | 工作内容 |
| --- | --- | --- |
| drone0 计算节点 | RoboPi | 本机状态适配、AS2、FollowTimedPath 执行 |
| drone1/2/3 计算节点 | PC 三个独立进程或容器 | 各自状态适配、AS2、FollowTimedPath 执行；各有独立监管线程 |
| 中央规划服务 | RoboPi，与 drone0 同机 | 单实例 CAT-ORA 原生算法、proxy、公共 Planner 与四机调用协调 |
| 工作站任务客户端 | PC | 起降、顺序提交三次队形变换、等待全部完成、异常收口 |
| 进程监管及观察 | PC | 启停与健康检查，不转发高频运动控制 |
| 仿真器与飞控 | PC | Town05 / AirSim，四个独立 PX4 SITL 实例 |

任务客户端决定“下一步变什么队形”；中央 Planner 决定“四机如何协同飞过去”；
各机执行节点负责“按轨迹控制本机”；PC 监管程序管理进程存活。
中央服务所在主机与任务客户端所在主机是两个独立配置。

默认采用 `board_center`：中央服务在 RoboPi。另保留 `board_worker` 验收场景：
中央服务移到 PC 的 drone1 节点，RoboPi 只执行 drone0 的轨迹。
两种场景始终恰好一台 UAV 计算节点在 RoboPi、三台在 PC；切换中央位置不复制算法服务。
按照共享部署约定，完整集中式支持的验收须覆盖两种中心位置。

执行链使用扩展 FollowPath 的 FollowTimedPath，保留完整轨迹与共同执行基时间。
旧 NH `run_robopi.py --mixed` 把任务放在板端且使用历史执行链，不能直接视为本 Demo 的入口。

## 默认斜视镜头

按本次用户要求，为 Town05 显式选择 `oblique`，覆盖通常的垂直俯视约定：

- 对准四架飞机实时位置的算术平均中心。
- 相对水平线向下 75°，即偏离垂直方向 15°。
- 位于中心的 X 负方向，朝 +X 看；视线距离 20 米，水平偏移约 5.18 米、高于质心约 19.32 米。`distance_m` 参数表示水平偏移，不是视线距离。
- 相对中心高约 6.43 米；飞行高度 3 米时相机约在路面上方 9.43 米。
- 视场角 60°，观测图像 1024×1024，当前跟踪频率 1 Hz。

初始主视口的 CameraDirector 与外部 `swarm_observer` 相机采用同样的位置和姿态。
实时跟踪通过共享 `deployment/swarm/overhead_view.py` 更新外部相机，显示在默认开启的
观测子窗口中；主视口是 Manual 模式，主视口动态跟随没有接入。
外部镜头实际构图、建筑遮挡与运行稳定性仍需有屏预览确认。
机群变换期间的中心变化会触发外部相机更新，不用初始固定相机冒充实时跟踪。

## 场地预览

使用已安装 `msgpack` 的 Python，配置本地 `airsim_town05` 资源后，在仓库根目录运行：

```bash
python3 deployment/airsim/demos/catora_town05/preview.py check
AIRSIM_RENDER_MODE=windowed python3 deployment/airsim/demos/catora_town05/preview.py start
python3 deployment/airsim/demos/catora_town05/preview.py status
python3 deployment/airsim/demos/catora_town05/preview.py stop
```

预览只启动场景和相机跟踪，不启动 PX4、AS2、Planner 或任务，不解锁。
端口复用 NH 四机 Demo 的 RPC 42471 / PX4 TCP 4680..4683，两者不可同时运行。
预览 `start` 或 `status` 成功不代表演示飞行通过。

## 验证状态

配置和相机几何的离线检查：

```bash
python3 -m unittest deployment.swarm.tests.test_swarm deployment.airsim.demos.catora_town05.tests.test_configuration -q
```

选址证据位于 `log/airsim/catora-town05-survey/`：`overview.png` 为全景，
`road-candidate.png` 为路口近景；`road-run/` 保留高清采集配置及引擎日志。
图像通过移动观测相机取得，未移动无人机或发送起飞指令，尚无本次斜视镜头截图。

初次启动被缺失材质提示阻塞，profile 加入 `-unattended` 后能加载
`/Game/Carla/Maps/Town05` 并取得图像，但缺失材质仍存在。
随后场景曾在 RPC 服务中因 `close: Bad file descriptor` 崩溃。
1 Hz 跟踪尚未证明能解决崩溃。

完整飞行 Demo 的当前状态以文末记录为准；初始待办包括 Town05 RPC 长期稳定性、起飞阶段定位时间诊断与真实多阶段执行、
真实 PX4 出生点/路面高度、轨迹全段净空与间距、逐阶段到点保持、故障收口、
落地解除武装及资源清理。当前不声明可直接公开演示飞行或复现成功。


## 2026-11-12 本轮进展

- 真实 Town05 有屏四 PX4 地面预检通过：四机约 15.7 秒全部 Ready for takeoff，
  再观察 30 秒并清理；没有解锁。证据：
  `log/airsim/catora-town05-ground-20261112-02/{px4-startup,report}.json`。
  初轮因工作会话缺少 DISPLAY 未启动，证据单独保存在 `...ground-20261112-01/`。
  清理期间 AirSim world-update 异常原样保留；不能把单轮短检查作为长期稳定性结论。
- 真实 CAT-ORA 固定输入规划检查通过：三次变换均保留逐机目标，
  最小采样间距分别约 5.62 / 5.62 / 5.37 米，速度与加速度上限满足浮点容差。
  证据：`log/airsim/catora-town05-plan-probe-20261112/run-02/`。
  这不是空中轨迹跟踪或连续安全证明。
- 板端已有隔离候选 AS2 的 FollowTimedPath、Planner 类型和 CAT-ORA adapter
  加载通过。默认 AS2 安装不含此扩展；未替换默认安装，未修改其他任务。
- `mission.py` 使用已有 NH 任务的测量/服务/Action 辅助逻辑，禁止启动算法或 AS2；
  逐阶段重置 task_id/参考计数/执行反馈，验证完成和保持后才进入下一阶段。
  任务配置和校准后的 deployment 路径必须在中央规划主机可读，且与工作站本地
  校准副本一致。取消结果不确定时不盲重发 START；恢复独立逐机尝试 Land。
- `compute.py` 每实例启动一机 AS2/状态适配，可显式选一实例承载中央服务，
  不启动 Mission 或访问仿真 RPC。现已在真实地面通过共享 swarm 的独立线程监管与 readiness 检查，完整飞行仍未通过。
- ROS 镜像内客户端导入及两项恢复 Mock 测试通过，不代表真实 ROS 接口调用通过。
- 当前运行缺口：PC 有线网卡无项目 IPv4 地址，HDC 传输正常但 DDS 物理网络不可达；
  需由有主机网络权限的操作者恢复本地配置。板端地址已有配置，无需额外改板端。
  网络恢复后仍须验证时差、overlay ABI/来源、四方启动与 ground preflight，再解锁。

地面场景/飞控检查（无 AS2，无运动）：

```bash
AIRSIM_RENDER_MODE=windowed python3 deployment/airsim/demos/catora_town05/preflight.py \
  --output log/airsim/catora-town05-ground-new
```

工作站客户端只应在中央服务和四机 AS2 已就绪、完成地面校准和网络/时钟检查后运行。
参数 `--compute-root`、`--compute-deployment` 使用中央计算主机上的路径，
`--local-deployment` 使用同一校准配置在工作站的本地副本。
先用 `--preflight-only` 确认不解锁的 Action/定位检查；未交付完整部署入口前，
单独启动客户端不会自动补齐运行环境。

## 已接通的一板三 PC 入口

在已配置的本地环境中，加载 `tools/robopi/load_env.sh`，使用含 msgpack/PyYAML 的主机
Python。现有候选 AS2、CAT-ORA 安装、PC计算镜像、标准工作站接口overlay和本地
msgpack fallback均需存在；入口不重建镜像、不替换持久安装。

```bash
# 仅地面：四飞控、真实一板三PC能力、15秒静止配准、12 Action、60秒观察；不解锁
AIRSIM_RENDER_MODE=windowed python3 deployment/airsim/demos/catora_town05/run_ground.py \
  --pc-overlay log/airsim/catora-offboard-fix/pc-ws \
  --board-as2-root /data/aerohub/validation/catora-offboard-fix/as2 \
  --board-framework-root /data/aerohub/validation/super-follow-path/as2/install/uav_planner_framework \
  --output log/airsim/catora-town05-mixed-ground-new

# 显式飞行：引用已通过的混合地面证据；本轮仍重新配准和做地面门控
AIRSIM_RENDER_MODE=windowed python3 deployment/airsim/demos/catora_town05/run.py --flight \
  --pc-overlay log/airsim/catora-offboard-fix/pc-ws \
  --board-as2-root /data/aerohub/validation/catora-offboard-fix/as2 \
  --board-framework-root /data/aerohub/validation/super-follow-path/as2/install/uav_planner_framework \
  --ground-evidence log/airsim/catora-town05-mixed-ground-new \
  --output log/airsim/catora-town05-flight-new
```

`mission/` 为不解锁的配准与预检；`flight/` 为实际飞行任务。配准副本通过部署层送到
中央计算主机，任务客户端只经ROS发起飞控/Planner调用。PC三个worker复用共享
ServiceThread，各自监管独立计算容器。地面和任务前各执行一次有线UDP时差门控。
这些点不能替代运行中时差故障测试，`board_worker`场景也尚未接入当前入口。

若生成 `recovery-required.json`，入口保留能力、时钟、仿真及中继，等待实际恢复，
不自动清理可能仍解锁的飞机。不要为了结束入口伪造disarmed结果或停止场景。

当前最新地面证据 `log/airsim/catora-town05-mixed-ground-20261112-06/` 通过。
此前02/03轮Town05 RPC崩溃、04轮时钟探针超时、05轮原点门控失败均保留。
修复后的Demo时钟桥仍以50Hz运行，改用连接复用；相机先合成世界NED再求质心。
06轮约90秒未见同类崩溃只是有限地面证据，不代表长期可靠或完整飞行成功。


## 首次解锁尝试：未通过，已安全恢复

`log/airsim/catora-town05-flight-20261112-01/validation-summary.json` 是本轮结论。
重新地面配准、观察和时钟门控通过后，drone0 已解锁，在切换 Offboard 时被
`stale/future local pose: drone0` 中止，**尚未发送 Takeoff Action**。
其余三机没有解锁，三个 CAT-ORA 队形任务也没有开始。

自动恢复确认 HOVER，但原生 AS2 Land 被拒绝。保留能力、clock和仿真后，
独立检查发现PX4仍Armed/Offboard且land detector未判落地；未强制disarm。
随后通过PX4原生Land应急恢复，确认landed=true，再确认四机原生commander
均Disarmed及ROS反馈解除武装。此应急恢复不算AS2 LandBehavior通过。
最初恢复失败的内容保存在`failure-stop-initial.json`，后续恢复另记录说明；
最终`report.json`仍为passed=false，板端21个本任务进程回收后remaining={}。

状态探针另发现VehicleStatus的DDS载荷88字节与读端87字节不匹配。
这是必须核对的接口兼容性缺口，尚不能直接判为定位时间异常的根因。
下一轮前须对齐目标PX4/消息定义，采集精确clock/pose stamp/接收时间及Offboard
切换时序，保留现有新鲜度门槛。单纯发现12个Action不足以证明所有消息ABI一致。

## 定位停更已修复并通过两侧地面与起飞复验

见 [Offboard 状态停更诊断](DIAGNOSIS-state-pause.md) 和
`state-pause-diagnosis.json`。不是RoboPi独有：给PC的drone1发送同一Offboard请求，
也会停更约1.2秒；只对目标机触发，PX4原始里程计、clock和其他三机正常。
共用AS2 Pixhawk服务回调的120×10ms预发送循环阻塞了单线程执行器。
原始诊断轮保持四机disarmed且资源已清理。随后已将预热改为定时器驱动的延迟响应，PC/RoboPi 隔离构建与地面复验通过；四机原生起飞和降落也已通过。具体补丁、测试与证据见诊断文档。

### 槽位分配与记录

三段变换将配置中的 `targets` 作为队形槽位集合。轨迹行与 `vehicle_id` 的对应关系保持不变，不按目标槽位重编号。Adapter 校验末点是槽位集合的一一排列；框架仅在完整轨迹通过 Gateway 校验后，将完成判据绑定到该机实际末点，并拒绝同一任务中途换绑。

算力节点保留 `compute/resolved/<task_id>.assignment.json`，工作站从可靠、持久的 `/planner/goal_assignment` 获取相同记录；各阶段结果保存请求槽位、实际目标和绑定。该话题用于审计与验收，轨迹仍由完整 FollowTimedPath 执行。

2026-11-13：非阻塞 Offboard 修复后的实飞已完成四机原生起飞和横排保持；首次菱形受旧固定目标检查拒绝，随后四机原生降落并确认 disarmed。本轮自主槽位配置的完整飞行结果另见运行报告，不能据此提前声明整个 Demo 通过。

Demo 为悬停交接使用更严格的 `timed_position_tolerance=0.08m`（执行层原默认 0.25m）；阶段验收仍为状态误差≤0.6m、真值≤0.75m、连续4秒。`hold-errors.jsonl` 保留逐次误差。该参数通过当前源码 launch 显式传入，不改变算法输出和轨迹时间基准。

## 当前结果：正常 Demo 单轮通过（2026-11-13）

`log/airsim/catora-town05-flight-20261113-04/validation-summary.json`：一板三PC、板端中心节点，完成四机原生起飞→横排→菱形→竖列→方形→原生降落/disarmed；各阶段保持超过4秒。最大阶段结束状态误差0.568m、真值0.610m，最小观测真值间距5.459m。板端任务进程清理后为空。三份板端绑定记录与工作站完全一致，汇总在 `goal-assignments.json`；各阶段截图位于 `flight/`。

此前03轮菱形轨迹虽完成，但保持超时，记录保留为失败；04轮收紧执行到点容差后通过，验收门槛不变。此结论仅覆盖正常演示单轮，未覆盖 board_worker、完整故障矩阵或长时可靠性。
