# CAT-ORA Town05 Demo 执行记录

## 本轮目标与正式 Skill

复用已有 CAT-ORA、FollowTimedPath、启动首包保护与共享 swarm 监管；四机横排起飞、菱形、竖列、方形，一板三 PC，工作站通过 ROS 调用算力服务。
已完整读取并按职责使用 `uav-sim-e2e-validation`、`hdc-helper`、`deploy-lib`、`uav-component-deployment`；部署按需执行，不默认替换已安装组件。
Town05 RPC 崩溃诊断另读取 diagnosing-bugs，先建立实际场景反馈循环，不凭堆栈直接修改其他组件。

## 预声明检查矩阵

- 环境：板连接/现有安装与镜像/占用/墙钟偏差 <=0.5s；失败不得起飞。
- 地面：Town05 有屏加载、15°实时观察、四 PX4 Ready for takeoff、新鲜状态与时钟、各机 Action 和中央 Planner 就绪。
- 飞行：四机 Takeoff 3m；横排保持4s；依次完整轨迹变菱形/竖列/方形，各保持4s；速度配置0.8m/s，加速度0.5m/s²。
- 阈值：端点本机状态误差<=0.6m、独立真值误差<=0.75m；飞行最小机间距>=2m；无空中障碍碰撞；全部Land/disarmed。
- 覆盖：默认 board_center、board_worker；正常/取消与故障收口分开记录，未执行不得写通过。
- 证据：运行版本、镜像ID、配置、节点与接口、逐阶段结果/轨迹/真值、清理身份进入 log/airsim/catora-town05-*。

## 开始状态

前两轮只完成选址、四阶段配置和离线几何测试；缺少实际四方多阶段任务入口。历史 mixed 入口任务在板且 worker 使用 legacy_setpoint，不能直接执行后宣称满足本次要求。
开始只读环境检查，不解锁，不替换部署，不调系统时钟。


## 实际执行结果

1. HDC 在线；CAT-ORA 持久安装存在。默认 AS2 缺少 FollowTimedPath，但已有隔离候选。
   按候选环境加载并只追加 CAT-ORA native/adapter 前缀，成功导入 FollowTimedPath、
   ControlPlanner、CatOraAdapter；未替换任何安装。
2. PC 镜像 `uav/apps-catora-open-field:migration-main-behavior` 的扩展及接口加载通过。
   标准工作站镜像存在；本轮只在算法镜像做离线客户端导入/Mock测试，尚未准备标准
   工作站接口 overlay，不能声称四方任务已部署。
3. Town05 ground-01 因 DISPLAY 缺失失败，无场景/飞控起飞。
   ground-02 使用实际桌面启动四 PX4，15.664s 四机就绪，30秒观察通过并清理。
   镜像ID和返回码在 report.json；world-update异常发生在清理期间，日志原样保留。
   预检代码在该次运行后增强 cleanup 失败判定；无新一轮真实地面运行验证这处增强。
4. 固定输入调用真实 CAT-ORA：run-01检查端点/间距/速度通过；run-02另查加速度、
   有限数值与采样包络通过。最小采样间距5.618/5.618/5.367m；不代表连续或飞行安全。
5. 新增 compute.py、mission.py：本机配置命令4项检查通过；ROS镜像内两项恢复Mock
   测试通过，取消和HOVER失败不阻止Land尝试。没有实际接口联调或起飞。
6. 用户确认网线已接并授权按需配板端地址。板端已有IPv4，PC只有有线IPv6。
   主机临时加IPv4分别因sudo需要密码、NetworkManager权限不足被拒；没有提权绕过，
   没有改变主机/板端网络或系统时钟，已交接最小PC命令。无线与默认路由保持不变。

## 当前结论

整体未完成；未起飞、未跑一板三PC ROS联调。网络恢复后继续原矩阵，不能从固定输入
算法检查和30秒地面就绪推断完整Demo成功。当前无本任务容器、场景或板端节点需要保留。

## Skill 回顾

已更新 uav-sim-e2e-validation：复用Town05地面/固定输入规划探针，明确证据范围，
多阶段反馈需按task隔离。已更新hdc-helper：HDC与DDS可达分开，检查两侧地址与
主机权限，不把PC缺IP错误修在板端。deploy-lib与component-deployment本轮仅用其
只读部署核查与角色边界，无实际部署、没有新增构建/安装事务经验，故无需修改。

## 网络恢复与真实混合地面联调（继续）

- 用户通过桌面配置后，PC有线地址/路由正确，3/3 ping通过，约0.6ms。
- 已获用户明确许可对板端系统时间。旧 sync-time 把PC时间提前取值后经HDC传输，
  执行后仍约-1.44s；直连UDP探针最小RTT约0.36ms，实测校正前约-1.555s，
  通过板端相对修正后15次测量通过，最小RTT样本偏差约-0.045ms。
  UDP证据在 `log/airsim/catora-town05-network-clock/udp-clock.json`。
  临时探针已关闭。后续runner每轮执行只读UDP时差检查，包含RTT不确定性，门槛仍0.5s。
- 复用标准工作站镜像和已有SUPER的消息overlay；本地msgpack纯Python fallback挂载到
  Demo外部runtime，不改共享镜像。工作站客户端导入通过。
- 新增run_ground.py，复用ServiceThread，每个PC worker独立线程/进程，板端仅一机AS2与
  单实例CAT-ORA/Planner；任务仍在标准工作站镜像。只读检查后复用现有候选，不覆盖安装。
- mixed-ground-01真实地面通过：12个Action、中央Planner、四机定位可用；未解锁。
  板端回收21个本任务进程，remaining为空。首张斜视图发现镜头没有四机。
- 相机问题：RPC位姿是相对各自出生点，跟踪器直接求平均导致非零出生位置场景错误。
  增加出生点到世界NED转换；测试覆盖平移后质心(-49,0,-3)，保留默认向下模式。
- mixed-ground-02仍触发Town05原有RPC close Bad file descriptor崩溃，未解锁。
- 新增PersistentRpc可选客户端，不改变原Rpc默认。相机与Town05工作站采用连接复用；
  本机真实TCP回归验证三次调用共用一连接，以及断连不重发请求，通过。
- mixed-ground-03相机中心日志已正确(-50,0,约-0.056)，但仍在RPC socket关闭栈崩溃。
  进一步确认clock.py原实现每50Hz创建/关闭连接，前一轮没有覆盖这个高频来源。
- 新增Demo clock.py，保持原50Hz逻辑，仅替换为PersistentRpc，通过外部源码挂载加载。
  mixed-ground-04正在执行相同60秒地面对照；执行前冻结源码快照，不边跑边改脚本。
- run.py新增显式--flight入口与前序地面证据检查。尚未启动飞行；disarm无法确认时
  保留能力、时钟与场景，不自动清理模拟器。此路径待真实飞行检验。

## 对照结果与首次飞行入口

- mixed-ground-04在只读UDP探针阶段超时，未启动场景；独立脚本方式替代内联多行命令。
- mixed-ground-05时钟门控通过（约9ms偏差），50Hz clock使用连接复用，场景未见RPC崩溃；
  但原点检查拒绝drone0，未解锁。没有放宽0.75m门槛，补齐旧Demo的静止15秒与地面配准。
- mixed-ground-06通过：12 Action、中央Planner、配准后误差检查、额外60秒地面观察。
  相机日志91次更新，世界质心(-50,0,约-0.057)，退出期间一次超时保留；无崩溃。
  板端21个本任务进程清理后remaining={}。各PC进程与线程身份在pc-workers.json。
- 启动首次flight-01：新的run.py显式--flight；先重新ground注册/观察，再由部署层
  通过HDC传同一校准文件到中央Planner，工作站使用本地副本。不经工作站转发高频控制。
  进入flight客户端前再次只读测墙钟，原门槛0.5s；首次arm前写arming-intent供恢复判断。
  飞行过程中未确认disarm则保留能力/clock/scene，避免自动回收导致锁步冻结。
  状态：正在执行，不能提前声明飞行通过。

## 首次解锁尝试最终结果

flight-01：重新地面配准/60秒观察、只读墙钟检查均通过；drone0解锁后切Offboard时
因stale/future local pose中止。Takeoff Action未发送，后三机未解锁，三个Planner任务未开始。
原生AS2恢复确认HOVER，但Land被拒；保留runtime而未清理。
独立恢复第一次正常disarm收到PX4临时拒绝；随后AS2状态机报告DISARMED但raw控制标志仍armed。
以PX4 native commander/land detector核对：实际Armed/Offboard、landed=false，未强制disarm。
执行PX4 native Land后landed=true、四机commander均Disarmed；ROS独立观察确认disarm，才释放清理。
原失败恢复保存failure-stop-initial.json，后续确认保存flight/failure-stop.json并注明PX4应急恢复。
报告依旧passed=false。板端21个本任务进程回收remaining={}；运行入口以失败退出。

发现独立VehicleStatus探针DDS载荷88 vs 87字节不匹配，类型定义需核对，不能将其直接当成
定位异常的已证实根因。定位保护仍为原0.5s；没有放宽，没有启动下一轮飞行。
完整摘要、源码快照、指令及后续恢复脚本、镜像ID、校准配置、SHA256索引均在flight-01。
本轮修改了验证Skill中的出生点/世界相机、静止配准、50Hz RPC连接复用与UDP时钟经验；
下面补充本次原生恢复边界及消息ABI检查。无新Skill目录，无commit/push。

## RoboPi定位专项诊断计划

使用diagnosing-bugs构建实际状态链反馈循环；配合正式hdc-helper、uav-platform-sensor-adaptation
（含airsim-px4参考）及uav-sim-e2e-validation。先只读地面基线：PC/板端分别采集PX4
VehicleOdometry、AS2原始odom/self_localization、CAT-ORA local_pose、/clock与输入拒绝统计。
原始header stamp与接收时间分别记录；CAT-ORA验收门槛仍[-0.1,+0.5]秒，预热15秒后评估。
PC三机为同时对照，板端同机探针可区分发布前与跨机接收异常；消息字段定义一并导出。
若静态基线未复现，再仅对明确disarmed的drone0请求Offboard开/关，不解锁、不发Takeoff，
有界复现控制服务切换与定位发布的关系；不直接开展第二次飞行。

## 定位停更诊断结果（2026-11-13）

- state-baseline-20261112-01：静态基线四机均通过。板端CAT-ORA状态约16–17Hz，
  接收时age最大约39ms。接收消息数与唯一stamp分别统计，不以100Hz重复帧冒充100Hz测量。
- state-offboard-20261112-01：保持disarmed，仅向drone0发送Offboard=true；服务约1.203s。
  仅drone0的新鲜度失败（848次检查违例），odom/self_localization/local_pose均停约1.204s；
  原始PX4最大接收间隔14.3ms，clock23.6ms，PC三机local_pose约70ms。
  板端同机trace中同样出现约1.204s停更，说明不是只在跨机传输/PC观察端停顿。
  两侧probe启动时刻不同，必须按wall_ns对齐；板端15s elapsed预热窗口会漏掉关键事件，
  因此保留full-trace warmup=0分析。前两轮板端最终report被清理中止，原始trace覆盖事件，
  不将缺失report的板端检查声明完整通过。
- 首次PC对照启动被工具自动审批额度限制阻止，未执行；用户continue后补跑
  state-pc-offboard-20261113-01：对PC drone1执行同样的disarmed Offboard请求，约1.203s；
  仅drone1失败（849次检查违例），local_pose最大接收间隔1.200s；此时RoboPi drone0约67ms，
  原始PX4最大间隔14.0ms，clock23.3ms。四机armed反馈始终false。
- 源码对应：PixhawkPlatform::ownSetOffboardControl中100Hz循环120次（sleep约1.2s），
  平台main调用as2::spinLoop的SingleThreadedExecutor，同节点里程计与状态回调一起停顿。
  已确认是共用平台实现的阻塞行为，不是RoboPi硬件特有或CAT-ORA规划计算停算。
- Offboard=false当前实现拒绝；探针记录success=false，不将它写成模式恢复成功。
  本轮无arming/Takeoff请求，结束按拥有身份清理；三个场景均stopped，板端剩余进程为空，
  无本Demo容器残留。
- 新增DIAGNOSIS-state-pause.md与机器摘要state-pause-diagnosis.json；各轮完整工件SHA256已保存。
  当前状态diagnosed_not_fixed，没有修改AS2平台实现、没有替换镜像或板端安装，没有飞行复验。
  若进入修复，应保留PX4超过1s预发送要求、0.5s定位门槛，先让两侧真实地面红色回归变绿。

诊断收尾：补齐离线trace分析的两个算术回归，连同配置/节点命令/共享相机测试共19项通过，
没有把离线测试算作ROS或飞行验证。更新正式uav-sim-e2e-validation，提炼disarmed双侧
Offboard对照、单线程阻塞识别，以及多侧时序对齐/断流age区别。平台传感器Skill提供数据契约，
本次没有改变传感器、坐标或采样模型，故无需改其正文。hdc-helper本轮没有新增独立经验。
AS2运行源码未改，三轮场景均停止、每轮板端22个本任务进程回收且remaining={}，无容器残留。

## 非阻塞修复开始（用户明确要求解决）

采用ROS Humble deferred service response，而非增加线程或放宽定位门槛。
在基类增加不改变成员布局/虚表的protected内联服务回调替换入口，Pixhawk在本地10ms
wall timer上完成至少120帧、至少1200ms预发送，再完成原Offboard状态切换并回复请求。
重复请求/false请求明确拒绝；3s超时、>500ms发送空隙、原已armed后失去armed、显式disarm、
kill/stop及shutdown会结束挂起握手。维持默认互斥callback group，不引入线程竞态。
现有PC镜像的基类头文件与修改前源码完全相同，仅Pixhawk消费者需要使用新头文件重编；
类成员/虚表不变。先构建PC独立overlay，再跑原disarmed红色回归；板端独立候选另行构建部署。

## 非阻塞修复验证进展

- PC独立overlay首次真实地面回归通过：Offboard服务仍约1.214s，AS2 odom最大接收间隔
  16.85ms，CAT-ORA local_pose约71.64ms，新鲜度违例0。证据pc-offboard-fix-20261113-01。
- 实际平台+合成DDS生命周期测试callback-test-01捕获“已disarmed的disarm未取消挂起请求”，
  保留失败；增加基类protected arming callback注册入口，显式disarm即取消挂起握手，
  不依赖ownSetArmingState是否会被基类调用。新增紧急stop/kill锁存，需新平台实例才能重开握手。
- 修订版PC与OHOS均补编译通过，基类仅增加非virtual内联注册方法，无新增基类数据或虚表，
  Pixhawk节点及其静态库作为受影响消费者一并重编。修改前PC/target基类头文件完全一致。
- callback-test-02通过：120帧，1.207s，odom最大间隔12.53ms；duplicate/manual拒绝保留，
  disarm取消、kill取消及禁止新握手通过。后补shutdown/timeout隔离测试通过，无延迟模式命令。
  合成DDS测试不等同仿真飞行。
- 私有SDK完整复制后构建，未改共享SDK。候选部署至/data/aerohub/validation/catora-offboard-fix/as2，
  使用现有安装器的架构/依赖/哈希/占用/事务校验，默认AS2与SUPER候选保持原状。
  仅追加旧候选中的纯Python Planner前缀，不混入其AS2库；topology.json显式记录来源。
- RoboPi真实地面复验robopi-offboard-fix-20261113-01通过：服务约1.206s，AS2 odom最大间隔
  17.07ms，CAT-ORA约73.64ms，违例0，四机disarmed；清理22个板端本任务进程后remaining={}。
- 补丁分别保存在deployment/platform/patches/as2-core-offboard-nonblocking.patch、
  pixhawk-offboard-nonblocking.patch，原submodule checkout保留未提交修改，未commit/push。
- 已启动两侧修复后的flight-20261113-02，先重新配准/观察/时钟门控；结果尚待运行，
  不提前声明Demo成功，其他集中式中心角色和故障矩阵仍未覆盖。

- flight-20261113-02：四机原生起飞与横排保持通过，菱形因 CAT-ORA 交换中间两机目标而被固定绑定策略拒绝；四机原生降落、disarmed、清理通过。用户随后明确授权自主分配并保留绑定。
- 新增显式 goal_assignment=fixed|optimize，默认 fixed；Demo 单独启用 optimize。沿用原轨迹 vehicle_id 和共同时间基准；校验端点槽位一一排列，在 Gateway 接受后更新完成判据并发布/落盘审计。
- 验证：ROS 框架 37 项通过（含接受计划绑定、拒绝计划无审计、中途换绑拒绝），Demo ROS 8 项通过；原生 CAT-ORA 三段连续规划通过，下一段使用各机实际前段末点。固定输入检查不代表飞行验收。
- flight-20261113-03 已启动，两侧继续使用修复后的 C++ 隔离 overlay；本轮只更新 Python 包与 registry，不重建共享镜像。

- flight-20261113-03：菱形自主槽位轨迹执行成功，审计记录 drone1→slot drone2、drone2→slot drone1；工作站保持阶段超时，不能计为通过。后续状态记录显示悬停交接后误差缓慢漂到约 0.6m；四机原生降落且 disarmed，板端 remaining={}。原运行没有逐次记录保持阶段真值误差，不能断言哪一项最先打破连续保持。
- 后续复验仅收紧执行到点容差 0.25→0.08m，验收状态0.6m/真值0.75m/连续4s不变。源 launch 暴露已有 C++ 参数，Demo 选择当前源 launch；无 C++ 重编译。新增 hold-errors.jsonl 保存逐次状态/真值误差。launch --show-args 与8项 Demo ROS测试通过；flight-20261113-04已启动。

- flight-20261113-04 完整正常 Demo 通过：横排/菱形/竖列/方形各保持4.005–4.020s；阶段结束最大状态误差0.5683m、真值0.6097m，最小观测真值间距5.4595m。四机原生AS2降落且disarmed，板端21个任务进程清理后remaining={}。
- 三份板端assignment.json与工作站阶段审计逐项相同；goal-assignments.json汇总每机绑定，validation-summary.json与artifact-hashes.json保存结论及校验。框架ROS37、Demo ROS8、源码补丁2及原生连续规划3段通过。
- 仅确认 board_center 正常演示单轮通过；board_worker、完整故障矩阵、长时间可靠性未执行，不能声明全矩阵复现成功。
- Skill回顾已完善 as2-extension-integration（非阻塞服务、取消及槽位绑定）、oh-ros-build（锁定源码补丁重放与显式Python overlay）、uav-sim-e2e-validation（红灯保留、两侧复验、真实末点续规划及审计）。依据分别为原始停更样本、回调/补丁回归和flight03/04；更严格到点参数在flight04单轮得到验证，不推广为普遍可靠性结论。
