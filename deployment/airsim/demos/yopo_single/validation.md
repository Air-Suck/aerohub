# YOPO 单机验收（2026-11-10）

单宿主 NVIDIA 共置四角色，AirSimNH + PX4 SITL，`plan_from_reference=True`。正常任务均使用2 m/s原生速度缩放、5 s候选时长、Demo独立横向悬停增益。不是原Demo默认measured分支、SUPER原forest8长任务、跨物理设备或真机验收。

## 已通过的分层检查

- 锁定YOPO `bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132`，默认权重SHA256 `09ec31094ed83e09702efc1facf18076564ebbe1afc164960c41036e16ba229f`。从仓库外通过公共选择入口执行locked uv准备、权重校验和CPU/CUDA预热通过；用户YAML未改写。证据`log/yopo-single/setup-outside.txt`。
- 原ROS入口预处理AST对照、非零速度/加速度和非恒等姿态下全部15候选/评分对照：默认6 m/s CPU、2 m/s CPU/CUDA均通过，atol=1e-5、rtol=1e-4。恢复uint8量化、inpaint、原normalize/prepare/CPU primitive解码，ROS单位/stride/端序转换另验。证据`model-baseline*.txt`。
- 五次曲线与原Poly5Solver全域PVA对照、实际Behavior C++采样器对照、yaw跨±π、未来交接与确认/拒绝时间线、5.000000000000001秒纳秒量化回归共6项通过。证据`core-final.txt`。
- ROS格式/时间契约6项、公共模型选择3项、runtime忙槽/取消2项通过。runtime在受限沙箱中挂起，实际运行权限下2/2通过；挂起的本任务测试已清理，未据此修改公共runtime。证据`contract-final.txt`、`runtime-actual-permissions.txt`。
- Humble四包及FollowTimedPath扩展在稳定onboard镜像自身AS2头文件/库上编译并实际加载。真实DDS测试覆盖图像5Hz、定位50Hz、定位80ms延迟、动态TF再落后40ms、严格同stamp权威Pose/body Twist配对、修改拒绝、超时、迟到响应丢弃、目标更换、幂等取消与停止。业务时钟回退使用注入读数验证，**不是实际/clock回退动力学验收**。证据`build.txt`、`behavior-build.txt`、`ros-planner-final.txt`。
- 原生AS2双容器只读查询重新通过30/30，中位3.34ms、p95 3.67ms；该基准不替代GPU模型链。证据`as2-baseline.txt`。

## AirSim + PX4 SITL 动力学

门槛在运行前固定：实际距离<5 m；停止应答、控制器悬停/位置模式、状态新鲜、速度<0.2 m/s连续1 s（ROS时间），10 s（墙钟）内完成；随后原生Land并确认disarmed。未放宽连续性门槛，不以后移时间戳补救迟到计划。

| 用例 | 轮次 | 接纳revision | 停止耗时/墙钟 | 低速保持/ROS | 结果 |
| --- | --- | --- | --- | --- | --- |
| 默认起点开放方向目标 `[0,15,2]` | sitl-15 | 21 | 5.40s | 1.038s | 通过 |
| 同上 | sitl-16 | 21 | 5.40s | 1.041s | 通过 |
| 同上 | sitl-17 | 21 | 5.36s | 1.020s | 通过 |
| 原森林起点短程目标 `[0,15,2]` | sitl-18 | 23 | 5.42s | 1.041s | 通过 |
| 同上 | sitl-19 | 23 | 5.42s | 1.038s | 通过 |
| 同上 | sitl-20 | 23 | 5.48s | 1.020s | 通过 |
| 人工ROS取消 | sitl-21 | 3 | 1.12s | 1.116s | CANCELED，通过 |
| 相机输入陈旧：仅暂停sensor容器 | sitl-25 | 3 | 1.78s | 1.014s | STALE_OR_INVALID_INPUT，通过 |
| GPU worker退出 | sitl-26 | 3 | 1.94s | 1.041s | WORKER_UNAVAILABLE，通过 |

以上9轮均原生Land成功、实际平台disarmed确认、任务碰撞检查未发现碰撞。正常两组各3/3，故障各1/1。ROS图中运动参考发布者均为原生Behavior/扩展FollowPath，未发现Planner/Inference绕过Behavior发布控制。正常到达不要求精确停在目标点，故障测试不要求到达。

森林组沿用SUPER场景已知起点（未复跑其原长任务），起飞后的真实DepthPlanar水平带各观察到15m内障碍，保存原始`depth-before-goal.f32`及尺寸/时间元数据；排除1.2m内近距返回作为障碍条件，以免把机体近距回波算作环境障碍。任务路线确有绕行，但此短程结果不证明任意障碍、长航程或精确到点能力。

精简机器可读结果、镜像ID、模型配置和门槛见[validation.json](validation.json)。逐轮`mission.jsonl`保存状态、姿态/速度与时间诊断，`plans.jsonl`保存曲线和接纳应答，`result.json`保存任务与恢复结论；故障另有注入/恢复标记。原始证据根为`log/yopo-single/`，可用：

```bash
python3 deployment/airsim/demos/yopo_single/analyze.py log/yopo-single --output log/yopo-single/summary.json
```

## 保留的失败与限制

- sitl-01：就绪/时序与目标接纳问题；起飞和Land后disarmed。sitl-02：6 m/s已进入5m门槛，但停止速度未收敛；独立恢复Land成功。sitl-03：浮点末点量化重复导致拒绝；修复未放宽执行校验。sitl-04：2 m/s配共享横向增益仍振荡，随后采用Demo局部参数。
- sitl-05/06：暴露重复cancel和Land内部disarm竞态，保留任务与恢复记录；修复为幂等停止、先等待真实解除武装反馈。
- sitl-08/10/13：输入等待/动态TF可用性失败，均停止并Land/disarmed；最终按源码保证使用严格同stamp权威Pose旋转body速度，有界等待且不换状态源。
- sitl-14：YOPO目标前原生Action未知结果及交替时钟，原始证据保留，独立恢复确认disarmed。相同PID只读探针得到不同DDS GID，未证明PID/GUID冲突；根因未确定。最终新轮次使用未复用DDS域并要求唯一发布者、单调时钟地面门控；三轮通过不代表所有DDS拓扑可靠。
- sitl-22的暂停注入因host/容器证据目录归属错误提前恢复，未达故障预期；sitl-23仅端口占用预检失败、未运动。修复为host预创建新case目录、禁止覆盖证据并等待清理完成，后由sitl-25/26完成故障验收。

AS2 checkout为`f387e0d733a5102808508bcbec1b9eca2c29d1f0`，仓库gitlink仍为`19c3973…`，均保留未切换。部署使用稳定镜像本身库；FollowPath实际安装头文件SHA256 `280f50f003d48b4e48882aca5db11109565d22a7ab74d66ba8de78ebadcbe2d4`与当前checkout该文件一致，不据此宣称整树一致。没有升级/替换共享镜像，没有创建commit/push。

## 最终GPU模型服务链分项计时

工作站容器 → onboard ROS Adapter → 独立GPU worker，320×185合成视轴深度，2 m/s部署配置，10/10通过且均低于100 ms；RTT中位 18.38 ms、最大 43.48 ms。证据`log/yopo-single/model-chain/1619540/`，包含两侧镜像ID、overlay、域197、逐请求计时与模型版本。

| 分项 | 中位/ms | 最大/ms |
| --- | --- | --- |
| conversion_ms | 2.351 | 4.975 |
| http_ms | 13.728 | 30.718 |
| adapter_ms | 16.229 | 36.266 |
| worker_preprocess_ms | 1.006 | 1.412 |
| worker_forward_and_transfer_ms | 1.401 | 10.912 |
| worker_decode_ms | 0.095 | 0.108 |

worker forward_and_transfer包含CPU→GPU、前向和GPU→CPU同步，不是纯GPU kernel计时；worker分项包含于HTTP时间，不能相加成RTT。RTT另含DDS和客户端开销。`inference/yopo/timing`以request_id关联这些诊断；计时不改变ROS推理核心请求/响应或规划语义。默认6 m/s的早期链路10/10、p50 17.76 ms/max50.95 ms另保留在`log/yopo-single/model-chain.json`，不覆盖为本轮2 m/s结果。

## 收尾与Skill回馈

本次GPU为NVIDIA GeForce RTX4090，驱动580.178.04，模型环境Python3.8.20/PyTorch2.4.1+cu118。GPU服务计时后已停止本任务worker；全部23个实际启动的独立仿真实例已按进程身份核验停止，测试容器清理完成，仅保留原有SUPER容器。证据`cleanup.json`、`remaining-containers.txt`。未改变用户已有子模块/工作区修改，没有commit或push。

已回顾过程与原始日志，完善五项正式Skill：model-migration-cuda（版本/配置对照）、uav-platform-sensor-adaptation（测量历史及同stamp权威姿态）、uav-ros-adaptation（双时间、纳秒量化与消息/TF顺序）、uav-reproducible-build（目标ROS构造与shell入口）、uav-sim-e2e-validation（停止/降落恢复、隔离与证据目录）。组件部署和能力卡片Skill的现有要求足够，本轮按其交付清单/卡片，回顾后无新增通用规则；未创建新Skill。

## 后续有屏长距离单次验证

用户现场观看下，沿用原森林起点和原长目标`[-37.57,231.76,6]`（水平234.79m），YOPO配置2m/s，导航等待300s，其他门槛不变。`live-forest-long-01/forest-long`接纳347个revision，导航122.66s，停止5.26s、低速保持1.020s，原生Land/disarmed确认；飞行阶段测量速度中位1.992m/s、峰值2.309m/s。结果为**单轮长程通过**，未做长程三次重复，不改变上文短程验收范围。

任务飞行/停止碰撞检查为false；降落后额外查询报告Landscape_0地形接触（原始has_collided=true、penetration_depth=0），数据单独保留，不宣称所有阶段原始碰撞标志均为false。详见`log/yopo-single/live-forest-long-01/forest-long/summary.json`及`collision-after-land.json`。用户要求现场观看，保留本次已解除武装的窗口和空闲部署；上文23轮清理结论仅对应原验收阶段。
