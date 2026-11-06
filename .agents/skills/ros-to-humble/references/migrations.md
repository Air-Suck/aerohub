# 迁移证据

## CAT-ORA：首轮，轨迹生成服务

- 上游 `ctu-mrs/catora`，提交 `4bc41f593eebde65dd74108a97e80430a145dfa0`，源码来自本地 `uav/catora:noetic-upstream` 镜像。
- 迁移源码：`libraries/cat_ora/humble`；ROS 1 对照归档：`thirdparty/cat_ora_ros1`，含原代理和旧部署。
- 保留 `SrvGetReshapingTrajectories` 字段、服务路径、输入顺序与原生目标分配；JSON 兼容层改调 ROS 2 服务。
- 核心无需 MRS 可视化依赖。nodelet 演示、离线文件入口、其他服务不属于这次服务迁移；请求可视化时明确拒绝。
- 真实对照覆盖单机、四机、重分配、静止，坐标容差 `1e-9`；速度/加速度取不同值时暴露并修正上游 callback 的传参颠倒，另用交换 ROS 1 参数的结果验证。
- 七种无效请求与恢复通过；JSON 入口与原生服务数值一致。
- 四机 Planner 接线验证包含真实算法、算法 adapter、公共 Planner 和 AS2 输出 adapter，检查轨迹与 setpoint；需已有 `onboard_computer:airsim-px4` 镜像，命令为 `bash libraries/cat_ora/humble/tests/verify_adapter.sh`。
- 原 smoke 脚本曾重复启动框架已管理的 adapter，已移除；测试就绪条件同时等待 ROS 服务和独立 TCP 代理。此为测试接线经验，不推定其他项目都有相同问题。
- 复现：`bash libraries/cat_ora/humble/tests/verify.sh`，需要本地 ROS 1 基线镜像与 Humble 镜像。完整边界和当前集成状态见 `libraries/cat_ora/humble/README.md`。

## Swarm-Formation：第二轮

- 上游 `ZJU-FAST-Lab/Swarm-Formation`，提交 `967a4bdfae949e994691f8ffc87dbb0147cbebb7`，与注册表一致；六包快照和旧代理/部署归档在 `thirdparty/swarm_formation_ros1`，原检出无本地修改。
- 本地无历史 `uav/swarm-formation:noetic-px4` 镜像，使用 Noetic 依赖镜像和固定源码重建基线；不能推定覆盖历史镜像可能存在的补丁。
- 原生 Humble 六包位于 `libraries/swarm_formation/humble`，交付镜像 `uav/aerostack2-swarm-formation:humble` 已构建，运行路径不使用 ROS 1。
- 同一份 C++ 程序链接两版真实优化器，固定算法 ROS 时间和七机参考轨迹，覆盖四场景、七车辆、编队优化开/关，共 56 个组合，每组合预热 3 次并测 15 次。840 次/版本均成功；51 点/轨迹逐点对照，最大位置差 `4.679728452803087e-8 m`，最大时长差 `3.239904877716526e-8 s`。
- 最终干净复现：编队优化开启时，中位数 ROS 1 / ROS 2 为 `0.583 / 0.511 ms`，P95 为 `0.776 / 0.701 ms`；关闭时中位数 `0.110 / 0.061 ms`，P95 `0.157 / 0.092 ms`。这是固定无障碍样例的观测，不是通用性能保证。
- 耗时用独立 steady clock 记录，构建后顺序运行。环境 GCC 9.4/Eigen 3.3.7 与 GCC 11.4/Eigen 3.4，Release flags 也有差异，不能把速度差异全归因于 ROS。
- 七个真实规划节点通过里程计/地图输入、广播、多次 PolyTraj 输出、JSON 采样、全部完成回调，以及三种无效请求拒绝和恢复。里程计为轨迹采样回放，不是动力学仿真。
- 首次输出后继续跑完成分支，暴露默认 system clock 零值与节点 ROS clock 不能相减的问题。用显式 ROS clock 零值保留 ROS 1 语义修正，后续全部完成回调通过。
- 消息生成与同名库在 rosidl 下发生 target 冲突，库 target 改为 `traj_visualization`；包名和消息契约保留。
- 参数路径统一 `/` 到 `.`，私有 topic 统一 `~/...`；目标 latch 对应 transient-local 发布和订阅，并等待发现。原代理触发路径与编队手动模式订阅不一致，launch 显式映射，单独记录为接线变更。
- 上游编队目标高度写死 0.5 m、编队 scale 默认 2，而当前 adapter 的目标形状校验按单位 scale。保留算法语义，完整任务目标一致性属于后续接入，binding 仍为 `not_integrated`。
- 完整复现：`bash libraries/swarm_formation/tests/verify.sh`；原始证据默认 `/tmp/aerohub-swarm-validation`。边界、性能解释和部署方式见 `libraries/swarm_formation/README.md`。

### 后续真实输入与应用验证

- CAT-ORA 已在独立 PC AirSimNH 空旷路口完成四机起飞、规划、到点保持、降落解锁。应用在地面校准固定局部原点，再由 AirSim 实际位置独立验收；最大实际终点误差约 0.117 m。代码与版本化摘要在 `demos/catora_open_field/`。
- Swarm 真实点云路径为 LiDAR → `as2_navigation_inputs` → 固定共享 ENU 输入桥 → 原生 GridMap/ESDF。必须匹配传感器 best-effort QoS，保留采样时间；body twist 保留其语义，由规划入口显式转到共享系，真实代理不再发布零速度里程计。
- 独立静态 AirSim 实例实测 841 个有效点全部匹配占据图和 ESDF 表面，断流健康状态与恢复通过；代理对七机健康 heartbeat 失效时拒绝缓存轨迹。此结论不覆盖七机运动避障飞行。
- 检查真实 ESDF 时发现上游临时 vector 在 `clear()` 后继续按下标访问的内存错误，保持尺寸后按原公式计算；840 次固定输入回归数值不变。将此修正与 ROS 接口变化分别记录。
- 真实传感器验收与接口模拟测试分别报告；版本化结果摘要在 `libraries/swarm_formation/validation/`，完整本机日志仍在 `artifacts/`。


## CAT-ORA：板端执行适配与故障回归（2026-11-06）

- 算法输入、输出差异放在算法自身 adapter 中；CAT-ORA 的局部定位中继从应用移入 adapter，不引入公共导航层，也不为算法修改 AS2 执行模式。PVA 转换复用现有实现。
- Planner CANCEL 只代表取消规划获准，不能当作飞机停止。停止意图转换为真实 AS2 HOVER 请求，核对响应与 HOVER/POSITION 反馈；完成后保留确认，后续降落不撤销停止结果。
- 取消／失败通知先于等待规划子进程退出；整个独立进程组都要回收，不能仅等 ros2 run wrapper。诊断 CLI 还可能留下继承 overlay 环境的 ROS2 daemon，应按本次运行标记复查。
- 输入检查 frame、源时间、有限数值和四元数；RPC 返回后也检查起点快照新鲜度。1.3 秒迟到响应测试确认旧快照不发布，后续新鲜输入可以恢复生成。
- 一次性 trajectory 发布使用 transient-local，观察者也匹配该 durability，避免发现竞态丢首条。状态流保留最新样本；不得给旧规划重新盖时间戳。
- 增加新 PX4 接口时必须核对实际消息版本：本次诊断发现板端 VehicleStatus v1 与 SITL v4 不匹配，未将其接入执行，也未通过改 topic 名掩盖 ABI 差异。解锁仍核对现有匹配的 ACK，拒绝即中止并安全收尾。
- 仿真故障必须与迁移分开：仅 PC 的四机最小循环复现 TCP 已连接但首心跳缺失，进而没有 HIL 传感器和定位。AirSim 源码先启动读取再注册回调；相同镜像只增加 100 ms 首包缓冲后三次启动通过。此为应用启动缓解，未修改 AirSim/PX4/AS2 二进制，也不代表长期可靠性验收。
- 当前板端原生服务和 JSON 接口五组轨迹仍满足既有基线 1e-9 m 容差（包含已记录的参数顺序修正），七类非法请求和恢复通过。输入／停止接口故障、数值回归、完整飞行的证据应分别报告。代码和复现入口见 `libraries/cat_ora/ros2_adapter/README.md` 与 `demos/catora_open_field/README.md`。

- 多机 TF 与转发时序的补充证据：四机共享 `/tf` 为 4 发布／20 订阅；逐机 remapping 后每机 1 发布／5 订阅，保持 frame_id 和变换不变。地面对照时任务侧轨迹约 18.7→30.6 Hz、最大时钟超前约 57→21 ms，但 CPU 仍近满载，不能声称有充分容量。应用侧 TCP 转发必须保留双向 TCP_NODELAY；未禁用 Nagle 曾引入约 41.9 ms 小包排队，修正后同条件约 3.3 ms，负向对照与回归测试均保留。
- 最终正常到点、主动取消、实际输出进程暂停三种四机 AirSim 路径均在 RoboPi 原生 CAT-ORA／Planner／AS2 上通过，安装清单一致且保留默认时间保护。索引 `demos/catora_open_field/stability-validation.json`；正常真值终点误差最大约 0.117 m，最小实际机间距约 5.87 m。仅固定场景验收，不推广为长期稳定性或扩容保证。
