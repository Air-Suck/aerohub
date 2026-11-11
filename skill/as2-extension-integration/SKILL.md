---
name: as2-extension-integration
description: 按原生 Aerostack2 扩展约定实现 Planner、Behavior、State Estimator 和 Perception 接入，检查插件注册、宿主加载、执行与状态接口。
---

# AS2 原生扩展接入

## 输入与依据

读取项目 AGENTS.md、开发者贡献指导、能力映射及接口契约，确认 AS2 锁定提交。官方架构见 https://aerostack2.github.io/_01_aerostack2_concepts/architecture/index.html ，Behavior 教程见 https://aerostack2.github.io/_09_development/_tutorials/_tutorials/behavior.html 。文档用于理解范式，编译接口、注册方式和参数以目标源码为准。

## 四类实现规则

- Planner：当前通过扩展 FollowPath/FollowTimedPath 适配。算法保留在 `src/planner/<算法>/`，Adapter 生成执行 Goal 并处理 Feedback、Result、取消和机群协调；不把算法整个搬进执行 Behavior，也不直接绕过它发布运动参考。
- Behavior：先检查现有能力。不能满足功能需求时，按原生 `as2_behavior::BehaviorServer<ActionT>` 等约定创建，放在 `src/behavior/<行为>/`。既有 FollowPath 扩展保留原入口和时间化入口的互斥；不能通过取终点或忽略时间来伪造完整轨迹兼容。
- State Estimator：按原生插件机制复用或新增。读取实际基类、原生示例、loader、plugins.xml、CMake 导出与 launch。外部算法可独立运行，由插件消费其结果并提供 pose/twist/TF；不要求重复 EKF。代码放在 `src/state_estimator/<算法>/`。
- Perception：优先复用原生能力，不满足时扩展原生功能，仍不适合再独立实现。检查 `as2_map_server` 和 `as2_behaviors_object_perception` 实际能力；允许不同环境表示，不强制经过二维地图服务器。适配与接口归入 `src/perception/<能力>/`。

## 加载与运行边界

1. 优先独立 ROS 包，确认插件导出、包发现、实际类名与参数规则。插件不是仅写一个 XML 就能加载；独立节点也不等于 pluginlib 插件。
2. 在真实宿主中验证加载和接口，检查时间、坐标、TF 发布权和输出新鲜度。基类头文件是否安装与 include 路径是否导出是两个问题，避免硬编码个人路径来修复。
3. 明确外部算法进程由谁启动、健康检查及退出如何收口。State Estimator 插件不自动具有 Behavior 生命周期，也不自动将结果送入 PX4 EKF。
4. 参数默认值不得伪装成实机标定。保留原有部署可用性；添加显式选择入口，不擅自切换飞行中的权威状态源。

外部 SLAM 的 Adapter 应先形成完整、可验证的 `nav_msgs/Odometry` 契约，插件再负责 AS2 坐标、杆臂、陈旧值和发布权。若上游没有速度，先用两个有效且单调的采样位姿计算 twist，首帧保持未就绪；不要把有限差分、GPU 算法和 AS2 宿主职责揉进同一个插件。跟踪丢失时停止提交新权威状态，恢复或重启产生新地图 epoch 时禁止在运动中热切换。带 PGO 的全局位姿可能跳变，未另行提供平滑局部 chart 时必须向消费者声明。

定位精度通过不代表输出速率适合直接承担控制状态。低频 SLAM 与高频平台里程计并存时，优先让平台里程计提供连续的 `odom -> base_link` 和 body twist，让 SLAM 按时间配对后提供 `map -> odom` 全局校正。动态全局校正应以零阶保持方式随每个新的本地采样时间提交，使完整 TF 链满足控制 freshness；这不同于把陈旧 SLAM pose 重发成新的 `odom -> base_link`。插件必须分别验证两路 frame、时间、频率、失效语义和唯一 canonical 发布权。

同仓库存在不同 AS2 构建版本时，每个目标都需实际编译并在对应宿主中加载插件。FAST-LIVO2 在原基类中使用 `on_setup` 与插件发布接口，而 AirSim 构建中的新基类使用 `onSetup`、`getTransformationTypesAvailable` 和宿主拥有的 `StateEstimatorInterface`；仅保持类名和 plugin XML 不够。兼容选择依据实际头文件接口，不能复用另一版本的插件二进制。新宿主在 setup 之后才注册变换权限，主变换应在注册完成后的有效输入回调中提交；检查宿主定时发布是否违背陈旧状态静默要求。这些差异必须通过非恒等变换、时间戳、拒绝、静默与恢复测试核对，不能只修到编译成功。

## 交付

交付包、注册/加载配置、构建入口、最小接口测试、运行说明和限制。说明复用了什么、扩展了什么、是否改动上游。上游必须修改时使用锁定版本可重放补丁，保留版权。记录位于 `skill/temp/<任务>/`；实际验收证据位于 `log/`。插件加载成功只是接口阶段，仍需独立端到端验收。

## 外部低阶控制的模式扩展

角速度与姿态都是执行参考时，优先复用已有执行宿主的会话、时效、撤销和模式反馈逻辑。显式声明模式并只校验激活字段，未使用字段不参与有效性判断；未知模式拒绝，模式切换要求重新接管。区分固定世界系姿态四元数、机体角速度与推力的单位和frame。扩消息保持源码默认值不等于DDS二进制兼容，全部消费者须重生成接口及补编译。

姿态/推力以独立话题经过AS2时，在控制器和最终平台两级检验共同源stamp、坐标、新鲜度及四元数归一化，保留源时间；模式协商通过不能替代实际PX4姿态环启用的证据。姿态转换至本地odom后再进行平台ENU/FLU→NED/FRD转换，避免只改frame名称。现有FollowReference姿态入口的验证方法见`deployment/airsim/demos/attitude_control/`，不将通用控制源测试当作DiffPhys策略闭环。
