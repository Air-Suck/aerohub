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

## 交付

交付包、注册/加载配置、构建入口、最小接口测试、运行说明和限制。说明复用了什么、扩展了什么、是否改动上游。上游必须修改时使用锁定版本可重放补丁，保留版权。记录位于 `skill/temp/<任务>/`；实际验收证据位于 `log/`。插件加载成功只是接口阶段，仍需独立端到端验收。
