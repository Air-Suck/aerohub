# Aerostack2 算力板在环方案：Grill / Discovery Notes
Date: 2026-10-09 · Goal: 确定同一套统一 Planner 框架如何按任务选择在 PC 或真实算力板运行算法，并形成可实施、可验收的算力板在环方案。

## Summary / key decisions

- 已有 PC 基线：CAT-ORA 和 Swarm-Formation 通过项目自建 Planner framework 接入 Aerostack2、PX4 SITL 和 Gazebo，两个 Planner 均有成功冷启动证据。
- 新目标：保持相同任务配置、Planner 契约和 Aerostack2 执行链，由 Model Hub/Behavior 层选择 Planner 运行位置为 PC 或真实算力板。
- 初始建议：PC 保留 Gazebo、PX4 SITL、Aerostack2 和统一 Planner framework；算力板运行被选中的 Planner adapter、ROS1 proxy 和原始算法。跨机只传 AS2 位姿、统一轨迹、失败状态和生命周期控制，不把 ROS1 内部话题暴露到网络。
- 第一版不实现完整远程 Model Hub，也不做运行中热迁移。先在任务启动前通过 `execution.target: pc|board` 选择部署位置。
- 用户要求先修复现有交付缺口：必须在全新隔离环境按文档从零复现 AS2，失败则同步修正文档和代码；更新 CAT-ORA 顶层 README 及其他过时内容；补充一键启动脚本。
- 用户倾向把 Planner 生命周期做成 Behavior 必须实现的接口，再分别适配两个 Planner 自身生命周期。需要结合当前锁定的 Aerostack2 Behavior API 核实原生 Action/状态机契约和实际改造量。

## Q&A log

### Q1 — 先处理现有基线缺口
- Asked: 算力板应只承载 Planner 运行单元，还是连统一 `planner-runtime` 一起迁移？
- Captured: 用户暂未选择部署边界，要求先完成四项基线工作：在全新隔离环境从零复现 AS2；如失败则修正文档和代码；厘清 AS2 Behavior 生命周期及两个 Planner 的实现方式；更新所有过时文档；增加一键启动脚本。
- Flags: 算力板部署边界问题保留，待基线修复后继续访谈。

## Open flags (pending input)

- 目标算力板型号、CPU 架构、国产 OS 版本、容器能力和 ROS 2/DDS 可用性待确认 -> 用户/目标硬件环境
- 单块算力板承载整个机群 Planner，还是每架无人机各有一块板待确认 -> 用户
- PC 与算力板之间的物理网络和故障策略待确认 -> 用户/后续实测
- Planner 是否升级为 Aerostack2 原生 Behavior Action Server，以及保留 `/planner/control` 的兼容方式待源码核查后讨论 -> Codex/用户
