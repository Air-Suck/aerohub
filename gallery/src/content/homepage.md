---
title: Aerohub
heroTitle: 连接算法与飞行能力。
eyebrow: 无人机算法接入、运行与能力展示
lead: 以 Aerostack2 为执行基础，连接多机规划、自身状态估计与环境感知，记录算法接入方式和验证边界。
features:
  - title: 从首次使用开始
    description: Quickstart 提供环境准备、Demo 选择和运行检查路线；User Guide 逐步补充配置、部署与使用教程。
  - title: 规划与执行分层
    description: Planner 生成计划，Adapter 保留车辆、坐标和时间语义，扩展 FollowPath 通过 FollowTimedPath 执行时间化轨迹。
  - title: 自身状态与环境感知
    description: State Estimator 提供自身位姿与速度，Perception 提供环境表示；FAST-LIVO2 的状态与配准点云分别接入两类接口。
  - title: 算法 Gallery
    description: 查看 CAT-ORA、Swarm-Formation、SUPER 与 FAST-LIVO2 的实现、运行绑定及已有验证范围。
  - title: 四方部署目标
    description: AirSim 提供仿真环境，PX4 承担飞控，RoboPi 承载自主算法与 AS2，工作站经 ROS 编排任务。当前历史 Demo 尚未全部按此边界拆分。
  - title: Mission 与 Tool 目标
    description: 目标链为 Application → Mission / Online Agent → Tool → 基础 Behavior；现有 Planner 是 Navigation Tool 的实现基础，通用 Mission、Tool 与资源仲裁尚未交付。
  - title: 可复现流程与证据
    description: Skill 覆盖架构、适配、构建、部署与验证。后续复现统一使用 AirSim + PX4 SITL，历史 Gazebo 结果按原场景保留，接口与合成测试单独说明。
galleryLinkLabel: 查看算法与接入记录 →
---
