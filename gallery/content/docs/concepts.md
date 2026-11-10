---
title: 架构与核心概念
description: 了解 Mission、Tool、Behavior 以及自身状态与环境感知的职责。
section: 深入了解
order: 40
status: 初版
---

## 目标运行链

Application → Mission / Online Agent → Tool → 基础 Behavior → 基础能力与平台。

Mission 负责目标与任务编排；Tool 组织能力；Behavior 承担基础执行。Tool 可以直接读取传感器、状态和地图。通用 Mission、Tool 与资源仲裁仍属于目标架构，不能将现有 Demo 任务脚本视为已经实现这些层。

## 当前轨迹执行链

算法 → ROS 2 Adapter → TimedTrajectorySegment → 校验与机群调用协调 → 扩展 FollowPath 的 FollowTimedPath Action → Motion Reference → AS2 控制器与平台 → PX4。

起飞另行调用原生 TakeoffBehavior。原生 FollowPath 入口保留，时间化入口是本项目扩展；两者互斥。

## 状态与环境

State Estimator 提供自身位姿和速度。Perception 负责环境感知、建图与环境表示。地图不归入自身状态估计接口。

更多接口和实现边界见[项目简介](https://github.com/Air-Suck/aerohub/blob/main/docs/项目简介.md)；原生框架介绍见[AS2 简介](https://github.com/Air-Suck/aerohub/blob/main/docs/as2简介.md)。
