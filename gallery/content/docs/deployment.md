---
title: 环境与部署
description: 理解四方运行职责、固定镜像与工作区补编译方式。
section: 用户指南
order: 30
status: 初版
---

## 四个运行角色

| 角色 | 职责 |
| --- | --- |
| 仿真器 | AirSim 提供环境、动力学与仿真真值 |
| 飞控板 | 物理 PX4 或 SITL PX4 承担飞控 |
| 算力板 | RoboPi 承载自主算法、Adapter、AS2 与执行链 |
| 工作站 | ROS 任务脚本调用已部署、已就绪的能力 |

角色不按容器名称判定；PC 仿真可共置多个角色。任务脚本在工作站，高频控制在算力板执行。

## 准备与部署入口

- [部署准备与首次使用顺序](https://github.com/Air-Suck/aerohub/blob/main/deployment/README.md)：镜像、场景、配置与运行组合。
- [RoboPi 部署](https://github.com/Air-Suck/aerohub/blob/main/deployment/robopi/README.md)：板端编译、部署与诊断。
- [PC 开发环境](https://github.com/Air-Suck/aerohub/blob/main/deployment/onboard_computer/README.md)：可选开发路径及当前限制。

## 源码迭代

共享 Docker 镜像作为稳定工具。日常修改在外部工作区补编译，运行时显式加载 install overlay。AS2 接口或 ABI 变化时，需要连同消费者重新编译。

完整职责与当前实现差异见[项目简介](https://github.com/Air-Suck/aerohub/blob/main/docs/项目简介.md#deployment-roles)。各组件是否完成迁移和验收仍由其独立文档说明。
