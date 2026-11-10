---
title: 快速开始
description: 了解首次运行所需环境，选择 Demo，并找到检查、运行与停止入口。
section: 开始使用
order: 10
status: 待完善
---

## 你将完成什么

首次使用推荐先完成环境准备和一个 Demo 的地面检查，再按该 Demo 的说明执行仿真任务。当前复现平台是 AirSim + PX4 SITL；环境包含外部场景、固定镜像和设备配置，完整教程尚在整理。

## 1. 准备运行环境

四个运行角色分别为 AirSim 仿真器、PX4 飞控、承载算法与 AS2 的算力板、调用能力的工作站。角色可以共置，但每个 Demo 支持的拓扑需单独确认。

按[仓库部署准备](https://github.com/Air-Suck/aerohub/blob/main/deployment/README.md#首次使用顺序)准备 SDK 与目标依赖、锁定源码、镜像、AirSim 场景及本机配置。RoboPi 路线还需要匹配的板端系统和连接环境。

不要覆盖已有 `.env` 或本机场景配置。共享镜像保持固定，源码迭代通过工作区补编译并加载 overlay。

## 2. 选择一个 Demo

| Demo | 用途 | 当前阅读重点 |
| --- | --- | --- |
| [SUPER forest8](https://github.com/Air-Suck/aerohub/blob/main/deployment/airsim/demos/super_forest/README.md#four-part) | 单机导航与四方运行 | 地面与短程验证通过；原 forest8 有更新被拒记录 |
| [CAT-ORA](https://github.com/Air-Suck/aerohub/blob/main/deployment/airsim/demos/catora_open_field/README.md) | 有限多机轨迹 | 按所选拓扑准备，核对取消与失流处理 |
| [Swarm-Formation](https://github.com/Air-Suck/aerohub/blob/main/deployment/airsim/demos/swarm_open_field/README.md) | 多机滚动轨迹 | 按现有三机组合阅读，不能外推到默认七机 |

建议从 SUPER 的四方地面检查与短程路径了解运行链。它仍需要上述环境准备，不能据此认为完整 forest8 已验收通过。

## 3. 检查、运行与停止

在所选 Demo 文档中依次执行：构建与部署、地面检查、短程或完整任务、结果核验、停止与清理。地面检查通过后再进入运行步骤；验收应同时核对任务结果、轨迹执行和错误记录。

当前各 Demo 的参数与停止命令不同，本页暂时链接到各自维护的准确入口。运行输出放入新的 `log/airsim/` 子目录，保留成功与失败记录；共享端口或板端资源的 Demo 串行运行。

## 接下来

阅读[运行 Demo](../docs/running-demos/)，或在 [Gallery](../gallery/) 中了解可用能力与接入范围。

待补：统一环境检查清单、可复制的首个 Demo 操作步骤、预期输出示例及常见失败恢复。
