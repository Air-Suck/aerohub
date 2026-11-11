---
title: 运行 Demo
description: 选择现有示例，理解运行前检查、任务执行和结果核验。
section: 用户指南
order: 20
status: 初版
---

## 选择运行入口

[Library Demo 索引](https://github.com/Air-Suck/aerohub/blob/main/deployment/airsim/demos/README.md)集中维护 CAT-ORA、Swarm-Formation 与 SUPER 的运行入口。先核对示例的设备、机数、传感器和运行拓扑，再按对应 README 操作。

## 推荐的操作顺序

1. 完成[快速开始](../../quickstart/)中的环境准备。
2. 阅读 Demo 的前置条件，准备指定场景、配置与组件。
3. 完成构建、部署和地面检查。
4. 按 Demo 的任务命令执行，观察状态与输出。
5. 对照通过条件核验，执行文档中的停止命令并保存日志。

## 怎样理解结果

构建成功只说明产物生成；接口检查只覆盖调用与数据流。仿真闭环和板端验收需要各自的运行证据。上游演示视频用于展示算法能力，不计入本仓库的验收。

SUPER 四方模式的地面与短程验证通过，原 forest8 记录了更新被拒；具体原因与范围见[专项 Validation](https://github.com/Air-Suck/aerohub/blob/main/deployment/airsim/demos/super_forest/VALIDATION-four-part-20261109.md)。

## 查看能力

前往 [Gallery](../../gallery/) 查看各组件的接入状态、依赖和展示资料。不同算法的历史验证平台与日期可能不同，使用时需核对具体证据。
