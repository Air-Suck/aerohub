---
title: 接口与配置参考
description: 查找轨迹接口、生命周期和部署配置的现有说明。
section: 深入了解
order: 50
status: 待完善
---

## 轨迹与生命周期

当前主接口包括 TimedTrajectorySegment 和扩展 FollowTimedPath Action。`/planner/control` 是项目自建生命周期服务。具体字段、调用与约束见[项目简介](https://github.com/Air-Suck/aerohub/blob/main/docs/项目简介.md)。

## 配置入口

- [AS2 配置](https://github.com/Air-Suck/aerohub/blob/main/deployment/airsim/config/as2/README.md)
- [AirSim 场景配置](https://github.com/Air-Suck/aerohub/blob/main/simulator/airsim/config/README.md)
- [部署配置说明](https://github.com/Air-Suck/aerohub/blob/main/deployment/README.md#配置与源码)

待补：按接口分类的消息与 Action 字段表、参数默认值、最小配置示例，以及兼容性说明。
