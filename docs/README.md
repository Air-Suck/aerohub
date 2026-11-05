# 文档导航

## 当前运行与交付

| 文档 | 用途 |
| --- | --- |
| [项目入口](../README.md) | 当前分层、目录和单机运行流程 |
| [容器配方与交付](../deployment/README.md) | flight_board tar、官方 OHOS 环境前提与验证范围 |
| [RoboPi 运行](../deployment/robopi/README.md) | PC 仿真 + 板端 AS2 的构建、部署、启动和验证 |
| [场景与 settings](../config/airsim/README.md) | AirSim binary、profile 与传感器配置 |
| [配置组织](../config/README.md) | Library 注册、AS2 和 DDS 配置 |
| [项目目录说明](项目目录说明.md) | 目录职责及按任务查找入口 |

初次使用按“项目入口 → 容器配方与交付 → RoboPi 运行”阅读。

## Planner 设计与历史参考

这些文档保留接口、设计决策和历史验收价值；旧 Gazebo 部署命令对应 main
的历史提交，不代表当前分支有同名入口，见[旧部署参考版本](history/README.md#旧部署参考版本)。

| 文档 | 用途 |
| --- | --- |
| [Aerostack2 架构介绍](Aerostack2架构介绍.md) | AS2 分层、控制与反馈闭环参考 |
| [统一 Planner 框架](统一Planner框架集成设计与使用.md) | 契约、生命周期和输出桥设计 |
| [历史使用指南](Aerostack2统一Planner使用指南.md) | main 的旧 Planner 部署与操作记录 |
| [接口与配置设计](Aerostack2统一Planner第一阶段接口与配置设计.md) | 消息、坐标、时间及验收边界 |
| [第一阶段验收](Aerostack2统一Planner集成与PX4-Gazebo多机验收.md) | 旧七机集成结果与限制 |
| [历史验证证据](../artifacts/resolved/FINAL_VERIFICATION.md) | 当时构建和冷启动证据 |

## 讨论与资料

- [方案讨论](discussions/README.md)：设计决策和算力板方案。
- [工程交接](history/README.md)：历史基线及旧部署参考提交。
- [公共工作区资料](workspace/README.md)：调研、复现与运维记录。
- [文档迁移表](文档迁移记录.md)：早期文档来源。

历史记录保留当时路径及验证范围。当前源码、配置、目标环境与验证结论分别
核对，不将过去的闭环结果直接推广到新的运行组合。
