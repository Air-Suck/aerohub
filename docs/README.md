# 文档导航

本目录包含 Aerostack2 统一 Planner 工程文档，以及从 UAV 工作区迁入的公共资料。整理日期：2026-11-03。

## 项目必读

| 文档 | 用途 |
| --- | --- |
| [项目入口](../README.md) | 工程结构、构建和启动 |
| [项目目录说明](项目目录说明.md) | 全部顶层目录、主要子目录及按任务查找入口 |
| [Aerostack2 架构介绍](Aerostack2架构介绍.md) | 用起飞、向东飞 5 米、降落的完整实例解释原生执行架构 |
| [统一 Planner 框架主文档](Aerostack2统一Planner第一阶段接口与配置设计.md) | 设计逻辑、模块接口、配置、使用、验收与实现边界 |
| [使用指南入口](Aerostack2统一Planner使用指南.md) | 已并入主文档的构建、启动、任务控制与排障章节 |
| [接口与配置设计入口](Aerostack2统一Planner第一阶段接口与配置设计.md) | 主文档：模块职责、轨迹契约、坐标、时间及验收边界 |
| [第一阶段验收入口](Aerostack2统一Planner集成与PX4-Gazebo多机验收.md) | 已并入主文档的验收与后续范围章节 |
| [最终验证证据](../log/resolved/FINAL_VERIFICATION.md) | 2026-10-05 测试、冷启动任务及镜像差异说明 |

初次接触建议按“项目入口 → Aerostack2 架构介绍 → 统一 Planner 主文档”的顺序阅读；三个旧入口会跳转到主文档，合并前快照位于 `history/planner-before-consolidation/`。

## 讨论与历史

- [方案讨论](discussions/README.md)：集成可行性、算力板在环方案。
- [工程交接](history/README.md)：基线审计、构建和验收的历史待办。
- [公共工作区资料](workspace/README.md)：跨项目调研、复现、运维与参考论文。
- [本次文档迁移表](文档迁移记录.md)：29 个文件的旧路径与新位置。
- [本次整理记录](history/HANDOFF_20261103-docs-organization.md)。

## 阅读边界与维护约定

- 第一阶段验收覆盖 CAT-ORA 四机、Swarm-Formation 七机的统一 Planner 仿真闭环。独立复现中的 16 机演示和障碍物场景不能直接当作统一框架的验收结果。
- 当前生命周期是自建服务。原生 AS2 Behavior、Model Hub、Mission Agent 和算力板方案中的计划，不等于已实现能力。
- 历史记录中的容器状态、路径和待办仅反映记录当时。本次没有重新运行构建、仿真或远端健康检查。
- 核心使用与接口文档放顶层，讨论放 discussions，工程交接放 history，公共资料按 workspace 下的主题归档。
- 移动文档时更新 Markdown 链接和迁移表。历史命令及远端绝对路径保留原文，实际执行前核对环境。
