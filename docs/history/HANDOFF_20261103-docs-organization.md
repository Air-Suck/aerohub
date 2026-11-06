# docs 文档整理

## 当前任务

整理 docs 后，新增 Aerostack2 架构介绍与统一 Planner 集成设计及使用文档。

## 已完成

- 按用户要求将原生 AS2 架构文档重写为具体任务的连续执行流程，围绕起飞 1.5 米、向东飞 5 米、降落解释各层职责。核对上游三个运动插件，明确位置 GoTo 不自动调用避障规划，Land 速度插件使用高度/速度/持续时间判据；数值示例不作为实飞验收。

- 按用户要求将统一 Planner 的设计逻辑、模块接口数据形式、配置、使用方式、验收结果和部署边界集中合并到 `Aerostack2统一Planner第一阶段接口与配置设计.md`。
- 原三个独立 Planner 文档改为主文档入口，合并前原文保存于 `history/planner-before-consolidation/`，便于追溯；原生 Aerostack2 架构文档不混入 Planner 内容。

- 按用户澄清重写 Aerostack2 架构介绍，聚焦原生框架；移除本项目特有的 Planner、Compose、机数和通信配置。核对固定提交的上游源码，补充原生行为树、Python API、规划/轨迹生成 Behavior、控制与估计插件及三类仿真路线。
- 原生 Behavior 的实际代码采用 Action 加附加控制服务；旧上游 README 的全 service 描述未作为实现依据。原生 BehaviorStatus 为 IDLE/RUNNING/PAUSED，不套用本项目状态枚举。

- 在统一 Planner 设计文档第 2 节补充模块间接口：配置文件、控制服务、位姿、两类 TCP/JSON 代理、ROS 1 原生接口、统一轨迹、FleetSetpoint、AS2 控制参考、失败与时钟格式；增加字段树及数据示例。
- 根据源码明确 Swarm 固定队形校验、同 task_id 不重置 proxy 缓存、代理响应不携带采样时间等实际边界。

- 新增项目目录说明，覆盖全部顶层目录、主要功能子目录和按任务查找入口；核对 vendor 构建依赖、上游源码准备路径和历史辅助目录用途，并更新总导航。
- 按用户要求将目录说明改为完整展开的目录树形式，各节点直接标注用途；尚未准备的上游目录单列说明。

- 新增两份说明文档并加入总导航；依据 Compose、启动脚本、Registry、framework 和 adapters 核对实现。
- 明确自建生命周期、TCP/JSON 算法代理、位置参考输出、逐机结果未填充及定位新鲜度检查边界。
- 使用章节说明当前编号目录的 PX4 挂载路径设置，以及 NO_CACHE 仅直接作用于平台构建的脚本行为。
- 新文档的 Bash 代码块通过语法检查，CAT-ORA YAML 示例与现有配置逐字一致；全 docs 的本地 Markdown 链接通过存在性检查。未验证实际命令执行或 Mermaid 渲染。

- 顶层保留使用指南、接口设计、第一阶段验收摘要。
- 方案讨论移至 discussions，集成交接移至 history。
- workspace 细分 research、reproductions、operations、references、archive；交接记录归入相应主题。
- 原工作区 AGENTS.md 按已有“已存档”说明改名为普通历史文档，保留原文。
- 移动 29 个文件，移动后逐文件核对内容一致；更新受影响的 Markdown 相对链接。
- 新增总导航、分类索引及完整迁移表；没有删除原始资料。
- 校验 170 条本地 Markdown 链接；修正 Survey 草稿中此前失效的 SkyDreamer 来源链接，全部目标存在。

## 有效结论

- 历史验收不等于当前运行状态，独立复现的规模和障碍物场景不等于统一 Planner 的验收范围。
- 文档原有 Windows、远端绝对路径及历史命令保留；运行前需核对环境。

## 下一步

后续从 docs/README.md 查阅；本次为文档整理与编写，未运行构建或仿真。

## 相关链接

- [文档导航](../README.md)
- [迁移表](../文档迁移记录.md)
