# 项目架构与开发约定

适用范围：本目录及子目录。以下架构决策由用户于 2026-11-03 确认，后续设计、实现和文档维护应遵循。

## 主工作目录（2026-11-05）

- 用户指定远端 `RKOH_Remote:/home/noah/code/UAV/aerostack2-integration` 为主工作目录；后续源码编辑、构建和测试以该目录为准。
- 本地目录保留为迁移时的副本，不再默认作为最新代码来源；不得未经差异检查将本地副本覆盖远端新修改。
- 临时测试目录不作为源码的长期保存位置。远端现有上游依赖、独有文件及用户修改须保留。

## 目标架构

### 2026-11-07 目标架构补充（优先于下方早期分层描述）

- 目标运行链为 Application → Mission（人类目标、Online Agent）→ Tool → 基础 Behavior → 基础能力与平台。现有 Planner 和 adapter 是 Navigation Tool 的实现基础；本次不实现 Mission 或 Tool。
- Tool 可直接读取传感器、状态与地图。基础能力层的 State Estimator 聚焦自身状态；新增 `src/perception/` 对应并列的环境感知、建图与环境表示，不将地图塞入自定位接口。
- 高层复合 Behavior 不加载、不暴露，保留上游代码；基础执行按需兼容扩展。
- Tool 必须声明功能卡片；资源互斥、取消交接和应急优先级由运行时软件强制执行，不仅依赖 Agent 自律。
- Offline Agent 负责适配和部署，Online Agent 负责任务编排。详细接口、目录和验证约束见 `docs/开发者贡献指导.md`。

### 早期 Planner 集成分层与当前实现约束


```text
Mission / Agent（后续阶段，当前不实现）
        ↓
Planner 层
    CAT-ORA / Swarm-Formation 算法
        ↓
    Adapter：统一算法输出，并转换为 Behavior 执行请求
        ↓
Behavior 执行层
    优先复用原生 Aerostack2 Behavior
    必要时新增符合 AS2 接口的执行 Behavior
        ↓
Motion Reference → Motion Controller → Aerial Platform
        ↓
PX4 / 仿真后端 / 飞行器
```

- Planner 是 Mission 与 Behavior 之间的独立层。当前阶段只考虑两个多机 Planner 的规划与执行接入，不提前实现 Mission 或 Agent。
- Planner 算法负责生成规划结果；adapter 负责数据适配，以及 Behavior 的 Goal 调用、Feedback/Result 处理、更新与取消等接口映射。
- “统一算法输出”指统一为 **Behavior 执行请求**，不是把轨迹数据本身变成 Behavior。
- Behavior 层负责执行计划及其执行生命周期；Motion 层负责运动参考与控制。规划、执行、控制的职责应清晰分离。

## Behavior 复用与扩展原则

- 2026-11-04 用户明确要求：以原生起飞、降落、到点、路径/轨迹跟随等作为执行层；导航、编队等复合功能作为 Mission 测试任务。优先扩展现有 FollowPath，兼容原生功能，并承接外部 Planner 的时间化轨迹，不能以仅新增一个无关 Behavior 代替该目标。

1. 优先复用原生运动 Behavior。不能仅凭 GoTo、FollowPath、FollowReference 等名称判断适用性，应核对目标版本的 Action、更新机制、插件实现和执行语义。
2. 复用必须保留算法所需语义，尤其是车辆与目标对应关系、路径约束、时间化轨迹、多机共同执行基时间、滚动更新连续性，以及暂停、取消和失败处理。
3. 不得以丢弃轨迹中间点、时间信息或机间协调约束来换取接口兼容。例如把完整多机轨迹仅取终点调用 GoTo，不能视为等价执行；逐机调用 FollowPath 也不自动保证同步。
4. 如果原生 Behavior 无法完整表达必要能力，应在 AS2 Behavior 扩展体系下新增执行 Behavior，例如时间化轨迹执行 Behavior。优先在本项目独立 ROS 2 包中使用 `as2_behavior::BehaviorServer<ActionT>` 等扩展接口，减少修改上游核心代码和与其内部实现的耦合。
5. 新增 Behavior 封装的是 **轨迹/计划执行能力**，算法仍留在 Planner 层；不要默认将整个 Planner framework 与算法封装成一个包办规划和执行的 Planner Behavior。
6. 执行 Behavior 内部可以采样轨迹并发布 Motion Reference；目标架构中 Planner 层不再通过直接发布 Motion Reference 绕过 Behavior 执行层。
7. 单个 Behavior 的生命周期不自动解决多机同步、跨 Behavior 冲突或机群失败收口；这些能力必须明确归属到调用协调逻辑或执行 Behavior，并验证其行为。

## 当前实现与验证边界

- 2026-11-04 默认链路已迁移为：算法 → ROS 2 adapter → `TimedTrajectorySegment` → 校验与机群调用协调 → 扩展 FollowPath 的 `FollowTimedPath` Action → Motion Reference → AS2 控制器/平台 → PX4。动力学验收进度与限制以主文档为准。
- 扩展包继承锁定版本的原生 `FollowPathBehavior`，保留原 Action 与插件；时间化入口与原生入口互斥。旧 setpoint 链路仅通过显式 `execution_backend:=legacy_setpoint` 启用。
- 起飞流程另行调用原生 `TakeoffBehavior`；两个 Planner 的轨迹通过扩展 FollowPath 执行。2026-11-04 已分别通过 CAT-ORA 四机、Swarm 七机指定无障碍 PX4/Gazebo 场景；这不代表未扩展的原生 FollowPath 自带时间化接口，也不代表真机验收。
- `/planner/control` 是项目自建生命周期服务，不是原生 AS2 Behavior 接口。
- 现有 Registry、配置校验、算法代理、统一轨迹格式和校验逻辑可以复用。迁移时应重新划分执行职责；Gateway 的采样、轨迹执行管理等可成为执行 Behavior 的内部实现。
- 本文件记录架构决策和验证边界，不授权仅因读取本文件就自动实施其他重构。

## 后续核查与验证

- 先审查锁定版本中原生 FollowPath、FollowReference 及轨迹相关组件，确认时间、同步和滚动更新能力，再决定复用或新增执行 Behavior。
- 针对 CAT-ORA 的有限多机轨迹与 Swarm-Formation 的滚动轨迹分别验证，不能把单机到点成功当作完整支持多机规划结果的证据。
- 实现变更应验证调用接口、轨迹语义、生命周期、异常路径和真实 PX4/Gazebo 动力学闭环；文档中区分设计目标、已实现功能与实际验收结果。

## 文档归属

- `docs/local/UAV调研与参考资料.md` 仅保留在远程磁盘，加入 `.gitignore`，不随 Git 仓库分发。

- 插件扩展和 Model Hub 开发者贡献指导独立维护在 `docs/开发者贡献指导.md`，作为独立的开发者贡献指导主文档；相关开发应遵循其中约定。

- 原生 Aerostack2 架构介绍维护在 `docs/as2简介.md`，以原生框架的具体执行实例解释，不混入本项目 Planner 的实现细节。
- 统一 Planner 的设计、模块接口数据形式、使用和验收集中维护在 `docs/项目简介.md`。
- 本文件用于保存开发约束和架构决策；详细接口及迁移设计写入上述 Planner 主文档，并明确与当前实现的差异。

## 目录约定（2026-11-06）

自建代码位于 `src/`，先按 `planner/`、`behavior/`、`state_estimator/`、`perception/` 分类，再按对象分子目录。两个 Planner 都使用 `upstream/`、`ros2_adapter/`、`ros1_proxy/`。上游算法分别在 `src/planner/cat_ora/upstream/` 和 `src/planner/swarm_formation/upstream/`。同类对象的目录规则见 `docs/开发者贡献指导.md`。AS2 依赖在 `as2/`，PX4 在 `px4/`，官方 Gazebo 工程在 `simulator/gazebo/`，均以 `.gitmodules` 和 gitlink 锁定。运行产物与测试日志统一进入被忽略的 `log/`。部署入口仍为 `deployment/`，不得重新依赖项目外部的旧源码路径。
