# Aerostack2 集成可行性：Grill / Discovery Notes
Date: 2026-09-30 · Goal: 明确 Aerostack2 是否适合作为 CAT-ORA 与 Swarm-Formation 的统一管理和运行框架，并确定可接受的集成边界。

## Summary / key decisions

- 引入 Aerostack2 的长期目标是为 UAV model hub 打基础。
- 当前阶段不把 CAT-ORA 和 Swarm-Formation分别固化为“队形重构 Behavior”和“持续规划模块”；二者统一视为同一类 `planner` 模块的两种可替换实现。
- 当前成功标准已确认：在同一 Aerostack2 系统中，CAT-ORA 和 Swarm-Formation 均注册为 `planner` 类型模块；系统能通过配置选择其中一个，使用统一生命周期启动，接收统一任务语义，并将规划结果交给同一套 Aerostack2/PX4 执行链路。
- 远期理想 Model Hub 包括在线热切换、自动选择最优 Planner、自动适配不同硬件、将算法核心逐步原生 ROS2 化，以及完整模型仓库服务；本阶段只记录，不展开设计。
- 远期 Mission 可演进为 Agent：把 Planner 等下层能力作为工具进行组合，完成复杂任务；本阶段不实施。
- Behavior 控制命令、生命周期状态、失败原因和无人机物理状态分层表达；统一命令与状态见后述公共状态机。
- 用户认可“两级 Planner 契约”和 Adapter/Output Gateway：不同 Planner 保留原生输出，由各自 adapter 归一化后接入同一 Aerostack2/PX4 执行链路。
- Adapter 分类不能只看消息格式，还需记录批处理/流式、单机/多机、输入依赖、坐标系、更新率、规划时域、动力学约束和算力需求。
- Aerostack2 当前 Path Planner 输出无逐点时间的几何路径，但这不妨碍一般多机独立飞行；它只是不直接保证多机轨迹的严格时间协同。ROS2 能传输多机速度/位置指令，但不会自动提供跨机时钟一致性、延迟补偿或同步执行保证。
- 第一阶段明确采用单活模式：一个任务同时只激活 CAT-ORA 或 Swarm-Formation 中的一个 Planner，通过配置选择，不做二者并行协作或在线热切换。
- 用户倾向为 Swarm-Formation 输出补充时间信息，从而让两个 Planner 经 adapter 后采用统一的下游数据格式。源码审计确认这条路线可行：Swarm 内部已按 `start_time + t_cur` 对连续轨迹采样，现有 `PositionCommand` 已含消息生成时刻、位置、速度、加速度、yaw/yaw_dot，但只发布当前采样点，没有未来各点的执行偏移。第一阶段可由 adapter 将其封装为短时 `TimedTrajectorySegment`，不改算法核心。
- Model Hub 定位为 Planner 的管理与适配系统，而不是 Planner 本身。目标架构分为控制面（注册、查询、版本、下载、依赖解析、硬件匹配、镜像选择、配置生成）和运行面（部署、启停、健康检查、Planner 选择、故障恢复、未来热切换），其下通过 Aerostack2 Planner Behavior 和 Planner Adapter 接入算法。
- 第一阶段只要求一个任务同时激活一个 Planner，由配置选择 CAT-ORA 或 Swarm-Formation；不实现两者并行协作、运行时热切换或自动选优。
- 第一阶段为每个 Planner 维护统一格式的 Planner 描述文件和可选择的运行配置；现阶段可以人工编写和验证，未来由 Model Hub/Agent 辅助生成、校验和部署。
- Swarm-Formation 同样需要多机协调，但属于基于带时间戳的新鲜状态进行持续闭环修正；CAT-ORA 的避碰保证更依赖各机按同一时间参数化轨迹执行。二者所需的时间语义强度不同。
- 用户正式确认 Model Hub 的目标流程：发现合适的 Planner/版本 → 检查硬件与软件兼容性 → 下载或构建镜像 → 生成 Planner 描述文件 → 生成面向当前机群的运行配置 → 部署并交给 Aerostack2 Planner Behavior 启动。
- 配置不应理解为整个系统永远只有一个文件：可保存多个 Planner 参数模板和任务配置；第一阶段只要求一个顶层活动配置在一次任务中选择一个 Planner，并引用该 Planner 的专属参数配置。
- 北向任务配置与南向规划结果分离。Mission/Planner Behavior 通过统一任务语义选择并启动 Planner；两个 adapter 的南向输出都归一化为 `TimedTrajectorySegment`，不再暴露两种输出 profile。
- 用户已正式接受统一南向数据契约为 `TimedTrajectorySegment`：CAT-ORA 与 Swarm-Formation 使用相同的数据结构，但允许不同的轨迹长度（horizon）和发布频率。CAT-ORA 可输出较长、多机联合的分段轨迹，Swarm-Formation 可高频输出每机极短轨迹段，并以新段替换旧段。AS2 原生 `TrajectorySetpoints` 没有逐点执行时间，因此该契约应是 Planner/Adapter 层的自定义接口，再由 Output Gateway 按当前时刻采样并送入 AS2 控制链。
- 两个 Planner 的时间表达也必须完全同构：统一为 `valid_from`（基时间）+ 每点 `time_from_start`（相对偏移）。Swarm-Formation 的基时间接近当前时刻且频繁更新，不代表另一种格式或语义，只是其滚动规划特性造成的数值和发布节奏差异。
- 轨迹失效保护已确认：Output Gateway 遇到过期轨迹，或超过规定时间仍未收到新的有效轨迹时，必须停止沿用旧轨迹并维持安全参考；Planner 状态为 `FAILED`，原因码为 `TRAJECTORY_TIMEOUT`。当前由 Gateway 实现并纳入仿真验收，未来可演进为独立 Safety Supervisor。
- 用户进一步明确首期应采用配置驱动：每个 Planner 有自己的描述/参数配置，每个具体任务另有任务配置；任务配置同时指定使用哪个 Planner、参与机群、目标与约束。Mission 读取任务配置，再调用对应 Planner Behavior。首期不额外设计一个对用户暴露的自定义 `PlanRequest` 消息；Mission → Behavior 的运行时调用优先复用 Aerostack2 现有 Action/生命周期接口，只传启动、取消、反馈关联所需的最小信息。
- Planner 参数文件定位为“已验证可运行的默认模板”，不是不可覆盖的唯一配置。任务配置允许提供 Planner 参数覆盖项；未提供的参数继承模板默认值。第一阶段只保留“模板 + 任务覆盖”两层，运行前生成并记录一份 resolved/effective config。
- 参数合并规则已确认：YAML 映射递归合并，标量直接覆盖，数组整体替换；出现模板/schema 中不存在的参数立即报错；每次启动保存最终 resolved config。
- Swarm-Formation 同样需要任务目标，但其原生目标语义不是任意逐机终点：`formationWaypointCallback` 接收编队中心目标，并按 `end_pt = swarm_central_pos + swarm_scale * relative_pos[drone_id]` 计算各机终点；相对队形和缩放参数来自 Planner 配置。持续 odom/点云/邻机轨迹用于滚动避障与协调，不能替代“编队最终飞到哪里”的目标。
- 用户决定第一阶段的公共任务配置直接使用逐机目标坐标。Swarm adapter 在启动前对这些坐标求目标中心，并计算 `relative_pos[i] = target[i] - center`（首期令 `swarm_scale=1`），转换为 Swarm 原生的中心目标与相对队形参数；CAT-ORA adapter 直接使用逐机目标。该选择让外部配置保持直观，隐藏算法内部目标表示差异。
- 首期逐机目标只包含 `x/y/z`，不包含目标 yaw，也不以 yaw 判断任务完成。轨迹点 yaw 是可选 Planner 输出；未声明有效时由 Gateway 映射为 PX4 `NaN`，不得自行补值。
- 统一任务成功判据已确认：所有参与无人机必须同时进入各自目标的三维位置容差，并让整个机群连续保持指定时间；任意一架离开容差，保持计时清零重来。判据由 Mission 根据 AS2/PX4 实际位置反馈判断。默认 `position_tolerance=0.5 m`、`hold_duration=1.0 s`、`task_timeout=120 s`，均允许任务配置覆盖。
- 任务总超时处理已确认：Mission 将任务标记为 `FAILED`，原因设为 `TASK_TIMEOUT`，取消当前 Planner Behavior，Output Gateway 停止接收后续轨迹，并让所有参与无人机定点悬停。`TASK_TIMEOUT` 表示整个任务耗时过长，与 Planner 未及时提供有效轨迹的 `TRAJECTORY_TIMEOUT` 分开记录。
- 主动取消语义已确认：任务终态为 `CANCELED` 而非 `FAILED`；取消 Planner Behavior，Output Gateway 丢弃该任务后续到达的轨迹，所有参与无人机定点悬停。取消后必须创建/启动新任务才能继续运动，旧任务延迟消息不得重新驱动车辆。
- 第一阶段必须实现 `PAUSED/RESUME`：暂停时保留任务与 Planner 配置，暂停新轨迹生成，丢弃暂停前尚未执行的轨迹并定点悬停；恢复时必须基于最新实际位置重新规划，禁止继续执行暂停前旧轨迹。
- 暂停期间停止累计 `task_timeout`，有效任务耗时按“墙钟运行时间减累计暂停时间”计算；同时停用轨迹新鲜度超时，避免悬停期间误报 `TRAJECTORY_TIMEOUT`。恢复后重新开始等待所选 Planner 基于最新状态输出的新轨迹。
- Planner 失败必须覆盖两条路径：Planner/adapter 明确返回无解时立即进入 `FAILED/NO_SOLUTION`；Planner 没有返回结果时由等待定时器在阈值后进入 `FAILED/TRAJECTORY_TIMEOUT`。两者都保持悬停，且第一阶段均为必做项。
- 轨迹等待不拆成 `initial_trajectory` 与 `trajectory_update` 两个公共参数，而统一为 `trajectory_timeout`。它只监控“Planner 按契约本应继续供给轨迹，但连续多久没有有效轨迹”的中断；CAT-ORA 与 Swarm-Formation 的 Planner 参数模板为同一变量提供不同默认值，任务配置可继续覆盖。任务是否及时到达目标只由 `task_timeout` 判断，不能再用 `trajectory_timeout` 限制终点收敛时间。
- `TimedTrajectorySegment` 增加统一布尔字段 `is_final`：它仅表示 Planner 声明该段是本任务最后一段规划输出，不表示无人机已经实际到达。`is_final=true` 的段结束后由 Gateway 保持终点，并由 Mission 在 `task_timeout` 内判断全机群到达与 1 秒稳定；`is_final=false` 的段耗尽且没有后继段时才启动 `trajectory_timeout`。
- `TimedTrajectorySegment` 的层级定位进一步明确：它是 Planner Behavior 所管理的 Planner/Adapter 向南交给 Output Gateway 的统一规划结果，不是 Aerostack2 所有 Behavior 通用的下行消息。一条消息对应一个任务机群，并承载该任务中全部参与无人机的轨迹；第一阶段不支持一条消息混装多个独立机群。
- 首期不设置 `segment_id`。同一任务内使用统一 Gazebo `/clock` 下的 `generated_at` 判断轨迹段版本：更晚者接受并替换，相同者视为重复并忽略，更早者视为乱序并拒绝；`task_id` 不匹配直接拒绝。`valid_from` 只表示执行基时间，不用于版本排序。
- 轨迹点时间合法性已确认：每个 `time_from_start >= 0`；同一车辆内严格递增；整个机群共享 `valid_from`；各机点数可不同，但最后一点的 `time_from_start` 必须一致。某架车辆原生轨迹提前结束时，由 adapter 补终点保持点对齐机群段结束时刻。
- `TimedTrajectoryPoint` 字段有效性已确认：`time_from_start` 与 position 必填；velocity、acceleration、yaw 可选，由每条 `VehicleTrajectory.valid_fields` 显式声明是否可信。adapter 不得仅为填满格式而伪造可选字段；Gateway 只消费被声明为有效的字段。
- `TimedTrajectorySegment` 采用机群级原子校验：若缺失参与无人机、vehicle id 重复、包含任务外无人机、任一车辆轨迹为空，或共享时间基准不一致，则 Output Gateway 整体拒绝该段，不允许部分车辆执行，并上报 `INVALID_TRAJECTORY_SET`。
- 统一坐标固定为任务级机群共享 `LOCAL_NED`。任务配置定义共享原点，轨迹段通过每次运行唯一的 `task_id` 继承，不携带 ROS 自由文本 `frame_id`。yaw 完全沿用 PX4 约定。
- Output Gateway 根据 Gazebo 出生位姿生成并保存逐机 `px4_origin_in_fleet_ned`，负责将共享坐标平移到各 PX4 本地 NED；核心契约不依赖 ROS TF。
- 任务以可复用 `task.name` 表示配置，以 `planner.id + 精确 version` 从本地 Registry 选择 Planner。Mission 每次启动生成唯一 `task_id`；暂停/恢复沿用，取消后重启更换。
- 用户任务文件只含 Planner 选择和覆盖、`vehicles[]`、逐机 xyz 目标及完成参数；默认值、坐标原点、镜像和 topic 等完整运行信息进入 resolved task config。启动前任何校验失败都使整任务 `REJECTED/INVALID_TASK`，不会发送飞行指令。
- Mission 编排同级的 Arm、Takeoff、Planner、Land、Disarm Behavior。单机是仅含一架无人机的机群特例；所有 Behavior 使用统一机群级结果和逐机诊断。
- 公共状态为 `REJECTED/RUNNING/PAUSED/SUCCEEDED/FAILED/CANCELED`，失败原因另设原因码。失败源先维持安全参考，Mission 启动独立 Hover Behavior 并以 `FAILED` 结束当前 Mission；首期不自动降落。
- CAT-ORA 与 Swarm-Formation 共用最小 manifest schema，包括身份版本、运行入口、参数 schema、能力、输入、统一输出版本、资源需求和支持平台。本地 Registry 只做按精确版本查找和校验。
- 第一阶段以同一 Aerostack2 工程内配置单选两个 Planner、PX4 SITL + Gazebo 多机真实执行、统一生命周期与异常路径测试、冷启动部署文档为验收范围。并行协作、热切换、自动择优、完整 Model Hub、真机/国产 OS 和 Mission Agent 均留待后续。

## Q&A log

### Q1 — 引入 Aerostack2 的首要目标
- Asked: 是否先统一任务启动、多机命名、状态监控、运动指令和 PX4 接口，而暂不要求重写算法核心？
- Captured: 用户澄清长期目标是为 model hub 打基础。现阶段 CAT-ORA 和 Swarm-Formation 都应被定义为 `planner` 模块，只是两个不同的 planner 实现；目标是在同一框架上运行这两个 planner，而不是把二者设计成职责不同的模块。
- Flags: 需要确定统一 planner 契约的抽象粒度，以及切换是构建时、启动时还是运行时完成。

### Q2 — Planner 层级、阶段验收与远期愿景
- Asked: 是否接受将统一 Planner 输出定义为“带时间信息的短时轨迹段”？
- Captured: 用户确认当前验收标准为：两个算法都注册成 `planner`，可通过配置选择，具有统一生命周期和任务语义，并复用同一 AS2/PX4 执行链路。用户认为在线热切换、自动选优、硬件自动适配、ROS2 原生化和完整模型仓库属于理想 Model Hub；Mission 未来可以成为将 Planner 等模块作为工具组合的 Agent。用户要求暂时记录这些远期需求，不在当前阶段展开。用户质疑生命周期状态是否被误当成无人机动作状态，并要求详细解释短时轨迹段及 AS2 Planner 当前输出。
- Flags: 确认统一 Planner 输出契约；区分模块执行状态、规划结果状态、飞行器物理状态。

### Q3 — Model Hub、时间同步与 Adapter
- Asked: 是否接受统一生命周期与任务语义、允许不同输出类型，再由 Output Gateway 归一化的两级契约？
- Captured: 用户认可这种架构，理解为针对不同类型 Planner 增加 adapter 并形成统一输出；询问是否按 Planner 输出数据格式分类。用户认可此前提出的 Model Hub 与 Mission Agent 设想并要求完整记录。用户把 Behavior 理解为 Aerostack2 运行时，把 Model Hub 暂理解为离线 Planner/算法下载适配器，并询问 AS2 缺少逐点时间戳是否导致无法多机飞行、是否直接控制各机速度即可。
- Flags: 明确 Model Hub 控制面与运行时数据面的边界；确定首期 Output Gateway 支持的标准输出 profile。

### Q4 — 单活 Planner 与统一时间化输出
- Asked: 第一阶段是否规定单任务只激活一个 Planner，通过配置选择 CAT-ORA 或 Swarm-Formation，不做并行协作？
- Captured: 用户明确希望只实现单个 Planner 激活，暂不需要并行；同时提出为 Swarm-Formation 加入时间信息，以统一两个 Planner 向下游的数据格式。
- Flags: 时间信息的精确定义；Swarm 流式单点如何封装为统一轨迹段；统一消息是否需要每点 `time_from_start`。

#### 接口审计补充
- Swarm-Formation 的 `quadrotor_msgs/PositionCommand` 字段为 `Header`、position、velocity、acceleration、yaw、yaw_dot、控制增益、trajectory id/status；`traj_server` 使用 `t_cur = now - start_time_` 从内部轨迹采样并将 `header.stamp = now`。
- 这说明现有 header 只表达当前命令的生成/采样时刻，不等价于未来轨迹点的预定执行时间。
- 可在 adapter 层形成统一时间轨迹段；若只封装当前点，则仍需定义短有效期和“新段替换旧段”。更稳妥的后续实现是从 Swarm 的内部轨迹接口导出一个短 horizon 的多个采样点。

### Q4 — Model Hub 元数据、Planner API 与首期选择策略
- Asked: 第一阶段是否规定“一个任务同时只激活一个 Planner”，通过配置选择 CAT-ORA 或 Swarm-Formation，不做两者并行协作？
- Captured: 用户确认第一阶段只需通过配置指定一个 Planner 工作。用户认可“Model Hub 控制面 → Model Hub 运行面 → Aerostack2 Planner Behavior → Planner Adapter”的架构，并将 Model Hub 类比为离线 Planner 下载器，其中“下载”包含由 Agent 完成适配的过程。用户希望现阶段明确 Planner 描述文件和 Planner 配置选择的作用；要求记录 Mission Agent 设想。用户进一步区分了其所称的适配器重点：将 CAT-ORA 的多机时间轨迹和 Swarm-Formation 的 PositionCommand 流经 Output Gateway 转换到共同下游，并询问统一 Planner API 的职责。用户认为为多机输出加入时间语义合理，同时追问 Swarm-Formation 编队保持是否也需要严格协同。
- Flags: 下一步确定统一任务请求的最小公共字段与算法专属扩展；随后确定首期两个输出 profile 的精确定义。

### Q5 — 配置数量、时间语义与公共信封
- Asked: 是否接受统一任务请求采用公共字段加 Planner 专属扩展？
- Captured: 用户确认“发现版本、兼容性检查、镜像下载/构建、描述文件生成、机群配置生成、部署并由 Planner Behavior 启动”正是其 Model Hub 设想。用户询问是否多个 Planner 各有描述文件、但系统配置文件只有一个。用户理解 CAT-ORA 是无机间感知的预先编队变化规划，Swarm-Formation 依靠机间状态感知形成持续闭环；认为给后者加入时间信息无伤大雅，并询问是否可将二者最终统一成带时间的 path。用户询问公共信封是否是 Planner/Behavior 向下暴露的信息。
- Flags: 需要用户确认输出统一到“带时间元数据的结果信封 + 两种 payload profile”，而不是强制统一为 timed path；随后再确定请求公共字段。

### Q6 — 统一时间轨迹段
- Asked: 是否接受统一 `TimedTrajectorySegment`，允许 CAT-ORA 和 Swarm-Formation 使用不同的轨迹长度与发布频率？
- Captured: 用户明确接受。两个 Planner 的南向输出采用同一数据结构；CAT-ORA 可输出较长轨迹段，Swarm-Formation 可高频输出短滚动轨迹段，不要求 horizon 和更新率一致。
- Flags: 确定 `valid_from` 的时钟语义、多机共同起始时间和过期处理规则。

### Q7 — Swarm 是否也能使用“基时间 + 偏移”
- Asked: 两个 Planner 的 `valid_from` 使用统一 ROS 时钟，轨迹点使用相对偏移，并定义共同起始和过期规则。
- Captured: 用户追问 Swarm-Formation 是否也可统一成“基时间 + 偏移”，担心此前说法会使南向数据格式不统一。
- Resolution: 可以且应当完全统一。源码进一步确认 Swarm 上游已经发布 `traj_utils/PolyTraj`，消息原生包含 `start_time`、分段 `duration` 和五阶多项式系数；`traj_server` 正是用该基时间计算 `t_cur`。因此 Swarm adapter 可直接订阅 `PolyTraj`，在短 horizon 内采样为多个 `TimedTrajectoryPoint`，不必从单点 `PositionCommand` 猜测未来，也不必改规划算法核心。
- Flags: 确认统一时钟/过期策略；确定 Swarm adapter 的采样周期与 horizon（实施参数，不必固化为跨 Planner 相同值）。

### Q8 — 当前阶段的时钟与过期策略
- Asked: 是否接受仿真使用 Gazebo/ROS `/clock`、真机使用同步系统时钟，并由 Output Gateway 拒绝过期轨迹段？
- Captured: 用户指出当前工作范围仅为仿真实现，不应把真机时钟同步带入本阶段；确认仿真统一使用 Gazebo/ROS `/clock`，并确认过期 `TimedTrajectorySegment` 必须由 Output Gateway 拒绝。
- Resolution: 当前阶段只实现和验收仿真时钟契约。Chrony/PTP 等真机时钟同步仅作为远期事项，不进入当前设计或验收范围。
- Flags: 过期或持续收不到有效轨迹后，执行链应采取悬停、刹停还是降落。

### Q9 — 轨迹失效后的安全动作
- Asked: 轨迹过期或持续收不到有效轨迹时，是否停止旧轨迹、发送定点悬停参考，并把 Planner 标记为 `FAILED/TIMEOUT`？
- Captured: 用户确认该处理正确，并要求写入文档；同时提出该能力以后可演变为安全控制功能。
- Resolution: 第一阶段将其作为 Output Gateway 的强制失效保护和验收项，不额外引入独立安全模块；远期可把轨迹新鲜度监控、悬停/刹停/返航/降落等策略提升为 Safety Supervisor。
- Flags: 确定 timeout 的具体阈值与“悬停参考点”取值；这些可作为 Planner profile/部署配置参数，而不必写死在统一消息中。

### Q10 — `PlanRequest` 的含义
- Asked: 是否接受第一阶段为两个 Planner 定义公共 `PlanRequest`，当前状态由统一 AS2 状态接口读取？
- Captured: 用户要求解释“任务统一输入”和 `PlanRequest` 的作用。
- Clarification: `PlanRequest` 是 Mission/Planner Behavior 发起一次规划任务的北向命令/Action Goal，表达“让当前已配置激活的 Planner 为哪些飞机、针对什么目标和约束开始工作”；它不是 Planner 运行期全部传感输入的集合，也不是南向轨迹输出。odom、点云、邻机状态继续走独立持续 topic。由于 Planner 由部署配置选择，删除草案中的 `selected_planner` 字段。
- Follow-up: 用户指出数据本应由 Mission 发给 Behavior，并认为更适合用此前约定的 Planner 配置文件和具体任务配置文件来指定任务及 Planner。
- Resolution: 用户的分层判断正确。第一阶段采用“任务配置 → Mission → Planner Behavior → Planner Adapter”的路径；不新增公开 `PlanRequest` 消息。任务配置就是声明式任务输入，Mission 将其解析为对现有 Behavior 接口的最小运行时调用。等未来出现远程动态提交任务、多个 Mission 客户端或稳定的 Model Hub API 需求时，再把配置结构固化为独立 `PlanRequest` schema。
- Flags: 确认首期任务配置的最小字段以及 Planner 参数由引用文件还是内联覆盖。

### Q11 — Planner 参数模板与任务覆盖
- Asked: 第一阶段是否只允许任务配置引用独立 Planner 参数文件，不允许在任务文件内覆盖算法专属参数？
- Captured: 用户不接受完全禁止覆盖；任务可以覆盖部分参数，缺省项直接复用 Planner 参数文件。Planner 参数文件的定位是可用模板。
- Resolution: 采用两层参数模型：`effective_params = planner_template + task_overrides`。模板提供经过验证的完整默认值，任务只声明差异；不再增加额外参数层级。每次启动应保存解析后的有效配置，保证实验可追溯。
- Flags: 确认合并规则（映射递归合并、数组整体替换、未知参数报错）和配置校验边界。

### Q12 — 参数合并规则
- Asked: 是否接受映射递归合并、标量覆盖、数组整体替换、未知参数报错，并保存最终有效配置？
- Captured: 用户接受。
- Resolution: 上述规则进入第一阶段配置解析器的实现与测试标准；不实现数组逐项合并或更多优先级层次。
- Flags: 确认首期任务目标是显式的逐机目标位姿，还是同时支持抽象队形模板/队形中心等高级语义。

### Q13 — Swarm-Formation 为什么需要目标
- Asked: 第一阶段任务目标是否只使用逐机目标位姿？
- Captured: 用户理解 CAT-ORA 需要直接目标，但质疑 Swarm-Formation 设定目标的用途。
- Source audit: 当前 Swarm-Formation 的 `formationWaypointCallback` 订阅 `/move_base_simple/goal` 作为 `swarm_central_pos`，然后为每机计算 `end_pt = swarm_central_pos + swarm_scale * relative_pos[drone_id]`。`relative_pos` 和 `swarm_scale` 来自 `normal_hexagon.yaml` 等 Planner 配置；现有七机复现也为每机配置了保持六边形偏移的终点。
- Resolution: Swarm 的目标决定编队目的地，持续规划决定到达路径。此前“首期只使用任意逐机目标位姿”的建议需要修正：更贴合两算法交集的任务语义是“目标编队中心 + 每机相对偏移/队形模板”，由 adapter 展开为 CAT-ORA 的逐机终点，并映射为 Swarm 的中心目标与队形参数。
- Follow-up: 用户决定先使用逐机目标坐标。
- Resolution: 将逐机目标坐标定为首期公共格式。对 Swarm，由 adapter 在任务启动前转换成目标中心和每机相对偏移；第一阶段单任务、单 Planner、配置驱动，因此无需支持运行中动态更换队形参数。
- Flags: 确认目标坐标首期是否只包含 xyz，还是必须包括逐机 yaw。

### Q14 — 逐机目标字段
- Asked: 第一阶段逐机目标是否只包含 xyz，不要求目标 yaw？
- Captured: 用户确认只包含 xyz。
- Resolution: 任务 schema 不含目标 yaw；`TimedTrajectorySegment` 仍可携带 Planner 生成的 yaw，二者不矛盾。
- Flags: 确认两个 Planner 共用的任务完成判据和容差参数。

### Q15 — 统一任务完成判据
- Asked: 是否以“全部无人机进入各自目标的位置容差并持续保持指定时间，期间无轨迹超时或 Planner 失败”为统一成功判据，并由 Mission 根据 AS2/PX4 实际位置反馈判断？
- Captured: 用户认为判据没有问题，但要求保持时间更短，例如 `1 s`。
- Resolution: 接受统一完成判据；默认值采用位置容差 `0.5 m`、连续保持 `1.0 s`、任务总超时 `120 s`，三个参数均可由任务配置覆盖。成功只能依据 AS2/PX4 实际位置反馈，不能仅相信 Planner 报告规划完成。
- Flags: 明确任务总超时后的状态原因和安全动作，并与轨迹更新超时区分。

### Q16 — 任务总超时
- Asked: 超过 `task_timeout` 后，是否由 Mission 标记 `FAILED/TASK_TIMEOUT`、取消 Planner Behavior、停止接受后续轨迹并让全部参与无人机定点悬停？
- Captured: 用户接受。
- Resolution: 采用上述统一处理；状态和原因分开表示。`TASK_TIMEOUT` 与 `TRAJECTORY_TIMEOUT` 是不同故障原因，但第一阶段采用相同的悬停安全动作。
- Flags: 明确用户主动取消任务的终态和安全动作。

### Q17 — 主动取消任务
- Asked: 用户或 Mission 主动取消时，是否标记 `CANCELED`、取消 Planner、丢弃后续旧轨迹、定点悬停，并要求新任务才能继续？
- Captured: 用户接受。
- Resolution: 主动取消不是故障；以 task id 隔离旧消息，取消后的任务不可恢复，只能启动新任务。
- Flags: 明确 `PAUSED` 是否在首期实现，以及恢复时能否复用暂停前轨迹。

### Q18 — 暂停与恢复
- Asked: 第一阶段是否实现 `PAUSED/RESUME`，暂停时悬停并清除旧轨迹，恢复时基于最新实际位置重新规划？
- Captured: 用户明确需要实现。
- Resolution: `PAUSED` 保留任务上下文但停止轨迹执行；`RESUME` 不续播旧轨迹，必须触发所选 Planner 从最新状态重新规划。
- Flags: 暂停期间是否停止累计 `task_timeout`；Planner 无法从最新状态恢复规划时的失败原因。

### Q19 — 暂停期间的超时计时
- Asked: 暂停期间是否停止累计 `task_timeout`，并停用轨迹新鲜度检查？
- Captured: 用户确认需要停止累计。
- Resolution: 暂停时冻结任务总超时计时并禁用轨迹超时；恢复时继续累计有效运行时间，并重新等待一条新轨迹。
- Flags: 恢复后 Planner 重规划失败或迟迟不给出第一段轨迹时的状态与等待阈值。

### Q20 — 恢复后无轨迹的失败语义
- Asked: 恢复后 Planner 无法重新规划或未及时给出第一段轨迹时，是否标记 `FAILED/REPLAN_FAILED` 并悬停？
- Captured: 用户指出“无法重新规划最后应该会走到超时情况”。
- Follow-up: 用户接受区分，并询问两种情况是否都必然需要实现。
- Resolution: 两种均为第一阶段必做路径。显式无解由 Planner/adapter 主动上报并立即失败；无响应无法从内容判断，只能由超时监控兜底。两者统一悬停。
- Flags: 首段轨迹等待超时与运行中轨迹更新超时是否使用两个独立的 Planner 模板参数。

### Q21 — 统一轨迹超时参数
- Asked: 是否分别设置 `initial_trajectory` 和 `trajectory_update` 两个超时参数？
- Captured: 用户认为这种设置近似把前者给 CAT-ORA、后者给 Swarm-Formation，不正确；二者应共用同一参数，再由具体 Planner 运行配置覆盖，或直接在各自 Planner 参数模板中设置不同默认值。
- Follow-up: 用户接受该精确定义。
- Resolution: 正式采用单一 `trajectory_timeout`。统一监控语义不是“多久没收到消息”，而是“连续多久没有有效轨迹覆盖当前执行时刻”；这样 CAT-ORA 的长轨迹在有效期内无需持续发布，Swarm 的短滚动轨迹则必须持续延长覆盖范围。默认值由各 Planner 模板分别设置，任务可覆盖。
- Flags: 确定轨迹段正常结束但尚未满足任务成功判据时的处理，尤其是完成判据所需的 1 秒保持窗口。

### Q22 — 轨迹结束与任务完成之间的窗口
- Asked: 轨迹正常结束后，是否由 Output Gateway 保持最后位置，让 Mission 完成 1 秒稳定性判断；若仍未满足成功条件，则累计 `trajectory_timeout`？
- Follow-up: 用户追问所称的 `trajectory_timeout` 是否就是整个任务超时，并提出若不是，应由整个 `task_timeout` 判断所有无人机是否都到达目标。用户接受轨迹结束后保持最后位置等其余规则，同时强调多机必须全部到达各自目标后再稳定 1 秒。
- Resolution: 修正此前建议。终点到达只受 `task_timeout` 约束；有限轨迹正常结束后，Gateway 保持最后位置，Mission 持续等待所有无人机同时进入容差并整体连续稳定 1 秒。任一无人机离开容差则保持计时重置。`trajectory_timeout` 只用于检测预期中的 Planner 轨迹供给中断。
- Flags: 为统一契约增加一种最小信号，用来区分“有限轨迹正常结束”与“滚动轨迹意外断流”；候选为 `is_final`。

### Q23 — 最后一段轨迹标记
- Asked: 是否在 `TimedTrajectorySegment` 中加入 `is_final`，区分 CAT-ORA 有限轨迹正常结束与 Swarm-Formation 滚动轨迹意外断流？
- Captured: 用户接受。
- Resolution: `is_final` 只描述 Planner 输出终结，不直接触发任务成功。最终段结束后保持终点并等待 Mission 完成判据；非最终段断流才使用 `trajectory_timeout`。
- Flags: 确认一个 `TimedTrajectorySegment` 是携带整个机群轨迹，还是每架无人机独立发布一条消息。

### Q24 — `TimedTrajectorySegment` 的层级与多机粒度
- Asked: 一条 `TimedTrajectorySegment` 应携带整个机群轨迹，还是单机轨迹？
- Captured: 用户询问它是否是 Behavior 层向下传递的数据格式，并据此认为一条消息应包含“多个机群”。
- Clarification: 它准确地说是 Planner Behavior 管理边界内由 Planner Adapter 输出给 Output Gateway 的统一南向规划格式，不是所有 Behavior 的通用格式。已确认用户倾向单条消息包含多机数据。
- Follow-up: 用户澄清其含义是“一条消息包含一个任务机群中的全部无人机轨迹”。
- Resolution: 一条 `TimedTrajectorySegment` 对应一个 task/机群，`vehicle_trajectories[]` 必须覆盖该任务的全部参与无人机；不在首期支持多个独立机群共用一条消息。
- Flags: 确认缺失、重复或包含未知 vehicle id 的不完整轨迹段是否整体拒绝。

### Q25 — 多机轨迹段的原子校验
- Asked: 若轨迹段缺失/重复/包含未知无人机、存在空轨迹或时间基准不一致，是否由 Output Gateway 整体拒绝并报告 `INVALID_TRAJECTORY_SET`？
- Captured: 用户选择整体拒绝。
- Resolution: 多机段不可部分接受或部分执行；任务参与车辆集合以 resolved task config 为唯一校验基准。
- Flags: 确认滚动段的单调序号、重复消息和乱序消息处理规则。

### Q26 — 轨迹段顺序与 `segment_id`
- Asked: 是否增加单调递增 `segment_id`，用它忽略重复段、拒绝乱序旧段和非当前任务消息？
- Captured: 用户接受顺序保护，但追问已有时间戳为何仍需额外字段。
- Analysis: `valid_from` 是执行基时间，不能可靠表达消息修订顺序；`generated_at` 才能用于新旧比较。在当前仅仿真、所有节点统一 Gazebo `/clock` 的范围内，可要求同一 task 的 `generated_at` 严格递增，并据此接受新段、忽略相同时间的重复段、拒绝更早段，因此 `segment_id` 可以暂不增加。
- Follow-up: 用户接受首期省略 `segment_id`，并认为 `task_id + generated_at` 比使用 `segment_id` 更合理。
- Resolution: 删除首期 `segment_id`。要求 adapter 保证同一 task 新版本的 `generated_at` 严格递增；Gateway 以它处理替换、重复和乱序。未来只有在同一时间戳多修订或跨时钟域的实际需求出现后，才考虑增加 revision。
- Flags: 确认每架无人机 points 内部的 `time_from_start` 是否必须非负、严格递增，以及各机终点时刻是否必须一致。

### Q27 — 轨迹点时间合法性
- Asked: 是否要求相对时间非负且逐机严格递增、机群共享 `valid_from`、允许点数不同但要求各机结束时刻一致？
- Captured: 用户接受。
- Resolution: 采用上述原子校验；提前结束的车辆由 adapter 在共同结束时刻补终点保持点。
- Flags: 确认每个轨迹点的 position/velocity/acceleration/yaw 哪些是必填，CAT-ORA 缺失字段如何生成。

### Q28 — 轨迹点必填与可选字段
- Asked: 是否规定 time/position 必填，velocity/acceleration/yaw 可选，并用 `valid_fields` 声明可信字段？
- Captured: 用户接受。
- Resolution: CAT-ORA 首期可以只声明 position 有效；Swarm 可声明原生提供的速度、加速度和 yaw。Output Gateway 不使用无效字段，adapter 不为格式统一而伪造数据。
- Flags: 确认机群段统一坐标系、frame id 和 ENU/NED 转换边界。

### Q29 — 统一坐标约定
- Asked: 是否将统一 Planner 输出固定为 ROS `world/ENU`，并仅在 AS2/Pixhawk 平台层转换为 PX4 NED？
- Captured: 用户不接受 ROS 坐标作为统一契约，要求使用 PX4 的坐标规定，因为后续系统可能不再使用 ROS。
- Resolution: 撤销 `world/ENU` 作为统一 Planner 契约的建议。统一数据模型应与 ROS 解耦，采用 PX4 坐标语义；现有 CAT-ORA/Swarm-Formation 若输出 ENU，则由各自 adapter 转换。当前仍需精确确认统一采用 PX4 `LOCAL_NED`，以及 yaw/机体系字段的约定。
- Source audit: PX4 官方文档确认 `TrajectorySetpoint` 的 position、velocity、acceleration 均为本地 NED；世界/本地轴为 North-East-Down，机体系为 Front-Right-Down。PX4 topic 默认采用 FRD/NED，除非消息另有说明。
- Flags: 用户确认首期是否固定为 `LOCAL_NED`；yaw 使用相对 NED x 轴、绕 down 轴的 PX4 `[-pi, pi]` 表达；是否需要在 schema 中保留坐标枚举。

### Q30 — `frame_id` 的含义
- Asked: 是否固定 `LOCAL_NED` 并删除消息中的自由文本 `frame_id`？
- Captured: 用户询问 `frame_id` 是什么；在解释轴方向与参考原点的区别后，接受由任务配置定义机群共享的 `LOCAL_NED` 原点，轨迹段通过 `task_id` 继承，不携带自由文本 `frame_id`。
- Clarification: `frame_id` 标识坐标数值所依附的参考系/原点，例如 world、map、某架无人机的 local frame；它与 NED/ENU 轴方向是两件事。多架 PX4 即使都使用 NED，若各自 local origin 不同，也不能直接比较或组成联合轨迹。
- Resolution: 第一阶段任务配置定义唯一的机群共享 `LOCAL_NED` 原点；同一任务的所有逐机目标和轨迹都相对该原点表达。`TimedTrajectorySegment` 不携带 ROS 风格的自由文本 `frame_id`，通过 `task_id` 继承 resolved task config 中的坐标定义。
- Flags: 确认统一 yaw 的方向、零点、单位与范围。

### Q31 — 统一 yaw 约定
- Asked: 是否完全沿用 PX4 yaw 约定：弧度、`0` 指向 NED `+X`（北）、`π/2` 指向 `+Y`（东）、范围 `[-π, π]`，且仅在 `valid_fields` 声明有效时执行？
- Captured: 用户要求“完全沿用px4”。
- Resolution: 统一契约不自定义 yaw 语义，完全继承 PX4 约定；算法若使用其他约定，由其 adapter 在边界转换。yaw 仍为可选字段。
- Flags: yaw 缺省时由下游保持当前航向，还是允许下游自行决定。

### Q32 — 可选字段缺省语义
- Asked: `valid_fields` 未声明 yaw 或其他可选字段时，是否完全沿用 PX4，以 `NaN` 表示 Planner 未指定，并禁止 Output Gateway 擅自补值或锁定航向？
- Captured: 用户接受。
- Resolution: Output Gateway 将所有未声明有效的可选 setpoint 字段映射为 PX4 `NaN`；不伪造速度、加速度或 yaw，也不在 Gateway 中引入隐式控制策略。position/time 仍按既定规则必填。
- Flags: 明确机群共享 `LOCAL_NED` 与每架 PX4 独立本地原点之间的转换来源和责任边界。

### Q33 — 共享坐标到逐机 PX4 原点的转换
- Asked: 是否由 Gazebo 多机出生位姿生成每架无人机的 `px4_origin_in_fleet_ned`，并由 Output Gateway 将机群共享 `LOCAL_NED` 轨迹平移到各 PX4 的本地 NED；不依赖 ROS TF，转换结果写入 resolved task config？
- Captured: 用户接受。
- Resolution: Planner/Adapter 始终产出机群共享 `LOCAL_NED`；Output Gateway 持有逐机原点映射并执行 `p_px4_i = p_fleet - origin_i`。仿真首期以 Gazebo 出生位姿为唯一来源，将解析结果固化到 resolved task config，保证可复现和可审计。
- Flags: 确定任务配置以文件路径还是 `planner_id + version` 引用 Planner 模板。

### Q34 — 任务配置引用 Planner 的方式
- Asked: 是否让任务通过 `planner.id + planner.version` 引用本地 Registry 中的 Planner 描述和默认参数模板，而不是直接写磁盘路径；任务局部 `parameters` 覆盖模板并生成 resolved task config？
- Captured: 用户接受。
- Resolution: 第一阶段使用最小本地目录 Registry，`id + 精确版本` 是稳定逻辑标识；不建设数据库、远程下载或完整 Model Hub。运行时解析默认模板、应用任务覆盖、严格校验并保存 resolved config。未来 Model Hub 接管 Registry 的解析/获取职责，不改变任务语义。
- Flags: 确认首期任务文件的最小公共顶层结构，以及算法专属字段是否只能放入 `planner.parameters`。

### Q35 — 首期最小任务结构
- Asked: 是否将公开任务文件限制为 `task.id`、`planner{id,version,parameters}`、`vehicles[]`、逐机 `targets{x,y,z}` 和 `completion`，算法专属参数只能进入 `planner.parameters`；固定坐标约定、逐机 PX4 原点、镜像/topic 和解析后的默认值只进入 resolved config？
- Captured: 用户接受。
- Resolution: 采用上述最小结构，不增加暂时无消费者的扩展字段。公开任务配置表达任务意图和必要覆盖，resolved task config 承载完整可执行结果。
- Flags: 区分可重复使用的任务配置标识与每次运行唯一的 `task_id`，避免旧运行的迟到轨迹污染新运行。

### Q36 — 配置名称与运行标识
- Asked: 是否将任务文件中的 `task.id` 改为可复用的 `task.name`，并由 Mission 为每次启动自动生成唯一 `task_id`；暂停/恢复沿用原 id，取消后重新启动生成新 id？
- Captured: 用户接受。
- Resolution: `task.name` 仅用于人类识别和复用配置；`task_id` 是一次运行的唯一身份，贯穿 resolved config、状态反馈和全部 `TimedTrajectorySegment`。Gateway 只接受当前活动 `task_id`，从根源隔离旧运行的迟到消息。
- Flags: 确认任务启动前的原子校验范围，以及配置无效时使用 `REJECTED/INVALID_TASK` 还是运行失败状态。

### Q37 — 启动前原子校验
- Asked: 是否在启动 Planner 和解锁无人机前，对 vehicle/target 精确对应、有限数值、Planner 版本、参数 schema、能力声明、部署配置和逐机 PX4 原点映射做整任务原子校验；失败记为 `REJECTED/INVALID_TASK` 且不发送飞行指令？
- Captured: 用户接受。
- Resolution: 配置解析或能力检查失败属于任务未进入运行态的拒绝，不记为运行中的 `FAILED`。任一参与无人机或 Planner 条件不满足即整体拒绝，不允许部分机群启动。
- Flags: 明确 arm/takeoff/land 属于 Mission 编排，还是纳入 Planner 任务生命周期。

### Q38 — Planner 与起降 Behavior 的职责
- Asked: 是否让 Mission/PX4 链路负责 arm/takeoff/land，而 CAT-ORA 与 Swarm-Formation 仅处理空中规划任务？
- Captured: 用户暂时接受，但明确希望后续起飞、降落等操作包含在 Behavior 中。
- Clarification: 正确分层应是 Mission 负责流程编排，Takeoff Behavior、Planner Behavior、Land Behavior 分别实现动作；不是由 Mission 自己实现起降，也不是把起降塞入 Planner Behavior。
- Resolution: 首期两个算法保持纯 Planner 职责；任务前后通过 Aerostack2 Behavior 接口完成起飞/降落。未来扩展 Behavior 目录与编排能力，不改变 Planner 契约。
- Follow-up: 用户确认“独立 Takeoff/Land Behavior 与 Planner Behavior 同级，由 Mission 串联”准确表达了其设想。
- Resolution: 正式确定 `Mission → Arm/Takeoff/Planner/Land/Disarm Behaviors` 的组合方式。Mission 负责选择、排序、组合和错误编排，各 Behavior 负责自身动作；Planner Behavior 不吸收起降职责。
- Flags: 确认多机 Behavior 的状态是机群级原子状态，还是允许逐机部分成功。

### Q39 — 单机/多机统一的 Behavior 状态
- Asked: 多机 Planner 是否只暴露一个机群级 Behavior 状态，全部完成才成功，任一不可恢复故障则整体失败并让其余可控无人机悬停，同时保留逐机诊断？
- Captured: 用户认为合理，并进一步指出单机 Behavior 也应统一成这种形式：单机只是仅包含一台无人机的多机 Behavior。
- Resolution: 系统不设计单机与多机两套 Behavior API；所有任务均面向规模至少为 1 的 `vehicles[]`。Behavior 对外提供一个机群级生命周期状态，内部携带逐机状态/错误明细。Planner 成功必须全体成功，不支持部分成功。
- Flags: 确认这一统一状态信封是否扩展到 Takeoff/Land 等所有机群 Behavior，而不仅是 Planner。

### Q40 — 全部 Behavior 的统一状态信封
- Asked: 是否让 Takeoff、Planner、Land 等所有机群 Behavior 共用 `BehaviorResult{behavior_type,task_id,overall_status,failure_reason,vehicle_results[]}`，单机仅有一个逐机结果，各 Behavior 自行定义全体成功条件？
- Captured: 用户接受。
- Resolution: 统一状态信封是 Behavior 层公共契约，不属于某一 Planner。Mission 主要消费机群级 `overall_status` 和原因，诊断/日志消费 `vehicle_results[]`；单机不使用简化版接口。
- Flags: 固定所有 Behavior 共享的生命周期状态集合与合法转换。

### Q41 — 统一 Behavior 状态机
- Asked: 是否统一采用 `REJECTED/RUNNING/PAUSED/SUCCEEDED/FAILED/CANCELED`，将 `START/PAUSE/RESUME/CANCEL/GET_STATUS` 作为命令，将 `TIMEOUT/NO_SOLUTION/INVALID_TASK` 等作为原因码，并取消含义模糊的 `STOPPED`？
- Captured: 用户两次明确接受这套统一状态机；第二次确认不视为对后续失败恢复策略的答复。
- Resolution: `START` 校验失败进入 `REJECTED`，通过则进入 `RUNNING`；`RUNNING` 可进入 `PAUSED` 或四种终态，`PAUSED` 仅可恢复到 `RUNNING` 或进入失败/取消终态。`REJECTED/SUCCEEDED/FAILED/CANCELED` 均不可恢复。状态与原因码严格分离。
- Flags: 确认 Behavior 失败后由 Mission 选择恢复 Behavior，还是由失败的 Behavior 自行自动降落。

### Q42 — Behavior 失败后的恢复策略
- Asked: 是否由失败的 Behavior 先执行即时止险并返回 `FAILED`，再由 Mission 根据 `on_failure` 启动恢复 Behavior；首期使用所谓 `hover_and_abort`？
- Captured: 用户询问 `hover_and_abort` 是什么。
- Clarification: 它只是提议中的策略简称，不是 Aerostack2 状态、PX4 模式或新的复合 Behavior。准确语义是：失败源先维持安全参考，Mission 随即启动独立 Hover Behavior 接管全部仍可控的空中车辆，同时终止当前 Mission 的正常流程并将 Mission 标记为失败；不会自动继续到 Land/Disarm。
- Recommendation: 删除含混的 `hover_and_abort` 名称，直接在 Mission 策略中表达“恢复 Behavior = Hover；Mission 结果 = FAILED”。
- Follow-up: 用户接受这一首期默认策略。
- Resolution: 失败源先维持安全参考，Mission 启动独立 Hover Behavior 接管全部仍可控的空中车辆，并将当前 Mission 终止为 `FAILED`。首期不自动进入 Land/Disarm；人工操作或新 Mission 决定后续动作。文档和配置不再使用含混的 `hover_and_abort` 名称。
- Flags: 固定 Planner 注册描述文件的最小字段。

### Q43 — 最小 Planner 描述文件
- Asked: 是否让本地 Registry 中每个 Planner 使用统一最小描述结构，包含身份/版本、镜像/命令/adapter、默认参数/schema、batch 或 streaming 能力、机数范围、所需输入、`TimedTrajectorySegment` 接口版本、CPU/内存/GPU需求和支持平台？首期不加入评分、自动下载、许可证管理和远程仓库字段。
- Captured: 用户接受。
- Resolution: CAT-ORA 与 Swarm-Formation 使用相同 manifest schema，仅字段值不同。首期本地 Registry 只负责按 `id + 精确版本` 查找、校验和提供运行元数据；Model Hub 的远程获取、评估、部署和硬件自动适配留到后续。
- Flags: 最终确认第一阶段的整体完成判据与明确排除项。

### Q44 — 第一阶段完成判据
- Asked: 是否以同一 Aerostack2 工程注册两个 Planner、配置单选、统一任务/轨迹/状态/坐标接口、PX4 SITL + Gazebo 多机真实执行、完整生命周期与异常测试、冷启动部署文档作为完成条件，并明确排除并行协作、热切换、自动择优、完整 Model Hub、核心 ROS2 重写、真机/国产 OS 和 Mission Agent？
- Captured: 用户接受。
- Resolution: 以上内容构成第一阶段正式验收边界。超出项保留为后续 Model Hub、Mission Agent、硬件适配和产品化阶段目标，不得混入首期实施范围。
- Flags: 无。

## Open flags (pending input)

- 第一阶段架构决策没有待用户补充的问题。
- 后续专题：完整 Model Hub 控制面/运行面、真机时钟同步、硬件自动适配、Mission Agent。
- 实施阶段再确定配置文件的物理目录布局；任务语义、合并规则和 resolved config 已定，不需要再次做架构决策。
