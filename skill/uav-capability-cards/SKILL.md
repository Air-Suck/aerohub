---
name: uav-capability-cards
description: 为 UAV 底层能力和 Agent Tool 创建或更新机器可读的功能卡片，并维护其精确匹配的数据能力类型与硬件类型契约。适用于 cards/ 下的新卡片、旧卡片迁移和卡片契约审查。
---

# UAV 功能卡片

## 使用前读取

创建或修改 Card 前，完整读取：

- [Card Schema 规范](references/card-schema.md)
- [Card 模板](assets/capability.yaml)
- 涉及新的数据能力路径时读取 [能力类型模板](assets/capability-types.yaml)
- 涉及新的硬件要求路径时读取 [硬件类型模板](assets/hardware-types.yaml)

模板是 AeroHub Card Schema v1 的固定骨架，不是 AS2 官方格式。不要从旧卡片反推新字段。

## 先核查实现

核查源码、ROS 接口、launch、组件清单和验证记录，不依据论文、目录名或计划目标虚构能力。明确：

- 这是供其他能力依赖的 `capability`，还是可由上层 Agent 直接选择的 `tool`；
- 数据输入和输出、操作接口、生命周期管理权限、硬件要求和逻辑资源访问；
- 是否与其他 Card 共用进程、设备或启停边界；
- 哪些中间或最终表示仅供 Demo 展示。

规划、Behavior、状态估计、感知、推理等是实现类别，不是 v1 的 Card `type`。它们既可能是底层 `capability`，也可能参与实现上层 `tool`。

## 编写流程

1. 从 `assets/capability.yaml` 复制固定骨架；除可选 `demo` 外不得省略顶层字段。空内容使用 `[]` 或明确的 `null`。
2. 使用稳定、唯一的层级 `id`，填写 Card Schema 版本和能力契约版本。`name`、`description` 只描述整个能力或 Tool。
3. `inputs` 和 `outputs` 只引用已定义的 `capability_type`。数据路径完整精确匹配；不得用父子路径、相似名字或自然语言推断兼容。
4. 若路径尚未定义，依据真实消息、单位、坐标、轴向、时间戳和 QoS，在 `cards/capability-types.yaml` 中新增定义；使用 `assets/capability-types.yaml` 的结构。不要在单张 Card 中私自解释路径。
5. `interfaces` 只填写请求响应、长任务和生命周期等操作接口。引用真实 ROS Schema；具体 topic、service、action 地址留给部署配置或运行时实例。
6. Card 只引用参数 Schema。Online Agent 生成调用参数值，不生成接口字段；运行时必须校验参数后再调用。
7. `hardware_requirements` 只引用 `cards/hardware-types.yaml` 中的规范硬件类型。新的硬件路径先按 `assets/hardware-types.yaml` 的结构进入统一类型表；实际设备实例和动态状态不写入 Card。
8. `resource_access` 声明读取、共享写入和独占写入的逻辑资源。它不实现锁、停止确认或控制权交接，不得声称 Card 已提供运行时安全保证。
9. `demo` 只声明值得展示的中间表示和最终效果，不参与功能依赖匹配。展示数据若需被其他能力消费，必须同时成为正式 `outputs`。
10. 核对共享运行时、耦合能力和生命周期权限。系统托管能力不能因存在启停实现就标为 Agent 可管理。
11. 在仓库根目录运行 `python3 skill/uav-capability-cards/scripts/validate_cards.py .`；修复所有结构、引用和组件清单错误后再交付。

## 静态边界

Card v1 不包含动态就绪、健康、成功、停止完成、失败重试、资源释放或硬件在线判据。这些标准尚需结合实现和证据校准；不要填入任意阈值，也不要把旧模板字段带入 v1。

验证阶段、源码提交、镜像摘要、overlay 和原始证据保存在组件清单、组件文档和 `skill/temp/` / `log/`，不复制到 Card。Card 中的 `implementation.manifest` 负责关联组件清单。

## 兼容与交付

新 Card 必须使用 v1 模板。修改旧 Schema Card 时，仅在用户任务涵盖迁移时迁移；不要顺手批量改写无关卡片。交付时说明：

- 创建或迁移的 Card；
- 新增或复用的能力类型与硬件类型；
- 与真实接口、组件清单和共享运行时的核对结果；
- 仍需运行时机制解决的缺口。
