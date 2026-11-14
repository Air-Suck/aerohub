# 功能卡片静态规范（第一版）

更新时间：2026-11-14。

本文说明第一版功能卡片及其类型词典。它定义静态契约，不表示运行时能力发现、硬件检查、资源仲裁或成功判据已经实现。

## 1. 文件职责

| 文件 | 职责 |
| --- | --- |
| `assets/capability.yaml` | 描述一个底层能力或一个可由上层 Agent 选用的 Tool |
| `assets/capability-types.yaml` | 统一定义数据能力路径及其 ROS topic、消息与数据语义 |
| `assets/hardware-types.yaml` | 统一定义 Card 可引用的硬件类型；不描述实际设备实例 |

项目实际使用的规范表保存在 `cards/capability-types.yaml` 和 `cards/hardware-types.yaml`；`assets/` 文件是创建或扩展这些规范表时使用的模板。

所有 YAML 必须只使用可无损映射到 JSON 的结构。第一版不允许未定义的顶层字段，也不提供任意扩展区；需要新表达能力时，应正式扩展规范并更新 `schema_version`。

## 2. Card 顶层字段

除 `demo` 外，以下顶层字段均为必填。没有相应内容时使用空列表或明确的 `null`，不要省略字段来掩盖信息缺失。

| 字段 | 含义 |
| --- | --- |
| `schema_version` | Card 格式版本 |
| `id` | 项目内稳定且唯一的能力标识 |
| `version` | 对外能力契约版本；不等同于源码提交或镜像版本 |
| `type` | `capability` 表示供其他能力依赖的底层能力；`tool` 表示可供上层 Agent 选择的任务入口 |
| `name` | 面向人类和 Agent 的简短名称 |
| `description` | 整个能力或 Tool 能做什么，不重复解释输入输出类型 |
| `implementation` | 底层组件、部署清单、共享运行时及耦合能力关联 |
| `inputs` / `outputs` | 数据能力依赖和产出 |
| `interfaces` | 请求响应、长任务和生命周期等操作接口 |
| `lifecycle` | 生命周期管理权限及逻辑操作引用 |
| `hardware_requirements` | 部署和运行所需的硬件类型 |
| `resource_access` | 运行期间对共享逻辑资源的访问声明 |
| `demo` | 可选的展示描述，不属于功能依赖契约 |

## 3. 标识、版本与实现关联

`id` 应使用稳定的层级名称，例如 `state-estimation/pin-slam` 或 `navigation/person-following`。`version` 在输入输出、操作接口或外部行为发生不兼容改变时更新。

`implementation.component_id` 和 `manifest` 指向实际组件与部署清单。源码提交、镜像摘要和 overlay 等实现版本由组件或部署清单记录，不复制进 Card。

`shared_runtime` 和 `coupled_capabilities` 用于当前紧耦合实现。例如同一 PIN-SLAM 进程可以同时提供定位与神经点地图；停止共享运行时可能影响两张 Card。这是当前适配手段，长期边界需要在项目架构设计中继续复审。

## 4. 数据能力与精确匹配

`inputs` 和 `outputs` 中的 `capability_type` 同时表示数据语义和标准逻辑 ROS topic 接口。Card 不重复声明消息类型、单位、坐标或时间语义，这些内容只在能力类型定义表中维护。

依赖解析采用完整路径精确匹配：

```text
consumer.inputs[*].capability_type
    == provider.outputs[*].capability_type
```

相同路径必须表示相同的连接契约。在不考虑性能和效果差异时，声明相同输出类型的提供者应具有相同的可接入消费者集合。父路径不自动匹配子路径，也不进行模糊匹配。

能力类型条目定义：

- 稳定的 `id`、名称和自然语言含义；
- ROS topic 消息 Schema 和 QoS；
- 单位、坐标、轴向与时间戳语义。

具体 topic 名称和实际 `frame_id` 属于部署或运行时实例绑定。同一数据经过 Adapter 或坐标转换、满足目标类型的完整规范后，才能声明为该目标类型。坐标转换不能自动解决数据模型、单位、时间语义或传感器语义差异。

## 5. 操作接口与调用参数

数据流不写入 `interfaces`；它由 `capability_type` 定义。`interfaces` 只描述操作，包括：

- `request_response`：一次请求和一次响应，通常对应 ROS 2 Service；
- `managed_task`：具有执行过程的任务，通常对应 ROS 2 Action，也允许描述当前真实实现采用的 ROS 2 命令 topic；
- 生命周期相关操作。

每个接口使用 Card 内唯一的逻辑 `id`，声明用途、交互方式、协议和真实 ROS Schema。具体 service/action 地址由部署配置或运行时能力实例提供。

Card 不复制具体参数字段。`parameter_schema` 可以引用由真实接口、适配或部署阶段生成的外部参数定义；Online Agent 只为具体调用填写参数值，运行时校验通过后才能调用 Tool。

## 6. 生命周期与权限

所有能力采用统一的启动、状态、运行和停止模型，但第一版只描述管理权限和实际存在的逻辑操作：

| `management` | 含义 |
| --- | --- |
| `system_managed` | Agent 可以发现和使用能力，但不能自行启停 |
| `agent_managed` | Agent 可以通过获准的生命周期接口按需管理 |
| `shared_managed` | 能力允许管理，但操作会影响共享运行时或其他能力，必须由部署/运行时协调 |

`lifecycle.operations` 的非空值必须引用 `interfaces[].id`。就绪、健康、成功、停止完成和资源释放判据尚未形成项目标准，第一版不在 Card 中虚构这些条件。

## 7. 硬件要求

`hardware_requirements` 引用硬件类型词典并采用精确匹配。它回答“运行该能力需要什么设备或算力”，而 `inputs` 回答“该能力需要什么数据”。即使依赖图最终落到硬件节点，也不应将中间数据能力直接绑定到具体设备实例。

硬件类型词典只定义稳定类型、名称和含义。实际无人机的设备实例、车辆归属、驱动、数据访问地址、标定和动态状态需要单独的硬件配置规范。

## 8. 资源访问

`resource_access` 用于声明能力运行期间对共享逻辑资源的访问：

- `reads`：只读访问；
- `shared_writes`：允许多个经过协调的写入者；
- `exclusive_writes`：必须独占的写入权，例如 `drone/motion`。

该字段支持发现并发冲突，但不实现锁或同步。实际互斥、停止确认、控制权交接和资源释放必须由运行时软件强制执行，不能依赖 Agent 自律。

## 9. Demo 表示

`demo.representations` 独立描述值得展示的中间表示和最终效果。每个条目包含：

- 本地唯一 `id`；
- `intermediate` 或 `final` 展示阶段；
- 自然语言说明；
- 逻辑展示接口的协议与消息 Schema；
- 建议渲染器类型。

具体 topic 地址由部署或运行时实例提供。`demo` 不参与 Agent 的功能依赖匹配；即使某个展示表示具有 ROS topic，也不能因此作为其他能力的正式输入。需要进入执行流的数据必须另行声明为正式 `outputs`。

## 10. 第一版不覆盖的内容

以下内容已有设计方向，但不属于当前静态模板：

- 运行时能力注册、发现、心跳和可用状态；
- 硬件可用性的动态检查及错误原因；
- 就绪、健康、成功和停止判据的阈值校准；
- 资源仲裁、停止确认和控制权安全交接；
- 通用坐标转换服务；
- 实际无人机硬件配置与运行时接口绑定；
- 紧耦合组件的长期拆分边界。

这些问题应记录在项目的架构待办或相应设计文档中，不得把示例值提升为已经确定的项目标准。

## 11. 校验

在 AeroHub 仓库根目录执行：

```bash
python3 skill/uav-capability-cards/scripts/validate_cards.py .
```

校验器检查固定字段、枚举、Card/能力/硬件 ID、精确类型引用、组件清单关联、耦合能力反向引用和 JSON 可表示性。它只检查静态契约，不能替代源码接口审查或运行时验证。
