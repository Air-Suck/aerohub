# 配置入口

根目录 [deployment/airsim/libraries.yaml](../libraries.yaml) 是面向用户的算法能力库注册表。
每个已接入算法是一个 library；传感器桥、坐标转换和 ROS proxy 属于支撑实现。

架构分为三层：用户 App 定义场景与任务编排，Library 提供可复用算法能力，
AS2 Runtime 提供 Behavior、状态估计、控制与平台接入。App 可以组合 Library，
也可以直接调用 Runtime 的基础 Behavior。Library 表示能力归属，不要求算法
必须以进程内函数库形式运行；现有 Planner 仍通过各自进程和 ROS 接口执行。

选择 library 后，查看其 `bindings`，选择已有运行组合，再使用对应的任务配置与
启动入口。当前 `launch` 是供用户执行的命令描述，尚无通用 library 启动器读取它。
历史运行记录位于 `history`，它只指向对应分支与证据，不提供当前启动命令。

- `environments`：模拟器、飞控和部署组合；AirSim 的具体地图由 scene profile 选择。
- `libraries`：稳定 library ID、用途、实现与已有运行组合。ID 与现有 Planner Registry 一致。
- `manifest`：引用运行时 Registry 中的算法版本、能力及参数，不另复制一份。
- `bindings`：library 在某个 environment 下的配置、入口与验证状态。
- `history`：main 中的历史环境和验收记录，不属于当前可运行组合。
- `validation: not_integrated`：尚未接通，不提供可执行的 launch。

新增算法时注册 library；算法接入新场景时增加或完善 binding。
library 的注册不代表它支持所有场景。场景启动配置与 library 任务配置分别维护，
本机 binary 路径通过资源名解析，不写进 library 注册表。

本目录当前包含：

| 配置 | 用途 |
| --- | --- |
| [resources.example.yaml](../../../simulator/airsim/config/resources.example.yaml) | 场景 binary 等外部资源路径示例，归 simulator 管理 |
| `simulator/airsim/config/resources.local.yaml` | 被 Git 忽略的本机资源路径，覆盖用户级映射 |
| [as2/](as2/README.md) | AS2 节点、机群与 PID 配置 |
| [dds/](dds/README.md) | 公共 DDS UDP 配置 |
| [airsim/](../../../simulator/airsim/config/README.md) | 场景 profile、PX4 与传感器 settings |

现有 Planner 任务与部署参数仍在 `src/planner/framework/examples/`，注册表引用它们。
