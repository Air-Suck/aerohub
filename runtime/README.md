# Runtime 平台支撑

Runtime 为用户 App 和算法 Library 提供状态、传感器、时钟与运动执行能力。
AS2 核心源码由 `thirdparty/` 管理；本目录维护项目自己的平台接入包。

| 目录 | 职责 | ROS 包名 |
| --- | --- | --- |
| [as2](as2/README.md) | 共用的 AS2 节点启动器与交互验收 | 使用 AS2 上游包 |
| [airsim_sensors](airsim_sensors/README.md) | AirSim 图像、CameraInfo、LiDAR、静态 TF 和独立时钟节点 | `uav_as2_sensor_adapter` |

目录按职责命名，ROS 包名和可执行入口保持兼容。算法专用 adapter 与 proxy
位于 `libraries/<name>/`。部署装配仍由根目录 `deployment/` 维护，用户场景
和 settings 位于 `config/`。

这里是当前自建接入代码的归属，不代表已经迁入全部 AS2 能力或实现统一
Runtime 生命周期。Planner 专用的 AS2 输出桥位于
`planner_framework/bridges/as2_setpoint/`，由 Planner framework 管理。
