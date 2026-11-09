# Runtime 平台支撑

Runtime 为用户 App 和算法 Library 提供状态、传感器、时钟与运动执行能力。
AirSim 使用的 AS2 fork 由 `as2/airsim/` 管理；传感器桥在 `simulator/airsim/sensors/`，导航输入整理在 `src/perception/navigation_inputs/`。

| 目录 | 职责 | ROS 包名 |
| --- | --- | --- |
| [as2](as2/README.md) | 共用的 AS2 节点启动器与交互验收 | 使用 AS2 上游包 |
| [airsim_sensors](airsim_sensors/README.md) | AirSim 图像、CameraInfo、LiDAR、静态 TF 和独立时钟节点 | `uav_as2_sensor_adapter` |

目录按职责命名，ROS 包名和可执行入口保持兼容。算法专用 adapter 与 proxy
位于 `libraries/<name>/`。部署装配仍由根目录 `deployment/` 维护，用户场景
和 settings 位于 `deployment/airsim/config/`。

这里是当前自建接入代码的归属，不代表已经迁入全部 AS2 能力或实现统一
Runtime 生命周期。Planner 专用的 AS2 输出桥位于
`src/behavior/legacy_setpoint/`，由 Planner framework 管理。
