# AS2 运动参考接入

本包把公共 Planner 的 `FleetSetpoint` 转成逐机 AS2 `motion_reference`，
处理本地 NED 到 ENU、yaw 转换、控制模式请求与参考发布。
它由多个算法共享，归属 Planner framework，是 Library 与 AS2 Runtime
之间的输出桥接模块。

ROS 包名为 `uav_as2_setpoint_adapter`，执行入口为
`as2_setpoint_adapter_node`。现有 `planner_framework` 仍负责启动该节点；
本次目录迁移没有修改进程生命周期。

当前输入依赖 `uav_planner_interfaces`，属于既有 Planner 执行链的接入，
并非所有 Library 都必须采用的统一控制接口。

历史整栈部署位于 `main`，当前分支不再提供其 Planner runtime 镜像配方。
`scripts/smoke_ros2.sh` 需要已准备的 ROS 2、AS2 与 Planner 安装环境，
第一个参数为仓库根目录。本包仍作为 Library 与 Runtime 的输出桥保留。

直接从仓库源码构建时，colcon 不会递归发现已识别包内部的嵌套包，
需显式指定本包路径，例如：

```bash
colcon build --paths planner_interfaces planner_framework planner_framework/bridges/as2_setpoint
```

上述命令需要已准备的 ROS 2 与 AS2 依赖环境。
