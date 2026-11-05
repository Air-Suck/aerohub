# Planner 执行框架

这套框架是算法 Library 与 AS2 Runtime 之间的桥梁：向上管理规划任务和
算法 adapter，向下校验、调度轨迹并交给 AS2 执行。

```text
算法 Library → TimedTrajectorySegment
                    ↓
        Planner 生命周期与 Output Gateway
                    ↓ FleetSetpoint
        bridges/as2_setpoint
                    ↓ motion_reference
               AS2 Runtime
```

- `uav_planner_framework/`：任务配置、Registry、生命周期、轨迹处理与完成判定。
- [bridges/as2_setpoint](bridges/as2_setpoint/README.md)：独立 ROS 输出桥，包含转换、控制模式请求与发布。
- `registry/`、`examples/`：现有算法 manifest、任务与部署配置。
- 根目录 `planner_interfaces/`：这条执行链的公共 ROS 消息与服务契约。

输出桥由 framework 启动，保持 `uav_as2_setpoint_adapter` 包名与既有执行入口。
它服务于本套 Planner 契约；直接调用 AS2 Behavior 的其他 Library 不必经过它。
