# AS2 ↔ PX4 1.18 消息布局

这些文件原样取自 flight_board 使用的 PX4 固件提交
`61907962c9220437ec01a98fa26bfbf5cbb254b5`，不是来自移动的 main 分支：

- [FailsafeFlags](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/msg/FailsafeFlags.msg)：包含高度丢失、降落伞、交通规避、Remote ID 与 GNSS 健康标志。
- [SensorGps](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/msg/SensorGps.msg)：新增天线偏移与改正数协议字段。
- [VehicleControlMode](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/msg/VehicleControlMode.msg)：删去旧版控制标志，影响后续字段的解码。
- [VehicleCommandAck](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/msg/versioned/VehicleCommandAck.msg)：使用消息版本 1。
- [VehicleStatus](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/msg/versioned/VehicleStatus.msg)：包含可显示导航状态、Offboard 接受状态与交通规避系统状态。

独立的 PX4 接口依赖镜像（`deployment/onboard_computer/px4_msgs/Dockerfile`）在固定的
px4_msgs 源码上覆盖这些定义后编译，onboard_computer 直接使用其安装产物。其余插件使用的
SensorCombined、BatteryStatus、VehicleOdometry、OffboardControlMode、
TrajectorySetpoint、VehicleAttitudeSetpoint、VehicleRatesSetpoint、VehicleCommand、
TimesyncStatus、ManualControlSwitches 已与该固件逐项核对，线上的字段类型和顺序一致。
部分常量、注释不同不影响序列化；不要据此宣称整个 px4_msgs 包都与固件一致。增加新订阅时必须
再次核对对应固件消息布局，不能仅依赖 ROS 类型名称相同。

版本 1 的 topic 必须显式带 `_v1`：battery_status、vehicle_attitude_setpoint、
vehicle_command_ack。仅改 topic 名不能修复消息布局。

离线校验原始文件：`cd deployment/onboard_computer/px4_msgs/overrides && sha256sum -c SHA256SUMS`。
迁移到其他固件时重新核对全部收发消息，不能沿用此处的兼容结论。
