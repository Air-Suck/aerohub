# AS2 单参考轨迹执行

公共桥 `as2_setpoint_adapter_node` 将 `FleetSetpoint` 的逐机局部 NED PVA 转为
`motion_reference/trajectory` 的逐机局部 ENU PVA。规划框架继续负责采样、插值和时间调度。
缺省 velocity / acceleration 的全 NaN 保留；没有 yaw 要求时使用接管时的局部航向。
桥观察 `controller/info`，不申请控制模式，不因收到 topic 自动接管。
接管航向取自 `sensor_measurements/odom`，不能将 `earth` 下的定位姿态冒充局部姿态。

CAT-ORA 当前使用其自有 `cat_ora_as2_adapter_node` 入口复用本包转换实现，
并将任务的暂停、取消、失败和完成转为 AS2 HOVER 请求及确认；仍不自动申请
TRAJECTORY。输入、停止与故障回归见 [CAT-ORA adapter](../../../src/planner/cat_ora/ros2_adapter/README.md)。
其他算法的原有入口保持兼容，不要求串联 CAT-ORA 的适配节点。

每条 `TrajectorySetpoints` 必须恰好一个点，frame 为 `<namespace>/odom`。
position 和 yaw 必须有限；twist、acceleration 各自必须全有限或全 NaN。
header.stamp 是源采样时间，控制周期不得改写。旧点不排队、重放或插值。

AS2 控制器和 Pixhawk 平台默认 50 Hz。控制器直接旁路传递完整 PVA，平台按
(x,y,z)→(y,x,-z) 映射至 NED；yaw 转为 pi/2-yaw。PX4 timestamp 使用本次发送时间。

控制器参数 `trajectory.max_age=0.3`、`trajectory.max_future=0.05`、
`trajectory.timeout=0.5`（秒）。平台也检查源时间窗口，修改时间窗口时在平台同步配置
`trajectory.max_age`、`trajectory.max_future`。进入模式时清空参考并开始等待；连续
500 ms 没有有效新样本就锁定并异步请求现有悬停流程。非法、重复、倒序样本不续期。
超过 300 ms 的参考停止转发。失败时报告错误，恢复必须重新显式申请轨迹模式。

## SUPER

`super_as2_adapter_node` 只将 `mars_quadrotor_msgs/PositionCommand` 转为单点轨迹。
接受 SUPER 主轨迹 flag=1 和备用轨迹 flag=2；拒绝其他状态。完整 PVA 必须有限，
输入必须已经是配置的局部 ENU frame，不隐式重命名 frame。
adapter 默认禁用，由任务调用 `<namespace>/super_adapter/enable`（SetBool）启用；
启用前必须已建立 TRAJECTORY + YAW_ANGLE 并收到局部里程计。离开该模式自动禁用。
默认固定接管航向；显式设置 `follow_yaw:=true` 才跟随 SUPER yaw。

```bash
ros2 run uav_as2_setpoint_adapter super_as2_adapter_node --ros-args \
  -r __ns:=/drone0 -p use_sim_time:=true \
  -p command_topic:=/planning/pos_cmd -p frame_id:=drone0/odom
ros2 run uav_as2_setpoint_adapter super_as2_mission --ros-args \
  -r __ns:=/drone0 -p use_sim_time:=true -p frame_id:=drone0/odom \
  -p goal:='[2.0, 0.0, 1.5]'
```

目标为局部 ENU 绝对坐标，起飞高度为相对高度；非零地面原点时相应调整目标。
任务执行：AS2 起飞完成 → 显式申请轨迹模式 → 启用 adapter → 发布 SUPER 目标 →
稳定到达 → 显式悬停 → 禁用 adapter → AS2 降落并确认解除解锁。
SUPER 和 adapter 应提前启动，避免模式进入后的 500 ms 等待窗口耗在启动进程。
同机同时只能启用一个参考来源，CAT-ORA / Swarm 桥与 SUPER adapter 不同时激活。

SUPER 使用 `thirdparty/SUPER-sim/super_planner/config/as2_trajectory_ros2.yaml`：
执行定时器开启，速度限制 1 m/s，保留主/备用轨迹和重规划。
虚拟地面与天花板也是局部 odom 坐标，必须覆盖当前定位原点和目标。

## 验证

- `tests/test_trajectory.py`：真实 ROS 消息下的主/备用 PVA、固定/跟随航向、非法输入、
  公共桥仅位置与 NaN 保留、失流后禁止自动恢复。
- `thirdparty/aerostack2/as2_core/tests/trajectory/trajectory_contract_test.cpp`：
  无 ROS 运行时的数值、单点和时间契约测试。
- `thirdparty/aerostack2/as2_motion_controller/tests/trajectory_stream_ros_test.py`：
  实际控制器的 20/50/100 Hz 输入、50 Hz 输出、源时间戳、重复参考失流和恢复锁定。
  该驱动需要在隔离 ROS domain 中启动 namespace=stream 的控制器；它提供模拟平台服务。
- `scripts/smoke_ros2.sh`：公共桥端到端 smoke，任务侧显式请求轨迹模式。

构建时显式列出嵌套包及 SUPER 消息：

```bash
colcon build --paths src/planner/interfaces src/planner/framework \
  src/behavior/legacy_setpoint \
  thirdparty/SUPER-sim/mars_uav_sim/mars_quadrotor_msgs
```

需要已有 ROS 2 Humble 与 AS2 安装环境。传感器线程及相机配置不由本包修改。
