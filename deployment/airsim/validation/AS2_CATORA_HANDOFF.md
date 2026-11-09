# CAT-ORA 迁移接入新版 AS2

本文件保留 `b5042e0` 时的接口交接约定；两库后续部署与验收已记录在主仓库
`02ed54d`。当前 AS2 另包含 executor 复用提交 `f387e0d7`。CAT-ORA 的插值、时间调度和多机协调继续留在 Planner framework；
逐机执行使用公共 AS2 单点轨迹桥。先核对下面的源码和接入约定，再确认新部署包含这些版本。

## 更新位置和版本

以下是当时的接口基线版本；当前实际版本以主仓库 submodule 指针为准。

| 位置 | 版本 | 更新内容 |
| --- | --- | --- |
| `thirdparty/aerostack2` | `8f1ffce9`，包含 `4c56b9f9` | 50 Hz 单点执行、数值和时间检查、失流悬停及恢复锁定；非零原点起飞完成条件修复 |
| `thirdparty/as2_platform_pixhawk` | `80f0f27` | TRAJECTORY + YAW_ANGLE、完整 PVA 的 ENU→NED 输出、基类参考有效期检查 |
| `planner_framework/bridges/as2_setpoint` | 主仓库 `c0a8469` | FleetSetpoint 转单点 TrajectorySetpoints；由任务显式申请控制权 |
| `src/planner/super/upstream` | `3bfa403` | AS2 定位和点云输入、低速执行配置；CAT-ORA 无须运行 SUPER 算法 |

重点代码：

- [轨迹契约与 Freshness](aerostack2/as2_core/include/as2_core/utils/trajectory_reference.hpp)。
- [ControllerHandler](aerostack2/as2_motion_controller/src/controller_handler.cpp)：
  `refTrajCallback`、`setControlModeSrvCall`、`controlTimerCallback`。
- [Pixhawk 平台](as2_platform_pixhawk/src/pixhawk_platform.cpp)：`ownSendCommand` 的 TRAJECTORY 分支。
- [公共输出桥](../planner_framework/bridges/as2_setpoint/uav_as2_setpoint_adapter/node.py)。
- [完整接口说明](../planner_framework/bridges/as2_setpoint/README.md)。

其他 checkout 使用前应确认所需主仓库与子模块提交均可取得；历史交接文本不代表当前推送状态。

## CAT-ORA 接入时要保持的约定

1. 输出到 `/<vehicle>/motion_reference/trajectory`，每条消息恰好一个点。
   frame 必须为逐机局部 ENU，例如 `drone0/odom`；position 和 yaw 必须有限。
2. velocity 对应 `TrajectoryPoint.twist`；twist、acceleration 各自允许完整三维有限值
   或全 NaN。CAT-ORA 只提供位置时保留全 NaN，不补成“指定零速度／零加速度”。
3. 公共桥输入是逐机局部 NED FleetSetpoint，再做 `(x,y,z)→(y,x,-z)` 转换。
   核对 fleet 原点扣除的位置，确保只转换一次。
4. `FleetSetpoint.sampled_at` 进入 `header.stamp` 后保持源采样时间。
   所有节点使用同一时钟；重发旧点或给缓存旧点重新盖时间戳不能充当新规划参考。
5. 没有 yaw 要求时，使用接管时的局部航向并保持。公共桥从
   `sensor_measurements/odom` 读取航向；`self_localization/pose` 可能处于 `earth`，
   不能直接当作 odom 定位使用。
6. 输出桥只观察 `controller/info`，已经取消自动申请控制模式。
   任务显式调用 `/<vehicle>/controller/set_control_mode`：

   ```python
   request.control_mode.control_mode = ControlMode.TRAJECTORY  # 消息枚举值 7
   request.control_mode.yaw_mode = ControlMode.YAW_ANGLE        # 消息枚举值 1
   ```

   `available_modes.yaml` 中的 `0b01110000` 是编码掩码，不能作为上述消息枚举填写。
7. AS2 起飞完成后交接轨迹模式；管线准备好再申请，模式建立后尽快提供有效新参考。
   默认源年龄上限 300 ms、未来偏差上限 50 ms、无有效新样本 500 ms 则锁定并请求悬停。
   重复、倒序、非法消息不续期。任务看到轨迹模式丢失应失败或恢复，不能自动反复抢回模式。
8. 到点／取消时显式悬停，再进入降落流程。插件实现悬停时，ControllerInfo 的输入模式
   可能显示 POSITION；正常轨迹执行时则应明确确认 TRAJECTORY。每架飞机保持一个活动参考源。

当前 `deployment/airsim/demos/catora_open_field/mission.py` 已有 `controller_mode()`、
桥 `prepared` 检查、TRAJECTORY 接管和 HOVER 退出；请在这些现有改动上继续。
公共桥的诊断／QoS 改动和 `as2_core/src/core_functions.cpp` 也有未提交修改，
应保留当前迁移工作的内容，避免用旧提交整文件覆盖。

## 部署和验证位置

当前迁移文档采用 `/data/aerohub/as2` 与 `/data/aerohub/lib/catora`，以你的实际部署记录
和 [RoboPi 文档](../deployment/robopi/README.md) 为准。我之前的飞行使用历史私有路径
`/data/local/tmp/aerohub-as2`；其成功不能证明新目录的 install 已包含上述代码。
构建前核对工作区副本及已部署库的来源／哈希，并协调共享 SDK、构建目录及板端消费者。
本次交接没有执行构建、部署或运行切换。

- [低速和失流验证](trajectory_validation/README.md)：PC 20/50/100 Hz 输入测试均为
  1.2 秒输出 60 次；OHOS 构建及单机闭环通过，受控参考源退出、实际 adapter 暂停、
  过期参考和显式取消验证通过。
- [forest8 记录](../demos/super_forest/README.md)：PC SUPER → RoboPi AS2
  → PX4 完成约 235 m A→B，到点误差 0.118 m，速度峰值 8.46 m/s，正常落地解除解锁。
  X 曾越观测边界约 0.44 m，原 observe 策略保留；没有触发备用轨迹切换。

这些记录证明公共执行链可用；CAT-ORA 四机协同的本轮迁移仍应由你的任务验证记录单独验收。


## CAT-ORA 后续收敛（2026-11-06）

CAT-ORA 接入差异现在归其自有 `libraries/cat_ora/ros2_adapter` 包：
`cat_ora_state_adapter_node` 校验并转换局部里程计，`cat_ora_adapter_node` 调原生算法，
`cat_ora_as2_adapter_node` 复用原 PVA 转换并把任务停止意图转为真实 AS2 HOVER 请求。
任务仍直接调用 AS2 起飞、轨迹接管及降落；adapter 不自行申请 TRAJECTORY。
暂停／取消规划获准不等于飞机已经停止，还需确认 AS2 服务响应和模式反馈。

AS2 执行源码、控制频率和轨迹保护未为 CAT-ORA 修改。多机运行通过现有 remapping
将每机 TF 隔离，frame_id 与变换数值不变；未设置 `AS2_TF_SCOPE` 的其他入口仍保留
共享 TF 行为，外部 TF 消费者需绑定相应话题。Swarm 和 SUPER 的原入口未在本轮迁移。

输入、停止、迟到 RPC 与进程组回收说明见
[CAT-ORA adapter](../libraries/cat_ora/ros2_adapter/README.md)。
完整应用还修复并验证了 AirSim/PX4 首心跳竞态的应用侧缓解，以及转发器的双向
TCP_NODELAY；这些属于仿真启动问题，不归因于 CAT-ORA 数值算法。
复现与证据索引见 [四机应用](../demos/catora_open_field/README.md)。
