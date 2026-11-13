# FollowReference 角速度/推力的 AirSim + flight_board 验证

本入口验证已有PX4飞控是否实际消费新FollowReference插件输出，不运行IPC算法。控制源是明确标注的验证用位置/姿态反馈器；不能将本结果写成IPC导航、动态避障或抗扰验收。

## 已验证范围

2026-11-10的 `airsim-control-13` 通过：原生起飞后，插件接管机体角速度/推力，在约2m高度保持5秒墙钟时间并产生yaw响应；主动停止控制源后，Behavior退出并请求保持，确认低速后原生降落、解除解锁。本次真实AirSim/PX4/AS2闭环使用GPU Vulkan渲染，没有使用AirSim运动API。

| 指标 | 预设判据 | 实测 |
| --- | --- | --- |
| AirSim真值保持漂移 | ≤0.75m | 0.1301m |
| 姿态倾角 | ≤0.35rad | 0.0680rad |
| AirSim NED yaw变化 | -0.4～-0.03rad | -0.1104rad |
| 断流后停止速度 | ≤0.5m/s | 0.0842m/s |
| 碰撞与恢复 | 无新增碰撞，降落且解除解锁 | 通过 |

202组源控制帧和AS2 actuator推力具有相同源时间戳，最大数值差 `2.38e-7 N`；198个PX4设定值样本的角速度FLU→FRD转换和推力归一化均在 `1e-5` 容差内匹配。PX4样本采用±0.5s窗口内数值匹配，允许重复设定值，不能声称唯一序号对应。

固件内部uORB另确认Offboard和角速度环启用、位置/速度/姿态环关闭；`vehicle_rates_setpoint` 采到yaw=-0.5rad/s和推力z约-0.49，AirSim真值确认实际旋转。源/AS2/PX4输入观测约48Hz仿真时间、36Hz墙钟，当前clock桥默认50Hz，**不是100Hz闭环验收**。本次未覆盖真机、RoboPi、长时可靠性、完整IPC和全部故障组合。

## 角色与运行

- AirSim：独立场景及真值评分来源，RPC只读评分，不控制运动。
- flight_board：固定镜像内PX4、Agent、传感器及clock；instance72、system73、独立端口。
- 算力角色：`compute_runtime.py` 提供AS2、FollowReference插件和验证控制源。
- 工作站角色：`mission.py` 仅调用ROS能力及独立评分，不启动算法/AS2，不转发高频控制。`session.py` 是验证部署进程的所有者；本轮任务和算力角色共置于PC临时容器，不代表RoboPi部署通过。

先构建独立overlay，基础镜像不会被修改：

```bash
bash deployment/airsim/demos/ipc_control/build_overlay.sh
python3 deployment/airsim/demos/ipc_control/run.py --output log/ipc/control-fresh-run
python3 deployment/airsim/demos/ipc_control/analyze.py log/ipc/control-fresh-run
```

AirSim资源由现有本地映射提供；也可用 `--binary` 指定可执行文件。端口占用时拒绝启动；`--rpc-port`、`--tcp-port`、`--relay-port` 可选择新的独立端口，避免服务冲突或旧TCP TIME_WAIT。ROS domain182启动前执行图发现探针，发现其他节点则拒绝；不停止其他任务。

默认依赖 `flight_board:v0.1.0` 与 `uav/apps-super-forest:repo-native-20261109`。运行前解析不可变image ID，构建source-provenance与运行images/manifest分别记录；固定镜像不可获取时，不声称从空机可复现。overlay默认在 `log/ipc/control-overlay/`，可通过 `--workspace` 改用持久构建工作区。算法/插件源码保存在仓库，外部目录仅保存构建副本与产物。

## 必要修复与时序

`deployment/platform/patches/as2-body-rates-bypass.patch` 修复ControllerHandler透传未复制推力的问题；BODY_RATES输出要求两种分量同时到达、同源时间戳、正确机体系且新鲜/有限。补丁适用于记录的AS2 f387e0d版本（项目AS2 PR补丁应用后），不改ABI。构建脚本检查基线/补丁后源码SHA256，遇到其他本地改动保留并拒绝覆盖。该修复仅通过显式overlay启用，未替换共享镜像。

接口牛顿值按平台max_thrust=15归一化。测试从起飞后的PX4 `vehicle_attitude_setpoint` 读取hover比例供验证源使用；这是映射基准，不是实机牛顿推力标定。

当前AS2控制器和平台均在未解锁时抑制actuator输出，地面探针只验证模式与输出抑制，不假造飞行状态。Offboard初始化会重置PX4模式，启动时保持探针的BODY_RATES，等待真实 `VehicleControlMode` armed/Offboard反馈后再由Takeoff首次切换位置模式，避免仅依据服务成功或缓存字段推进。

固定镜像Python `VehicleStatus` 为v1，而固件输出v4；未版本化FailsafeFlags也发现布局差异。任务不将旧类型改话题名冒充兼容，而从目标固件uORB独立读取完整起飞前检查。实际控制使用的VehicleRatesSetpoint v0在镜像与固件提交中逐字节一致，SHA256为 `ffd2818487ed90b95908a20b967a6ec4357a922aa59ca00919b01638f06a6c58`。

发送Goal前要求验证控制源有新鲜定位/速度输入且DDS发布订阅就绪。首次模式切换后丢弃旧帧并等新帧。取消服务接受不等于物理停止，本入口实际检查恢复模式与速度，之后才降落。

## 证据与失败保留

测试准备不足与重复运行的具体复盘，由 IPC 自己维护在 [validation 文档](../../../../src/planner/ipc/validation.md)。

版本化摘要见 `validation-20261110.json`。原始证据在 `log/ipc/airsim-control-13/`：result/analysis、输入与各级指令、真值、PX4 uORB、模式、源码与image清单、各进程日志以及清理结果。全部算力子进程均正常退出，无强制kill；本次容器、场景和中继已回收，原有SUPER资源保持运行。

早期轮次记录了域冲突、测试客户端类型/记录错误、Offboard缓存竞争和控制源未就绪等失败；`airsim-control-10` 是初次控制成功，13是完整入口与固件内部证据通过。失败轮没有提升阈值来获得成功。

恢复未确认时入口保留算力、时钟与仿真。已经由真实PX4确认未解锁时，可运行以下命令回收该轮拥有的资源；仍处于解锁状态时脚本拒绝回收，须先完成控制恢复/降落：

```bash
python3 deployment/airsim/demos/ipc_control/recover_ground.py --output log/ipc/control-fresh-run
```

后续真实IPC验证又补齐了两个同属配对协议的缺口：`as2-body-rates-source-stamp.patch` 保留控制器BODY_RATES透传输出的源时间戳，`pixhawk-body-rates-pairing.patch` 在平台最终发送处拒绝不完整/错序、超龄和非有限组合。`build_overlay.sh` 固定顺序检查并应用三个补丁，补编译控制器、Pixhawk平台和插件，运行实际平台类的隔离配对回归；不重建共享镜像。基础补丁清单允许已知后续补丁后的源码哈希，未知改动仍拒绝覆盖。最早的control验证证据对应当时版本，不能替代这些后续补丁的实际验收；新结果由IPC navigation validation记录。
