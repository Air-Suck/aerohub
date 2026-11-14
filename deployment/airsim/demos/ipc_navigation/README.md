# IPC 自由三维森林：SUPER A→B，名义巡航3m/s

真实IPC在PC算力侧完成A*/Floyd、EMVP安全走廊、MPC与控制转换，经既有FollowReference external_acro插件、AS2控制器、Pixhawk平台、PX4 SITL驱动AirSimNH。任务与算力进程当前共置PC容器，分别管理生命周期；高频控制不经任务脚本转发。真值仅用于评分，不进入IPC输入，不使用AirSim运动API。

2026-11-10，`super-3mps-balanced-01/02`在相同二进制、镜像、任务和预设标准下，两次全新启动完整通过。两轮均为`planar_path=false`，未把路径压到固定巡航高度；出生点与地面相对终点对齐SUPER。固定证据见[自由三维验收摘要](validation-free-xyz-20261110.json)，完整过程和失败记录见[IPC validation](../../../../src/tool/navigation/planner/ipc/validation.md)。

| 实测导航指标 | 第一次 | 独立复跑 |
| --- | ---: | ---: |
| 航迹长度 | 237.63m | 238.99m |
| 导航用时，仿真时间 | 82.87s | 84.79s |
| 实际速度中位数 | 2.976m/s | 2.979m/s |
| 全航迹平均速度 | 2.868m/s | 2.818m/s |
| 实际峰值速度 | 3.641m/s | 3.658m/s |
| 真值终点误差 | 0.109m | 0.126m |
| 最小原始点云距离 | 0.968m | 0.890m |
| 最大倾角 | 0.481rad | 0.453rad |
| 新成功求解平均频率，仿真时间 | 99.854Hz | 99.666Hz |
| PX4数值匹配 | 8264/8264 | 8468/8468 |

两轮无新增导航碰撞，取消后的物理停止、原生Land、解除武装及自有资源清理均通过。地面相对高度从约2m变化到6m。名义巡航3m/s包含实际加减速和周期速度波动，不是整段恒速3m/s；完整速度分布在摘要，真实航迹/速度/高度图在各轮`navigation-free-xyz.png`。

## 构建与运行

复用固定镜像`uav/apps-super-forest:repo-native-20261109`和`flight_board:v0.1.0`，不重建共享镜像。依赖获取、锁定版本与构建前置见[native README](../../../../src/tool/navigation/planner/ipc/ros2_adapter/native/README.md)。运行显式加载`log/ipc/control-overlay`和`log/ipc/native-deps`的install overlay；接口/ABI改变时重编译消费者。

需要本机AirSimNH可执行场景与Vulkan GPU。保留渲染、LiDAR及PX4传感器，仅关闭未使用的相机传输。

```bash
python3 deployment/airsim/demos/ipc_navigation/run.py \
  --binary /path/to/AirSimNH --output log/ipc/<fresh-run> \
  --rpc-port 24714 --tcp-port 5725 --relay-port 5825
```

当前默认选择[自由XYZ长任务](super-3mps-balanced-task.json)、[独立验收](super-3mps-balanced-criteria.json)及`--clock-rate-hz 200`。新飞行入口拒绝定高配置；历史`forest-task.json`和旧结果只保留作追溯。`--task`和`--criteria`可一起显式指定其他自由XYZ包线任务，不能用配置名代替实际参数检查。

入口先检查domain182、PX4实例72/system73、Agent8963、端口、容器归属与实际ROS参与者，不停止无关Demo。输出目录必须新建。解锁前检查固件uORB、导航输入、TF和发现；部署就绪后任务经ROS调用原生Takeoff和既有FollowReference。任务完成后自动运行严格数值分析，`acceptance.json`同时要求运动与映射通过；`result.json`单独为true不足以证明全门通过。恢复未确认时保留本轮算力/飞控/场景，见[恢复入口](../ipc_control/README.md)。

## 任务、参数与预设标准

- 出生场景NED为`[42.6,160.11,0]`，yaw默认0。地面稳定10仿真秒后测量任务原点，终点为该原点加`[-37.57,231.76,6]` ENU。目标6m相对地面，不叠加在起飞后高度上；原点和真值换算均留证。
- 起飞2m；全程自由XYZ搜索。任务地面相对边界为`[-60,-10,0.2]…[20,250,10]`。A*/点云/SFC使用一致的可配置earth高度范围，本配置为0…12m；更换地面原点时必须核对该绝对范围。
- 名义实际巡航3m/s，校准参考3.6m/s，参考起步斜坡1m/s²，逐轴速度上限4m/s、加速度3m/s²、jerk10m/s³。保留MPC步长0.1s、N=15；路径密度≤0.05m，参考stride保持对应时间推进尺度。
- 搜索/路径复查/SFC余量分别0.75/0.55/0.55m；原始点云距离≥0.35m且无新增碰撞。高频状态/控制保持原有陈旧拒绝，不因提速放宽。
- 本任务的独立工程验收：估计与真值到点误差≤1.5m、速度≤0.3m/s保持2仿真秒，导航≤300仿真秒/900墙钟秒；实际速度≥2.7m/s累计至少5仿真秒。仍报告全航迹平均、分布和峰值。
- 倾角工程门为0.6rad，最大横向偏差30m，另检查SUPER空间边界。该高速度包线独立于旧20m样例的0.35rad标准；两轮实测倾角均低于0.5rad，不能追认先前超限失败轮通过。
- 取消后在10墙钟秒内确认估计/真值目标误差≤1.5m、速度≤0.2m/s连续1墙钟秒，最大漂移≤0.75m；再原生降落并确认解除武装。NAV_LAND兜底单独记录失败。

## 走廊时序与时效

参考距离安排的下一走廊可能在当前速度/制动状态下无法按时进入。原时序QP不可行时，节点仅在当前状态仍位于初始有限凸安全走廊内，才以该走廊覆盖全部预测步、保留全部动力学硬约束，重新求解一份新QP。仅成功求解可输出；已出走廊、无法在走廊内刹停仍拒绝。它不软化障碍、不执行失败解、不刷新旧解时间。两轮分别记录18/43次成功保守重求解；这是项目的明确实现差异，不冒充上游原样行为。

`failed-qp.json`保存首次primary失败，即使保守重求解成功，该文件也可能存在；不能仅因文件存在判整轮失败。`ipc/health`记录实际求解stamp、状态/点云stamp、参考stride和`conservative_corridor`。发送缓存观察会遗漏两次发送之间被覆盖的新解，不能用发送去重频率替代全部求解频率。

```bash
# 后两项需在固定镜像中加载实际ROS消息overlay后执行；均不启动运动。
python3 deployment/airsim/demos/ipc_control/analyze.py <run>
python3 deployment/airsim/demos/ipc_navigation/audit_timing.py <run> --bag --output <report.json>
python3 deployment/airsim/demos/ipc_navigation/plot_navigation.py <run>
```

近100Hz是记录到的平均成功计算与PX4输入频率，求解间隔仍有抖动；未证明硬10ms期限或固件逐周期100Hz消费。点云p95年龄约0.2s，仍须纳入制动/感知预算。静态历史最多十帧/两秒，不覆盖动态障碍。

## 证据与边界

每轮保存实际源/运行库/补丁/镜像哈希、任务与标准、ROS bag、源/AS2/PX4样本、固件uORB、真值、运动与清理结果。两轮摘要验证二进制、镜像、任务、标准一致，原失败不覆盖。最初0.5m/s实际巡航不足、约25m真实QP失败、起步/转弯倾角超限和评分队列漏帧均在validation单独记录。

自由XYZ森林完成不等于每个障碍都必须通过升降越过；上绕/下绕墙另有几何回归，尚无该指定墙障碍的物理闭环验收。当前未覆盖RoboPi、真机、多机、突发/动态障碍、扰动或通用Mission/Tool运行时。

历史0.25m/s、20m、定高`forest-13/14`的原标准与结果见[旧摘要](validation-20261110.json)。这些结果属于当时产物，不替代本轮自由XYZ验收；后续飞行不再使用其定高配置。
