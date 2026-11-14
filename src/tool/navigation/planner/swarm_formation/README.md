# Swarm-Formation 原生 ROS 2 Humble

源仓库 https://github.com/ZJU-FAST-Lab/Swarm-Formation ，固定提交
`967a4bdfae949e994691f8ffc87dbb0147cbebb7`，原检出无本地修改。
`upstream/` 子模块保留上游源码、README 和 LICENSE；`ros2_adapter/native/` 是 ROS 2 移植，构建时显式选择，避免与上游同名包混合。
上游包清单的许可证填 `TODO`，README 和 LICENSE 明确 GPLv3；移植清单按
GPL-3.0 标记，保留原作者与许可证。

## 运行边界

迁移六包：`swarm_graph`、`traj_utils`、`plan_env`、`path_searching`、
`traj_opt` 和 `ego_planner`（源码目录 `plan_manage`）。保留原 MINCO、
L-BFGS、编队图、碰撞代价与 A* 算法计算；本轮不调整权重、速度/加速度
参数或终止条件。后续真实点云接入另修正了 ESDF 中 `clear()` 后继续按下标访问
临时数组的内存错误，保留距离场计算公式；见下述真实输入验收。

不包含旧 PX4/Gazebo 模拟器、`PositionCommand` 控制服务器、随机地图生成器
和 UDP `swarm_bridge`。当前 adapter 直接采样 `PolyTraj`。默认七机 launch 保留本机代理转发；
三机混合应用显式让各机原生规划器直接通过 DDS UDP 广播轨迹，见 `deployment/airsim/demos/swarm_open_field/`。

真实模式每台输入独立注册点云与完整里程计；原回归模式保留
`/drone_<i>_visual_slam/odom`、`/map_generator/global_cloud`。输出
`/drone_<i>_planning/trajectory`（`traj_utils/msg/PolyTraj`）。所有位置使用共享
ENU/world。`start_time` 改为 `builtin_interfaces/Time`，系数顺序、float32
精度及每段 duration 保留。算法进程使用同一节点的 ROS clock。

`plan_manage/config/humble.yaml` 从原 `normal_hexagon.yaml` 和
`advanced_param.xml` 的实际参数生成，参数路径 `/` 映射为 `.`；默认速度
1.5、加速度 8.0、规划范围 7.5。RViz marker 仍作为可选输出保留。

TCP `127.0.0.1:28501` 保留原 adapter 的逐行 JSON 契约、`drone0..drone6`
默认顺序与多项式采样公式；`vehicle_count` 可显式选择连续前缀的 3..7 机。请求目标发布到 `/traj_start_trigger`，launch 将编队
手动模式实际订阅的 `/move_base_simple/goal` 映射到此路径；原代理与该模式
之间的路径差异作为接线变更处理。一次性目标采用 durable QoS，默认七机等待全部七个
订阅者发现后发送。代理串行处理状态更新，ROS executor 在独立线程接收轨迹。

上游编队目标 callback 将中心高度写死为 **0.5 m**。旧模式默认保留；
真实导航模式显式打开 `global_goal.altitude_from_message`，采用目标消息高度，
与公共目标和 AS2 执行坐标一致。
同样，原配置的 `global_goal.swarm_scale=2`，当前公共 adapter 的目标形状
校验默认按单位 scale 以兼容旧回归。实际应用须显式匹配公共 `formation_scale`
与原生 `global_goal.swarm_scale`；本次三机两边均为 2。

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/tool/navigation/planner/swarm_formation/ros2_adapter/native \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch ego_planner seven_planners.launch.py input_mode:=navigation \
  fleet_config:=/absolute/path/to/calibrated-navigation-fleet.yaml
```

或在仓库根目录：

```bash
export SWARM_FLEET_CONFIG=/absolute/path/to/calibrated-navigation-fleet.yaml
docker compose -f deployment/swarm_formation_humble/compose.yaml up --build -d
```

镜像 `uav/aerostack2-swarm-formation:humble` 不需要 ROS 1、roscore 或旧代理。
Compose 默认 ROS_DOMAIN_ID=81，可用同名环境变量覆盖；地图发布者需在相同
domain。地图数据来自外部，Compose 不生成假地图。旧 `four_planner.launch`
实际含七台节点，主线 ROS 1 部署保留在仓库 `deployment/swarm_formation_ros1/`。

## 真实注册点云与里程计

```text
AirSim LiDAR + PX4/AS2 定位和 TF
  → as2_navigation_inputs（按采样时间注册至各机 odom）
  → navigation_bridge_node（固定原点/航向，统一至共享 ENU）
  → 各机 Swarm GridMap / ESDF 和规划状态机
```

| 接口 | 约定 |
| --- | --- |
| 输入 `/droneN/navigation/points` | PointCloud2，XYZ 已在 `droneN/odom`，best-effort；禁止将传感器局部点云直接重命名接入 |
| 输入 `/droneN/navigation/odom` | Odometry，位姿在 odom，速度/速度协方差在 child/body frame |
| 输出 `/swarm_formation/droneN/points` | 共享 `map_frame` 的注册点云，保留采样时间、布局及附加字段 |
| 输出 `/swarm_formation/droneN/odom` | 位姿/位姿协方差转到共享系，body twist 及其协方差保留 |
| 算法 `~/grid_map/input_healthy` | 同时要求里程计与点云新鲜，缺失或过期为 false |

`navigation_fleet.example.yaml` 给出七机配置格式，但其中原点来自旧部署示例，
**不是实测原点**。运行前填写每个导航局部系到共享 ENU 的固定 `origin_enu`
及 `origin_yaw`；不能把同一台飞机的数据复制成七台，也不能将局部坐标直接当
作共享世界坐标。需要先由 AS2/runtime 分别启动导航输入节点；本库不重启数据源。

真实模式开启 `fsm.twist_in_body_frame`，算法将 body 速度按消息姿态旋转到
共享系后参与规划。JSON 代理不再发布位置拼造的零速度 Odometry，而使用导航桥
的实际数据。模式默认使用仿真时间，可显式设置 `use_sim_time:=false`。

桥拒绝错误 frame、非有限状态或非法姿态、过期/倒序/零时间或非法点云布局；时间回退
清空顺序水位并要求新里程计。它只转换已经注册的云，不用最新 TF 替代采样时刻
TF，也不提供逐点扫描去畸变；源时间和运动注册质量仍由导航输入层负责。
仿真重置或世界变化后应重启规划器以清空旧地图；输入桥恢复不表示旧场景地图已重置。

真实模式 `grid_map.input_timeout` 默认 1 秒：无新鲜地图不开始规划，执行中丢失
输入进入原生 emergency-stop 分支；不擅自自动恢复飞行任务。地图可保留旧占据
内容用于观察，但输出保留原始输入时间。代理要求配置中全部地图健康且 heartbeat 未过期，
否则停止返回缓存滚动轨迹，上层公共 Planner 的轨迹超时机制继续生效。

Docker 默认走真实导航模式并要求挂载校准后的七机配置。launch 保留 `legacy`
兼容默认值用于旧回归，实际运行需显式选择 `input_mode:=navigation`。
DDS domain、可达网卡/profile 必须与数据源一致；
使用限定网卡的部署时设置 `SWARM_DDS_PROFILE=/absolute/path/to/profile.xml`。

### 真实 AirSim 点云验收（2026-11-05）

```bash
# 创建独立 PC AirSim/PX4/AS2 静态实例；不解锁、不起飞，结束后清理自身资源。
python3 src/tool/navigation/planner/swarm_formation/ros2_adapter/tests/verify_airsim_cloud.py \
  --output log/airsim/swarm-cloud-next
# 已有健康数据源时，可指定 ROS_DOMAIN_ID / SWARM_DDS_PROFILE 只读验证：
bash src/tool/navigation/planner/swarm_formation/ros2_adapter/tests/verify_real_cloud.sh log/airsim/swarm-cloud-existing
```

独立 NH 实例使用 RPC 42472、PX4 TCP 4684、控制 UDP 14844/14984、实例 44、
系统 ID 45、Agent 8905、ROS domain 92；不改变已有仿真或板端进程。
版本化摘要见 `validation/real-point-cloud.json`，完整本机记录在
`log/airsim/swarm-airsim-cloud-final/result.json`：

- 接收到 21 帧真实原始 LiDAR、16 帧共享系注册点云；非零平移/旋转后的最大
  点坐标误差 `1.43e-6 m`，字段、原始时间和 body twist 保留。
- 使用真实 `GridMap` 检查，841 个有效采样点全部匹配膨胀占据单元，全部在
  ESDF 表面附近；禁用测试中的虚拟天花板，结果不靠天花板制造占据点。
- 发布 37 帧非空膨胀地图，最后一帧 5808 点；暂停本测试的桥后健康状态变为
  false，恢复新数据后回到 true。未暂停真实传感器、AS2 或仿真。
- 独立接口测试覆盖旋转/平移、协方差、intensity 保留、错误时间/帧、状态过期、
  时钟回退，以及代理健康 heartbeat 失效后拒绝缓存轨迹与恢复。
- 840 次固定输入优化回归全部成功，仍与 Noetic 基线保持最大位置差
  `4.68e-8 m`、时长差 `3.24e-8 s`；此轮不重新作性能结论。

这证明单个真实传感器到原生地图的链路；尚不代表七机真实点云同时建图、运动中
注册稳定性或完整七机避障飞行验收。原有高度/编队目标契约差异也仍需后续处理。

## 可重复验证

需要本机 Docker 与已有 `uav/catora:noetic-upstream`、
`osrf/ros:humble-desktop-full-jammy` 镜像，仓库根目录执行：

```bash
bash src/tool/navigation/planner/swarm_formation/ros2_adapter/tests/verify.sh
```

脚本先构建两版，再顺序运行基准；结果默认位于
`/tmp/aerohub-swarm-validation`。Noetic 镜像只提供依赖，算法来自固定源码
快照；基线构建仅排除当前不使用的 `traj_server` target 及其消息依赖，并
添加对照程序，不修改算法。

同一份 C++ 测试程序分别链接原始和移植后的优化器。四组固定起终状态、七台
车辆、编队优化开/关，共 56 个组合；每组合预热 3 次、测量 15 次，合计
每版本 840 次。算法 ROS 时间固定为 100 秒，其他车辆参考轨迹相同；实际
耗时以 `steady_clock` 测量。逐次检查成功状态、总时长及 51 个归一化采样点，
位置与时长差异阈值均为 `1e-6`，不是检查随机 live 输出完全一致。

2026-11-05 干净目录复现结果：两版各 840 次均成功；最大位置差
`4.6797e-8 m`，最大轨迹时长差 `3.2399e-8 s`。

| 优化模式 | ROS 1 中位数 / P95 | Humble 中位数 / P95 |
| --- | --- | --- |
| 关闭编队优化 | 0.110 / 0.157 ms | 0.061 / 0.092 ms |
| 开启编队优化 | 0.583 / 0.776 ms | 0.511 / 0.701 ms |

性能数据包含 Ubuntu 20.04/GCC 9/Eigen 3.3 与 Ubuntu 22.04/GCC 11/Eigen
3.4 的环境差异。两版为 Release；上游自定义 `-O3 -Wall -g`，Humble 默认
Release 使用 `-O3 -DNDEBUG`。因此数据是两个可运行构建的观测，不能将速度
差异全部归因于 ROS 版本。

运行回放启动七个真实规划器，输入地图和合成里程计，将近未来采样位置回放为
后续里程计，验证多次滚动输出、七个完成回调、七条 JSON 采样轨迹、三类
无效请求拒绝及恢复。它验证 ROS 2 运行接线，不是动力学仿真或飞行成功率。
该次七台原生轨迹消息数量分别为 `[9,12,8,6,7,10,13]`，每台 JSON 返回
11 个采样点。首次验证曾在完成分支发现默认零时间的时钟类型不一致；移植
明确保留 ROS clock 零值后，七个完成回调全部通过。
验证还编译现有公共消息与 `uav_swarm_formation_adapter`，确认未经修改的
adapter 能从真实原生后端读取结果并发布七条 `TimedTrajectorySegment`
车辆轨迹；原有五项坐标、中心和连续性检查也通过。此检查不证明逐车目标
最终一致性，相关形状/高度差异见前述运行边界。

固定输入基准使用无障碍地图；运行回放仅放置远离通道的一个障碍点。
复杂障碍、拥挤编队及 ROS 1/ROS 2 完整分布式调度对照尚未验收。后续三机
真实 AirSim/AS2 混合飞行已通过，见文末应用验收；不扩展为七机验收。

## RoboPi 持久部署与简单验证

复用已准备的 OHOS 工作区和官方 build Docker：

```bash
bash tools/robopi/build_lib.sh swarm-formation
bash tools/robopi/deploy_lib.sh --component swarm-formation
bash tools/robopi/verify_lib.sh swarm-formation
```

板端安装于 `/data/aerohub/lib/swarm-formation/install`，环境入口为同目录上一层
`env.sh`，先加载 `/data/aerohub/as2/board_env.sh`。应用包含原生六包、算法 adapter
及公共 Planner 接口/框架、AS2 输出桥，AS2 本身保持独立。

简单验证只启动一个原生规划节点，以静态里程计、单点合成点云和手动目标检查
首条有限五阶 PolyTraj；不代表真实点云、七机编队、JSON 七机输出或飞行验收。
日志与结果保存在板端 `runtime/verification`，并取回本机
`log/airsim/robopi-lib-acceptance/swarm-formation/`。

OHOS 编译使用标准 Eigen 头路径，并仅链接实际需要的 PCL common；
`pcl_conversions` 的头文件与消息依赖单独声明，避免导出全部 PCL 库而拉入未使用的 IO/PNG。
首次板端规划还暴露上游 `desired_start_time` 在无旧轨迹分支未初始化；Clang 优化后
错误读取空轨迹。将该首轮不使用的占位值初始化为零，保留首次规划由当前时钟
设置实际起始时间的行为及后续轨迹衔接算法。

2026-11-05：上述持久目录中的板端简单验证通过，版本化摘要为 `validation/robopi-smoke.json`。程序安装保留，验证进程已停止。

## 三机 PC/RoboPi 真实应用验收（2026-11-06）

[应用与复现](../../../../../deployment/airsim/demos/swarm_open_field/README.md)，
[版本化验收](../../../../../log/pr-history/deployment/airsim/demos/swarm_open_field/hybrid-validation.json)。
PC 两个独立容器与 RoboPi 各一台，各自运行本机原生 Swarm、GridMap/ESDF、导航注册、
adapter 和 AS2，三路真实 AirSim LiDAR，跨机原生轨迹通过 DDS UDP 直接广播。

最终 `log/airsim/swarm-hybrid-flight-04` 完成 SUCCEEDED、全部降落并解除武装；
最小真值间距 2.746 m，最大真值终点误差 0.524 m，无新碰撞。
111 条公共 PVA 轨迹段和三机实际 AS2 参考均有效，实际执行期间三路输入与地图采样健康率均为 100%。
板端总 CPU 中位 93.5%、最高 96.4%，含全局代理、公共采样与验收开销；不能当作纯单机算法负载。
仍有一条原生广播因为 269 ms 起始时间差被拒收，不能宣称通信零问题或长期稳定。

关键接线修复：三机前缀与本机 ownership、真实模式消息高度与显式编队尺度、
原生 PVA 解析导数与同一 valid_from、Swarm 自身的 AS2 停止 adapter、
缺失的导航输入交叉编译和嵌套输出桥构建。默认七机优化公式与权重保持不变。

保留失败证据：[默认间距验收](../../../../../log/pr-history/deployment/airsim/demos/swarm_open_field/default-clearance-validation.json)。
原生默认 swarm_clearance=0.5 与应用最小 2 m 不一致；应用显式配置 2.0 后复验通过，
没有降低验收阈值。另修正外层 95 s 墙钟保护早于 90 s 仿真任务截止的问题；
本场景实时因子约 0.81，墙钟保护改为 180 s，任务仍为 90 s 仿真时间。
