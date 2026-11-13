# EGO-Planner-v2 原生 Humble 移植

源码来源为 ZJU-FAST-Lab/EGO-Planner-v2，提交 `9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`，加上原 AirSim 复现中保存的四文件补丁。上游采用 GPLv3，保留 LICENSE、源码署名与原维护者信息。上游部分 package.xml 的许可证为 TODO，本移植依据仓库 LICENSE 标记 GPL-3.0。

本目录包含六个必要运行包。`plan_manage` 的 ROS 包名为 `ego_planner`。`quadrotor_msgs` 当前仅提供规划器实际消费的 GoalSet。规划器保留本版地图、优化器、PolyTraj 与 MINCOTraj，现有 Swarm-Formation 仅用于参考 ROS 2 API。

## 构建

在已有 ROS 2 Humble 的 Ubuntu 环境中，从迁移工作区根目录运行。

```bash
bash tools/build_humble.sh
source install/humble/setup.bash
ros2 launch ego_planner ego.launch.py
```

当前本机使用 WSL 的 Ubuntu 发行版及 `/opt/ros/humble`，外部 Ubuntu-20.04 的 ROS 1 环境保持独立。构建产物位于工作区 build/humble 与 install/humble，日志位于 log/ego_planner_v2。编译并发限定为 2。

## 接口契约

| 接口 | 约定 |
| --- | --- |
| odom_topic | Odometry。规划器消费共享 world/ENU 中的位置及世界系线速度，输入适配负责转换 body twist。深度路径另外保留原机体姿态语义。 |
| cloud_topic | PointCloud2。XYZ 必须已注册到同一共享 world/ENU，保留测量时间。 |
| goal_topic | quadrotor_msgs/msg/GoalSet。drone_id 与 manager.drone_id 一致，goal 为共享 ENU 米制坐标。发布端使用 reliable、transient-local QoS。 |
| ~/planning/trajectory | traj_utils/msg/PolyTraj。五阶多项式，每段六个系数为最高次幂到常数项，duration 与系数保留 float32。 |
| /ego_swarm/trajectory | MINCOTraj。保留车辆 ID、轨迹 ID、ROS 开始时间及原机间有效性检查。 |
| ~/mandatory_stop | std_msgs/msg/Empty，进入原 emergency-stop 状态机。 |
| ~/planning/heartbeat | std_msgs/msg/Empty，规划状态机定时发布。 |

默认命名空间为 `/drone0`，节点名为 `ego_planner`，因此轨迹话题为 `/drone0/ego_planner/planning/trajectory`。使用 launch 参数指定每机输入话题和 ID。当前 launch 默认输入名字与 AeroHub 接近，但其 world/ENU、速度及原点语义仍须由后续导航输入接入验证。

参数路径由 ROS 1 的 `/` 转为 ROS 2 的 `.`，所有算法模块共享节点 ROS clock。`use_sim_time` 是显式 launch 参数。depth/pose 和 depth/odom 的 message_filters 路径保留，传感器订阅采用 sensor-data QoS。

配置取自旧单机 integration/airsim_px4.launch 与 advanced_param.xml，速度 0.23、加速度 0.35。它是迁移调试配置。双机森林历史测试的膨胀、间距与衰减参数需要在对应场景接入阶段显式核对。代码中 require_previous_agent 默认值保持 true，示例 YAML 显式设置 false 以支持独立启动。

## 执行边界

原生规划包以 PolyTraj 输出为边界。旧 traj_server、PositionCommand、MAVLink 执行桥、仿真器与旧 ROS 1 运行脚本保留在参考源码及备份中。相邻 `../bridge` 包负责完整时间化 PVA 到 TimedTrajectorySegment、FollowTimedPath 的接入，接口阶段结果见 `skill/temp/ego_planner_v2/STEP03_AS2.md`；AS2 控制/平台与 PX4 动力学另行验收。

迁移副本已增加全区间速度/加速度硬门，并检查 PolyTraj float32 量化后的多项式；超限候选有界重新优化，失败时不发布。固定物理门槛及完整规划器回归见 `skill/temp/ego_planner_v2/STEP02_CONSTRAINTS.md`。`fsm.odom_twist_in_body_frame` 默认 false 保持原世界系速度契约；AS2 接入显式设为 true，保留标准 Odometry 的 body twist，由 FSM 旋转到世界系。

预设航点在 executor 收到里程计后开始处理。滚动目标更新由同一串行 executor 处理，取消了原 ROS 1 在回调内部 spinOnce 等待状态机的方式，避免单线程阻塞。此为 ROS 调度适配，目标状态行为由回归测试检查。

## 验证入口

在工作区加载 Humble 与 install overlay 后，以独立 ROS_DOMAIN_ID 运行。

```bash
export ROS_DOMAIN_ID=173
python3 src/planner/ego_planner_v2/ros2_adapter/tests/verify_planner.py \
  --output log/ego_planner_v2/planner-run-01
```

本机普通 WSL 网络下出现参与者发现和数据传输不完整。经过临时 Linux network namespace 隔离，仅保留回环网卡后，双节点回归通过。该方式需要 Linux 创建网络命名空间的权限，可从工作区根目录运行。

```bash
sudo bash tools/verify_humble.sh --isolated-network log/ego_planner_v2/new-run
```

此入口运行结束后释放临时网络命名空间，结果写入指定的新目录。已通过的记录位于 `log/ego_planner_v2/planner-run-08/result.json`。两机各产生 18 条 PolyTraj 和 18 条 MINCOTraj，覆盖错误车辆目标拒绝、执行中目标更新、持续重规划、到达分支和原生停止轨迹。普通 WSL 网络与跨机通信仍待验证。

脚本启动真实规划器，发布合成里程计和点云，按原生多项式采样回放反馈。输出目录必须尚不存在。结果包括轨迹格式、滚动输出、车辆目标归属、MINCO 广播、目标到达与持续运行检查。此输入回放不包含 PX4 动力学，实际 AirSim 数据接入与飞行验收在后续阶段进行。

`tests/minco_regression.cpp` 位于 native 的同级 tests 目录，可分别链接参考与移植的头文件，比较四种分段数量下各 51 个采样点的 PVA。完整数值优化器的 ROS 1/ROS 2 多场景对照与真实平台验收需按实际证据分别记录。
