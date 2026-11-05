# CAT-ORA 原生 Humble 轨迹服务

上游：https://github.com/ctu-mrs/catora ，提交
`4bc41f593eebde65dd74108a97e80430a145dfa0`。
源码从现有 `uav/catora:noetic-upstream` 镜像提取，原始快照留在
`../../../thirdparty/cat_ora_ros1`，以 `COLCON_IGNORE` 排除。核心 `catora.hpp` 原样保留；
`formation_reshaper.cpp` 只替换日志，头文件移除未使用的 ROS 1 / MRS 类型。
作者与版权保留上游约定。上游 `package.xml` 写 MIT，README 写 GPL-3.0；
本移植沿用 README 声明的 GPL-3.0，未将算法重新授权。

本次迁移当前调用方实际使用的轨迹生成服务，不包含完整上游 nodelet 演示、
RViz、离线文件生成、assignment 或 reshape 服务。
`publish_visualization=true` 明确返回失败。

## 构建与启动

在有 ROS 2 Humble、Eigen3、colcon 的环境：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths libraries/cat_ora/humble
source install/setup.bash
ros2 run mrs_formation_reshaping cat_ora_node
# 另一终端 source 同一环境，为现有 adapter 保留 TCP 28500 协议：
ros2 run mrs_formation_reshaping ros2_proxy.py
```

或在仓库根目录：

```bash
docker compose -f libraries/cat_ora/deployment/compose.yaml up --build -d
```

服务 `/mrs_formation_reshaping/get_reshaping_trajectories` 使用
`mrs_formation_reshaping/srv/SrvGetReshapingTrajectories`。
输入输出使用机群共享 ENU；轨迹数组按输入车辆顺序排列。
算法可以重分配目标，原 adapter 仍负责拒绝违反逐车目标归属的结果。
采样步长来自请求 `trajectory_dt`；无显式 timestamp。

上游 callback 将 `max_velocity` 与 `max_acceleration` 传反；本版本按核心
`setConstraintsAndDt(acc, vel, dt)` 定义修正。两者不等时，输出采样点数量及
轨迹会与旧服务不同。JSON 请求/响应协议保持兼容，代理没有 ROS 1 运行依赖。

## 真实对照验证

```bash
bash libraries/cat_ora/humble/tests/verify.sh
# 有本地 companion_as2:airsim-px4 镜像时，另跑四机 Planner 接线：
bash libraries/cat_ora/humble/tests/verify_adapter.sh
```

此脚本在隔离容器中运行真实 ROS 1 服务采集基线，再编译运行 Humble 服务：

- 单机、四机、重分配、静止轨迹逐点对照（容差 `1e-9`）。
- 不等速度/加速度：另采集交换旧服务参数后的对照结果，并检查运动约束。
- 原生 ROS 2 服务与 TCP/JSON 返回一致。
- 空数组、数量不等、零步长、NaN 速度、负加速度、无穷坐标、可视化请求均拒绝；后续合法请求仍成功。

四机脚本在已有 AS2 镜像中编译本库、公共接口、Planner、算法 adapter 和
AS2 输出 adapter，验证四辆车的 `/planner/trajectory` 和 `/planner/setpoints`。
保留的 `smoke_real_ros1.sh` 是历史文件名，现在同样可连接 ROS 2 后端；
算法 adapter 由框架启动，测试不再重复拉起第二个同名节点。

以上为服务与接线验证。后续 [CAT-ORA 空旷场地应用](../../../apps/catora_open_field/README.md)
已在 PC AirSimNH 完成四机起飞、规划、到点和降落，并通过独立实际位置验收；
适用范围以 `libraries.yaml` 的 `scenario_pass` 说明为准，不代表 RoboPi 多机已通过。
