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
# 有本地 onboard_computer:airsim-px4 镜像时，另跑四机 Planner 接线：
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

以上为服务与接线验证。后续 [CAT-ORA 空旷场地应用](../../../demos/catora_open_field/README.md)
已在 PC AirSimNH 完成四机起飞、规划、到点和降落，并通过独立实际位置验收；
适用范围以 `libraries.yaml` 的 `scenario_pass` 说明为准，不代表 RoboPi 多机已通过。

## RoboPi 原生服务验证

2026-11-05 使用现有官方 build Docker 与隔离 SDK，当前 Humble 包交叉编译通过。
ELF 为 AArch64，生成的 Python 类型支持 ABI 为 `cpython-312-aarch64-linux-ohos`。
通过 HDC 将单个包安装到 `/data/local/tmp/aerohub-as2/install/mrs_formation_reshaping`，
未替换系统 ROS 或其他 AS2 包。

RoboPi 原生运行 C++ 服务及 Python JSON 代理，独立域 `179`、仅本机发现。
五组用例两类接口均通过，轨迹逐点与 ROS1 基线距离小于 `1e-9 m`；
七类非法请求拒绝及后续恢复通过。测试结束后停止本次两个服务进程。
本次不包含跨机 adapter、AS2 轨迹执行、多机飞行或持续负载性能验收。

摘要：[robopi-service.json](../validation/robopi-service.json)。原始构建、ELF、
校验和、板端日志保存在本机 `artifacts/catora-robopi/`。
复验使用 `tests/verify_board.py`，同目录放 `verify_service.py`、`cases.json` 和
由 `tests/capture_ros1.py` 取得的 `baseline.json`；板端在同一 shell 加载
`/data/local/tmp/aerohub-as2/board_env.sh`，设置 `ROS_DOMAIN_ID=179`
及 `ROS_LOCALHOST_ONLY=1`，然后执行 `python3 verify_board.py`。
构建复用 `deployment/oh-ros-build/run.sh build mrs_formation_reshaping`，
输入为隔离工作区 `ws/src/mrs_formation_reshaping` 中的当前 `humble/` 源码副本。

## 当前持久部署入口

AS2 独立安装于 `/data/aerohub/as2`；CAT-ORA 应用安装于
`/data/aerohub/lib/catora/install`，加载 `/data/aerohub/lib/catora/env.sh`。
前述 tmp 路径属于历史验证位置。

```bash
bash scripts/build_lib.sh catora
bash scripts/deploy_lib.sh --component catora
bash scripts/verify_lib.sh catora
```

此简单验证只调用一次原生轨迹服务并核对本地 JSON 响应，结束后停止测试进程。
AS2 可通过 `bash scripts/verify_lib.sh as2` 单独做消息加载和估计器最小启动检查，
不启动 PX4/AirSim 或发送运动指令。

2026-11-05：上述持久目录中的板端简单验证通过，版本化摘要为 `../validation/robopi-smoke.json`。程序安装保留，验证进程已停止。
