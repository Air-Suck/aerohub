# 交接文档

对话 hash：`207aaf4d-9c28-42ac-8b20-674f43504c63`（续自 `4e3e4eb0-4c24-455a-8f44-9b8ed461a32d`）

归档标题：**Swarm-Formation 与 CAT-ORA 的 PX4-Gazebo 动力学复现及 Aerostack2 集成评估**

## 归档范围

> 历史归档：本文记录 2026-09-30 时的四机 CAT-ORA 阶段和 Aerostack2 可行性评估。当前 CAT-ORA 独立复现已扩展为 16 机方阵到圆环，Aerostack2 集成也已落地。现行操作以 `CAT-ORA与Swarm-Formation的PX4-Gazebo复现说明.md` 和 `Aerostack2统一Planner使用指南.md` 为准。

本次归档覆盖两个相互隔离的完整复现：① Swarm-Formation 原版及 Gazebo Harmonic + PX4 SITL 单机/七机动力学闭环；② CAT-ORA 原版及四机 PX4/Gazebo 动力学闭环。ROS 均只在 Docker 内运行，部署资产、环境名、命令、问题和验收证据已固化。另完成 Aerostack2 统一管理两套 ROS1 算法的可行性评估与迁移顺序设计。

CAT-ORA 上游已固定 commit `4bc41f593eebde65dd74108a97e80430a145dfa0`（master）。官方环境为 Ubuntu 20.04 + ROS Noetic，包名 `mrs_formation_reshaping`，依赖 `mrs_lib`；主输出接口 `/mrs_formation_reshaping/get_reshaping_trajectories` 返回 `TrajectoryMsg[]`，每条为 `geometry_msgs/Point[]` 离散三维航点。

## 环境与技术栈

- CAT-ORA 阶段 1：镜像 `uav/catora:noetic-upstream`（4.63 GB），Compose project `uav-catora`，容器 `catora-upstream`
- CAT-ORA 阶段 2：镜像 `uav/catora:noetic-px4`（4.69 GB），Compose project `uav-catora-px4`，容器 `catora-planner-px4` + `catora-px4-gazebo`
- CAT-ORA 软件栈：Ubuntu 20.04 容器、ROS1 Noetic、catkin-tools、C++17、`mrs_lib` 1.0.4、MAVROS 1.13.0、MAVLink；PX4 v1.17.0 + Gazebo Harmonic / gz-sim 8.15.0
- CAT-ORA/PX4 锁定源码：CAT-ORA `4bc41f593eebde65dd74108a97e80430a145dfa0`；PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- 虚拟环境：ROS 全部在 Docker；PX4 镜像内 Python venv `/opt/px4-sitl-venv`；宿主没有 ROS，也没有为本项目创建 Conda 环境
- 目标机：RKOH，Ubuntu 22.04.5 LTS x86_64，20 CPU，31 GiB RAM，无 NVIDIA GPU
- 宿主能力：Docker 28.1.1、Docker Compose 2.35.1；**宿主未安装 ROS，也未创建 Conda 环境**
- 阶段 1 Docker 环境：Compose project `uav-swarm-formation`，镜像 `uav/swarm-formation:noetic-upstream`，容器 `sf-original`
- 阶段 2 ROS Docker 环境：Compose project `uav-swarm-px4-single` / `uav-swarm-px4-seven`，镜像 `uav/swarm-formation:noetic-px4`，容器 `sf-px4` / `sf-px4-seven`
- 阶段 2 飞控 Docker 环境：镜像 `uav/px4-gazebo:v1.17.0-harmonic`，容器 `px4-gazebo` / `px4-gazebo-seven`
- 容器内 Python 虚拟环境：`/opt/px4-sitl-venv`（`VIRTUAL_ENV` 和 `PATH` 已写入 PX4 镜像）；ROS Python 依赖由 ROS Noetic/Focal 镜像及 apt 管理
- 主要技术：ROS1 Noetic、catkin、MAVROS 1.20.1、MAVLink、PX4 v1.17.0 SITL、Gazebo Harmonic / gz-sim 8.15.0、Docker/Compose、Python 3 bridge
- 锁定源码：Swarm-Formation `967a4bdfae949e994691f8ffc87dbb0147cbebb7`；PX4 `v1.17.0` / `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`

## 从干净目录完整复现

以下命令均在本机 PowerShell 通过 `ssh RKOH '<command>'` 执行，或登录 RKOH 后直接执行。RKOH 的 `127.0.0.1:7890` 是 SSH RemoteForward 到本机代理；若网络可直连 GitHub，可去掉代理变量和 build args。

### CAT-ORA 两阶段

1. 创建目录、克隆并锁定源码：

   ```bash
   mkdir -p /home/noah/code/UAV/catora-px4-gazebo
   cd /home/noah/code/UAV/catora-px4-gazebo
   export HTTP_PROXY=http://127.0.0.1:7890 HTTPS_PROXY=http://127.0.0.1:7890
   git clone https://github.com/ctu-mrs/catora.git
   git -C catora checkout 4bc41f593eebde65dd74108a97e80430a145dfa0
   git clone --branch v1.17.0 --depth 1 https://github.com/PX4/PX4-Autopilot.git
   git -C PX4-Autopilot submodule update --init --recursive --depth 1
   mkdir -p .px4-sim-cache
   ```

2. 从本工作区同步部署资产（本机 PowerShell）：

   ```powershell
   scp -r F:\UAV\catora-px4-gazebo\deployment RKOH:/home/noah/code/UAV/catora-px4-gazebo/
   scp F:\UAV\catora-px4-gazebo\README.md RKOH:/home/noah/code/UAV/catora-px4-gazebo/
   ```

3. 构建并验收原版 CAT-ORA：

   ```bash
   cd /home/noah/code/UAV/catora-px4-gazebo
   docker build --network host \
     --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
     --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
     -f deployment/stage1/Dockerfile -t uav/catora:noetic-upstream . \
     2>&1 | tee catora_upstream_build.log
   cd deployment/stage1
   docker compose up -d
   ./verify.sh
   docker compose down
   ```

   判据不是仅看节点：脚本实际调用 `/mrs_formation_reshaping/get_reshaping_trajectories`，检查轨迹数、航点数、起终点集合、采样速度和最小机间距。本次结果：4 条轨迹各 31 点，最大采样速度 `1.000 m/s`，最小间距 `6.000 m`。

4. 构建 PX4 v1.17.0 / Gazebo Harmonic 镜像并首次编译 SITL：

   ```bash
   cd /home/noah/code/UAV/catora-px4-gazebo
   docker build --network host \
     --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
     --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
     --build-arg PX4_VERSION=v1.17.0 \
     -f deployment/px4/Dockerfile \
     -t uav/px4-gazebo:v1.17.0-harmonic deployment/px4
   cd deployment/px4
   docker compose up
   # 看到 PX4/Gazebo 正常启动后 Ctrl-C，再执行：
   docker compose down
   ```

5. 构建 CAT-ORA/PX4 执行器并做 4 机冷启动验收：

   ```bash
   cd /home/noah/code/UAV/catora-px4-gazebo
   docker build --network host \
     --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
     --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
     -f deployment/stage2/Dockerfile -t uav/catora:noetic-px4 . \
     2>&1 | tee catora_px4_build.log
   cd deployment/stage2
   docker compose up -d
   ./verify.sh
   # 停止：docker compose down
   ```

   本次冷启动结果：4 个 PX4、4 个 Gazebo x500、4/4 MAVROS `connected + armed + OFFBOARD`；每机 CAT-ORA 轨迹 65 点；MAVROS 最大终点误差 `0.060 m`，Gazebo 刚体最大终点误差 `0.071 m`。是否仍在运行须以 `docker compose ps` 实时检查。

两阶段均使用 host network/ROS 11311，必须串行启动。更详细说明与验收边界见 `/home/noah/code/UAV/catora-px4-gazebo/README.md`。

### 已完成的 Swarm-Formation 两阶段（历史基线）

1. 创建目录、克隆并锁定源码：

   ```bash
   mkdir -p /home/noah/code/UAV/swarm-formation-px4-gazebo
   cd /home/noah/code/UAV/swarm-formation-px4-gazebo
   export HTTP_PROXY=http://127.0.0.1:7890 HTTPS_PROXY=http://127.0.0.1:7890
   git clone https://github.com/ZJU-FAST-Lab/Swarm-Formation.git
   git -C Swarm-Formation checkout 967a4bdfae949e994691f8ffc87dbb0147cbebb7
   git clone --branch v1.17.0 --depth 1 https://github.com/PX4/PX4-Autopilot.git
   git -C PX4-Autopilot submodule sync --recursive
   git -C PX4-Autopilot submodule update --init --recursive --depth 1 --force
   ```

2. 从本工作区把已固化的部署资产同步到干净的远端目录（本机 PowerShell）：

   ```powershell
   scp -r F:\UAV\swarm-formation-px4-gazebo\deployment RKOH:/home/noah/code/UAV/swarm-formation-px4-gazebo/
   ```

3. 构建并验证原版 Swarm-Formation（ROS 只在 Docker 中）：

   ```bash
   cd /home/noah/code/UAV/swarm-formation-px4-gazebo/Swarm-Formation
   docker build --progress=plain -t uav/swarm-formation:noetic-upstream . \
     2>&1 | tee /home/noah/code/UAV/swarm-formation-px4-gazebo/swarm_formation_upstream_build.log
   cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage1
   docker compose up -d
   ./verify.sh
   docker compose down
   ```

4. 构建 PX4/Gazebo 依赖镜像。必须保持带 RemoteForward 的 SSH 会话存活，因为 Docker 构建会经 `127.0.0.1:7890` 下载官方脚本和包：

   ```bash
   cd /home/noah/code/UAV/swarm-formation-px4-gazebo
   docker build --network host \
     --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
     --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
     --build-arg PX4_VERSION=v1.17.0 \
     -f deployment/px4/Dockerfile \
     -t uav/px4-gazebo:v1.17.0-harmonic deployment/px4 \
     2>&1 | tee px4_gazebo_image_build.log
   mkdir -p .px4-sim-cache
   ```

5. 首次编译 PX4 SITL/Gazebo 插件。Compose 挂载源码并以 UID/GID 1000 运行，避免生成 root 属主文件：

   ```bash
   cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/px4
   docker compose up -d
   docker logs -f px4-gazebo
   # 看到 Startup script returned successfully 后停止
   docker compose down
   ```

   若首次构建在 `PX4-OpticalFlow` 或 Micro-CDR 下载处失败，确认 SSH 代理仍在，然后再次执行 `docker compose up -d`；CMake 会复用已完成目标。构建日志可另存为 `/home/noah/code/UAV/swarm-formation-px4-gazebo/px4_sitl_build.log`。

6. 构建带 MAVROS 和自研桥的 planner 镜像：

   ```bash
   cd /home/noah/code/UAV/swarm-formation-px4-gazebo
   docker build --network host \
     --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
     --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
     -f deployment/stage2/Dockerfile.planner-px4 \
     -t uav/swarm-formation:noetic-px4 deployment/stage2 \
     2>&1 | tee swarm_formation_px4_build.log
   ```

7. 单机物理闭环验收：

   ```bash
   cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2
   docker compose -f compose.single.yaml up -d
   ./verify_single.sh
   docker compose -f compose.single.yaml down
   ```

8. 七机编队物理闭环验收：

   ```bash
   cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2
   docker compose -f compose.seven.yaml up -d
   ./verify_seven.sh
   # 需要停止时：docker compose -f compose.seven.yaml down
   ```

   三套 Compose 都使用 host network 和 ROS master 11311，必须串行运行，不能同时启动。验收脚本是最终判据，不以“容器还在运行”代替功能验收。

## 已完成

- CAT-ORA 独立工程已创建：本地 `F:\UAV\catora-px4-gazebo`、RKOH `/home/noah/code/UAV/catora-px4-gazebo`；源码锁定 `4bc41f593eebde65dd74108a97e80430a145dfa0`
- 阶段 1 镜像 `uav/catora:noetic-upstream` 构建成功（`osrf/ros:noetic-desktop-full` + CTU-MRS stable PPA 的 `ros-noetic-mrs-lib` + C++17）；ROS 仍只存在于容器内
- 阶段 1 Compose 环境 `uav-catora` / 容器 `catora-upstream` 已通过确定性服务冒烟：4 条轨迹各 31 航点，最大采样速度 `1.000 m/s`（请求限制 `2.0 m/s`），最小采样机间距 `6.000 m`，起终点集合匹配
- 阶段 2 首版资产已创建并通过 Python/XML 静态检查：4 机 CAT-ORA 在线规划、MAVROS OFFBOARD 执行器、PX4/Gazebo Compose 及闭环验收脚本
- 阶段 2 冷启动复跑通过：4 个 PX4、4 个 Gazebo x500、4/4 MAVROS connected/armed/OFFBOARD；每机 65 航点，MAVROS 最大终点误差 `0.060 m`，Gazebo 刚体最大终点误差 `0.071 m`，PX4 实例日志无 error/failsafe
- 最终静态审计通过：三个 Compose 均可解析，所有 Bash 通过 `bash -n`，Python 脚本可编译，CAT-ORA/PX4 commit 匹配锁定值，CAT-ORA 上游工作树干净；容器挂载全部指向 `/home/noah/code/UAV/catora-px4-gazebo`
- 本轮已按续作流程读取当前交接、历史 handoff、Survey 总结与 `AGENTS.md`，确认继续以下部署主线
- SSH 连通 RKOH（`Host RKOH` → `noah@192.168.1.224`）
- 创建独立复现子目录：`/home/noah/code/UAV/swarm-formation-px4-gazebo`
- 调研 Swarm-Formation 使用的仿真器与 ROS 依赖（结论见下）
- 已审计 RKOH：Ubuntu 22.04.5 x86_64、20 CPU、31 GiB RAM、无 NVIDIA GPU、无宿主 ROS；Docker 28.1.1 / Compose 2.35.1 可用；根盘初始余量约 100 GiB
- 已克隆 `Swarm-Formation`，固定上游 commit `967a4bdfae949e994691f8ffc87dbb0147cbebb7`
- 阶段 1 原始镜像构建成功：`uav/swarm-formation:noetic-upstream`（ROS Noetic/Focal，约 5.21 GB）
- 阶段 1 最终回归通过：容器 `sf-original` 启动 `normal_hexagon.launch`，38 个 ROS 节点、290 个话题；向 `/move_base_simple/goal` 发中心目标后，`/drone_0_planning/pos_cmd` 为 `100.000 Hz`，0 号机 odom `x=-26.000→-7.582`
- 阶段 1 已固化为 `/home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage1/compose.yaml` 与 `verify.sh`；停止状态可随时重复启动验收
- 已克隆 PX4-Autopilot v1.17.0，固定 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，递归子模块完整
- 已构建 `uav/px4-gazebo:v1.17.0-harmonic`；容器内 Gazebo Sim 8.15.0、PX4 SITL 和 `gz_x500` 插件均构建成功，Python 工具隔离在 `/opt/px4-sitl-venv`
- PX4/Gazebo 单机底座已实际启动：`SYS_AUTOSTART=4001`，`default` world 就绪并生成 `x500_0`；Gazebo 有 IMU、磁力计、气压、NavSat、里程计、激光、光流和电机转速话题；MAVLink Onboard 链路为 PX4 `14580` → MAVROS `14540`
- 已完成阶段 2：`deployment/stage2` 对上游 launch 增加保持兼容的 `use_fake_drone` 开关，并实现 `PositionCommand → MAVROS PositionTarget`、MAVROS odom → planner odom 的桥接包
- 阶段 2 单机增强验收通过：MAVROS connected、PX4 `OFFBOARD + armed`，无 `poscmd_2_odom` 节点；目标发布后 MAVROS odom `x=-0.045→5.003`，Gazebo 世界中的 x500 刚体 `x=-0.015→4.992`，规划指令 `99.999 Hz`
- 阶段 2 七机闭环验收通过：7 个 PX4 进程、7 个 Gazebo `x500_0…x500_6`、7 条独立 MAVROS 链路全部 `OFFBOARD + armed`；45 个 ROS 节点、1202 个话题、假后端节点数 0
- 七机增强验收位移：MAVROS 世界系初始 x `[-26.10,-23.39,-26.01,-28.65,-28.61,-26.05,-23.41]`，目标后 `[-15.02,-11.55,-14.99,-18.45,-18.45,-15.02,-11.53]`；Gazebo 刚体初始 x `[-26.05,-23.41,-25.96,-28.68,-28.60,-26.05,-23.37]`，目标后 `[-14.97,-11.50,-14.98,-18.45,-18.40,-15.00,-11.48]`；0 号机规划指令 `100.000 Hz`
- 可重复部署已固化：`deployment/stage2/compose.single.yaml`、`compose.seven.yaml`、`verify_single.sh`、`verify_seven.sh`、`README.md`；七机容器曾在验收后保持运行，后续已为 CAT-ORA 测试停止
- 最终完成性审计通过：宿主 `roscore` 不存在；三个镜像均可 inspect；PX4 Python venv 导入测试通过；PX4 递归子模块无 `+/-` 偏差；当前七机有 7 个 PX4 进程、7 个 Gazebo x500、45 个 ROS 节点、0 个假后端节点、7 路 MAVROS 全部 connected/armed/OFFBOARD，PX4 日志无 `ERROR` 或已触发 failsafe
- 2026-09-30 目录整理完成：本地部署资产移动到 `F:\UAV\swarm-formation-px4-gazebo\deployment`；RKOH 的源码、部署资产、PX4 缓存和构建日志移动到 `/home/noah/code/UAV/swarm-formation-px4-gazebo`，UAV 远程根目录现只保留此子目录
- 移动后冒烟通过：阶段 1 为 38 节点/290 话题且 odom 可读；阶段 2 单机 MAVROS `x=-0.006→4.985`、Gazebo 刚体 `x=-0.006→5.004`、规划 `99.984 Hz`；七机有 7 个 PX4、7 个 x500、45 节点、0 个假后端、7/7 路 connected/armed/OFFBOARD
- 已检查七机容器实际挂载源，均指向新子目录中的 `PX4-Autopilot`、`.px4-sim-cache` 和 `deployment/px4/run_multi_px4.sh`

## 有效结论

- **Aerostack2 统一管理评估（2026-09-30）**：适合作为两套算法之上的 ROS2 多机任务/行为/平台抽象层，不适合作为“零改动容器管理器”。官方主线基于 Ubuntu 22.04 + ROS2 Humble，提供行为、行为树/Python mission API、标准 motion reference、每机 namespace、Gazebo Harmonic 和外置 `as2_platform_pixhawk`；而当前 Swarm-Formation 与 CAT-ORA 都是 ROS1 Noetic，必须增加 ROS1↔ROS2 适配层或迁移算法接口
- **建议映射**：CAT-ORA 封装为集中式 `FormationReshape` 自定义 AS2 behavior/service，输出每机带时间戳轨迹；Swarm-Formation 封装为每机持续运行的 path-planning behavior/external agent，输入 AS2 localization/map/goal，输出 AS2 trajectory/position motion reference；下游统一走 AS2 motion controller/platform → PX4
- **迁移顺序**：先做独立 `aerostack2-integration` Docker 试点，只接 1 机 PX4/Gazebo 并桥接 CAT-ORA 标准点列；再扩 4 机 CAT-ORA；最后处理自定义消息和实时反馈更多的 Swarm-Formation。短期保留现有 ROS1 算法容器，不重写核心；产品化阶段优先显式适配器/原生 ROS2 wrapper，避免长期依赖通用动态 `ros1_bridge`
- **风险**：AS2 的 Pixhawk 平台使用 ROS2/PX4 消息接口，需实测其当前版本与已锁定 PX4 v1.17.0 的 `px4_msgs` 兼容性；Aerostack2 官方目标环境不是国产 OS，后续还需单独验证目标国产发行版的 ROS2 Humble、DDS、Gazebo 和设备驱动支持
- **CAT-ORA 定位**：它不订阅飞行器状态来闭环控制，而是用起始配置、目标配置、`dt/max_velocity/max_acceleration` 一次性生成多机离散轨迹；动力学接入必须另加时间轨迹执行层
- **CAT-ORA 数据链**：CAT-ORA ROS service → 4 条 0.2 s 离散航点 → 25 Hz 线性插值/50 Hz PositionTarget → MAVROS → MAVLink → PX4 位置控制器 → Gazebo x500 电机/刚体动力学 → MAVROS odom 与 Gazebo world pose 双重验收
- **阶段 2 场景**：4 架 x500 从 `(-4,-4),(-4,4),(4,-4),(4,4)` 起飞至 1.5 m，CAT-ORA 自动分配并变形成 `x=8` 的纵列；输出目标顺序体现了 CAT-ORA assignment，而不是简单按输入目标序号直连
- **动力学边界**：CAT-ORA 本身的速度/加速度约束用于生成参考轨迹；实际跟踪受 PX4 控制器和 x500 动力学约束。当前以终点误差、飞控模式、刚体位置证明闭环，尚未加入风扰或全程 RMS 跟踪误差
- **版本选择**：继续使用 PX4 v1.17.0 + Gazebo Harmonic，这是 Ubuntu 22.04 上 PX4 v1.17 官方支持组合；ROS1 Noetic/MAVROS 放在独立 Focal 容器中，不在宿主装 ROS
- **仿真器**：不用 Gazebo/AirSim/PX4 SITL，而是仓库自带的轻量仿真 `src/uav_simulator`。这是 FAST-Lab 从 Fast-Planner/EGO 系列沿用下来的 ROS 包集合，部分包（`so3_*`、`quadrotor_msgs`、`mockamap` 等）源自 UPenn/港科大
  - 运动模型：默认 `poscmd_2_odom`（位置指令直接当里程计，理想跟踪）；`so3_quadrotor_simulator` + `so3_control` 在 `simulator.xml` 中被注释
  - 地图：`map_generator/random_forest`（由 `normal_hexagon.launch` 启动）；`mockamap` 可选
  - 感知：`local_sensing/pcl_render_node` 从全局点云裁局部点云（可选 CUDA 深度渲染）
  - 可视化：RViz（`rviz.launch`）；"2D Nav Goal" 下目标需 `flight_type=3`
  - 局限：无物理引擎、无传感器噪声/真实成像、无飞控参与，只能验证规划算法本身
- **ROS**：强依赖 ROS1（catkin_make / roslaunch）；官方测试 Ubuntu 18.04 Melodic、20.04 Noetic；提供 Noetic Docker（`make docker_build` / `make docker_run`）；依赖 `libarmadillo-dev`；推荐 `rosmon`
- **复现策略**：已按两步完成：先用自带轻量仿真跑通算法，再迁到 Gazebo + PX4 SITL，并用自研 MAVROS 位置设定点桥接接入原 ROS1 planner
- **PX4/Gazebo 锁版**：采用稳定版 PX4 v1.17.0 + Gazebo Harmonic；PX4 官方 v1.17 文档明确 Harmonic 是 Ubuntu 22.04+ 唯一受支持的 Gazebo 系列。ROS1 planner 与 PX4 之间使用 MAVROS/MAVLink，避免把旧 ROS1 算法强行迁移到 ROS2
- **版本复核（2026-09-29）**：PX4 官方 Release 页面仍将 v1.17.0 标为 Latest Stable；官方 v1.17 Gazebo 文档说明 Ubuntu 22.04 默认安装 Harmonic，且 Gazebo（非 Classic）是 22.04+ 唯一受支持系列
- **环境划分**：规划器/ROS1 使用镜像 `uav/swarm-formation:noetic-upstream`（后续派生 `:noetic-px4`）；飞控/物理仿真使用 `uav/px4-gazebo:v1.17.0-harmonic`。两容器使用 host network，通过 MAVLink UDP 连接；宿主不安装 ROS
- **阶段 2 数据链**：`drone_i_planning/pos_cmd` → 自研 ROS bridge → `uav{i}/mavros/setpoint_raw/local` → MAVLink → PX4 位置控制器 → Gazebo x500 刚体/电机动力学 → MAVLink odom → bridge → `drone_i_visual_slam/odom` → planner
- **多机端口/ID**：实例 `i=0…6` 使用 PX4 本地 UDP `14580+i`、MAVROS 本地 UDP `14540+i`、`MAV_SYS_ID=i+1`，Gazebo 模型名 `x500_i`
- **坐标处理**：PX4 local x/y 原点在各自出生点；bridge 向 planner odom 加原 hexagon x/y 偏移，向 PX4 setpoint 减该偏移。z 不加偏移，先位置控制垂直起飞到 0.5 m，再开放 planner 横移
- **控制接口**：只向 PX4 发送位置+yaw，mask 掉 planner 的速度/加速度前馈；真动力学和内环仍由 PX4/Gazebo 计算。七机仿真慢于实时，直接前馈会放大地面瞬态并导致翻机
- **headless 解锁**：无 QGC 时 PX4 v1.17 默认 `NAV_DLL_ACT=2` 会因无 GCS 拒绝解锁；bridge 显式、可配置地设置 `NAV_DLL_ACT=0`，先连续发送 setpoint，再请求 OFFBOARD 和 arm
- **当前能力边界**：飞控、刚体和电机动力学已迁到 Gazebo；规划感知仍使用 ROS 生成点云，ROS 障碍物尚未同步为 Gazebo 碰撞体，因此这是动力学集成基线，尚不是传感器/碰撞都物理一致的丛林 demo
- 硬件/算法选型结论见 `docs/HANDOFF_13eb16da-bb97-44f9-9586-fc6f80e5789e.md` 与 Notion 选型总结

## 卡住的问题

无外部阻塞。CAT-ORA 原版与 4 机 PX4/Gazebo 动力学闭环均已完成并通过冷启动验收。

## 下一步

1. 若采用 Aerostack2，新建独立 `aerostack2-integration` 目录做单机试点：ROS2 Humble + AS2 + Gazebo Harmonic + PX4 v1.17.0，先验证平台接口版本兼容性
2. 先实现 CAT-ORA → AS2 trajectory 的 4 机适配与统一 mission；通过后再实现 Swarm-Formation odom/map/goal/PositionCommand 双向适配
3. 可选：增加全程 RMS/峰值跟踪误差、速度/加速度实测和机间最小距离，并加入风场、噪声、延迟与故障注入
4. 可选：限制或轮转 PX4 ULog，避免多机容器长期运行再次产生数十 GB 日志

## 踩过的坑

- 仓库网页 README 声明 GPL-3.0，但 `package.xml` 写 MIT，且仓库根目录没有独立 LICENSE 文件；对外分发或集成前应按更保守的 GPL-3.0 处理并向上游确认，不能仅依据 `package.xml`
- 已有 PX4 工作树一度显示 76 GB，其中约 74 GB 是 `build/px4_sitl_default/rootfs/0…6/log` 的长期多机 ULog，不是构建产物；磁盘只剩 16 GB 时不要整树复制。本次用 `rsync -a --exclude="build/px4_sitl_default/rootfs/[0-9]*/"` 复制源码和编译产物，得到独立的 2.6 GB CAT-ORA PX4 树，旧目录未删除
- 多机 MAVROS 冷启动参数尚未同步完成时，会暂时报 `NAV_DLL_ACT unknown` / `parameter receiving not complete`；执行器每 2 秒重试，参数同步完成后 4 路均自动进入 OFFBOARD/armed，这不等同于最终失败
- CAT-ORA 上游 `CMakeLists.txt` 注释掉了 C++ 标准设置，但源码实际使用 `std::filesystem`、`std::scoped_lock`，新 `mrs_lib` 头文件也依赖 C++17；默认编译会大量报错。保持源码不改，在 `catkin config` 中显式传 `-DCMAKE_CXX_STANDARD=17`
- 从 `ros:noetic-ros-base-focal` 安装 `mrs_lib` 会间接下载数百个 PCL/Qt/OpenCV 等包；阶段 1 改用公开 `osrf/ros:noetic-desktop-full`，依赖更符合官方用法且开发板可复用已有层缓存
- WebFetch 会吞掉 launch/XML 内容；改用 `ssh RKOH 'curl -x http://127.0.0.1:7890 <raw_url>'` 拉原文（RKOH 经 SSH RemoteForward 走本机代理）
- 本地是 PowerShell：ssh 远程命令用单引号包裹，避免 `|`、正则被本地解析
- RKOH 登录 shell 是 zsh；`${PIPESTATUS[0]}` 是 Bash 语法，在 zsh 中导致镜像实际构建成功但包装命令退出 1。需要显式 `bash -lc` 或使用 zsh 的 `pipestatus`
- zsh 的 `status` 是只读特殊变量，包装命令应使用 `rc` 等变量名
- GitHub 直连经常超时；Git/submodule 使用 RKOH 的 `HTTP_PROXY/HTTPS_PROXY=http://127.0.0.1:7890`。Docker BuildKit 还要加 `--network host` 并传同名 `--build-arg`
- PX4 外部模块 `PX4-OpticalFlow`、Micro-CDR 首次构建会联网；未传代理时会失败，重新带代理构建可复用已完成目标
- 将宿主 PX4 源码挂到 root 容器会产生 root 文件并触发 Git dubious ownership；运行账户统一为 `1000:1000`，`HOME=/tmp`，已有 root 文件需限定在 PX4 仓库内修正属主
- PX4 的 `pxh>` 在无 TTY 时会刷提示符；Compose 需 `stdin_open: true`、`tty: true`
- MAVROS 安装包不包含必需的 GeographicLib 数据；缺失时主进程报 `egm96-5.pgm` 不可读并退出。镜像构建必须运行 `/opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh`
- 首版手写 patch hunk 行数错误，`git apply` 报 `corrupt patch`；应先用 `git apply --check`，本次已按真实上游文件生成正确 patch
- 后台 Docker build 若 SSH 关闭，`127.0.0.1:7890` 的 RemoteForward 也会消失；需要保持 SSH 会话前台构建，或建立独立持久隧道
- PX4 多实例脚本最初用 `-w instance_i`，导致相对启动脚本 `etc/init.d-posix/rcS` 找不到；最终每个实例从同一个 `build/px4_sitl_default/rootfs` cwd 启动，使用唯一 `-i`
- 多机初版把 planner z=0.5 映射成 PX4 local z=0，导致贴地横移、随机翻机；z 偏移应为 0，并在无 planner command 时先保持 x/y、起飞到 local z=0.5
- 七机慢于实时环境下直接转发 planner 速度/加速度前馈，部分 x500 会出现大姿态瞬态；当前可靠基线只转发平滑位置轨迹+yaw，由 PX4 内环生成速度/加速度
- 七机冷启动约需 40 秒；只看到 7 个模型并不表示 MAVROS 已全部完成参数同步，冒烟必须继续等待 7 路 state 都达到 connected/armed/OFFBOARD。过早检查会出现暂态 `NAV_DLL_ACT` unknown/parameter receiving not complete，随后重试可自动恢复
- 上游 Dockerfile 注释称 Melodic、实际基于 `osrf/ros:noetic-desktop-full`；`python3-pip\ ` 排版可疑，但本次可正常解析并构建。README 明示 `libarmadillo-dev`，基础镜像当前已间接带入，后续自有 Dockerfile 应显式声明
- 无 GPU/桌面会话时先用 headless 验收；RViz 需 X11 转发或 VNC，不应作为算法链路是否跑通的唯一依据

## 相关指令 / 链接

- SSH：`ssh RKOH`；远程工程路径：`/home/noah/code/UAV/swarm-formation-px4-gazebo`
- CAT-ORA 新工程：`/home/noah/code/UAV/catora-px4-gazebo`；本地部署资产：`F:\UAV\catora-px4-gazebo`
- CAT-ORA 上游：https://github.com/ctu-mrs/catora （commit `4bc41f593eebde65dd74108a97e80430a145dfa0`）
- CAT-ORA 阶段 1 环境：镜像 `uav/catora:noetic-upstream`、Compose 容器 `catora-upstream`；宿主未安装 ROS/Conda
- CAT-ORA 完整说明：`/home/noah/code/UAV/catora-px4-gazebo/README.md`；阶段 1 日志 `catora_upstream_build.log`；阶段 2 日志 `catora_px4_build.log`
- CAT-ORA 状态检查与复验：`cd /home/noah/code/UAV/catora-px4-gazebo/deployment/stage2 && docker compose ps && ./verify.sh`
- Aerostack2：https://github.com/aerostack2/aerostack2；官方文档：https://aerostack2.github.io/；PX4 平台：https://github.com/aerostack2/as2_platform_pixhawk
- 本地部署资产入口：`F:\UAV\swarm-formation-px4-gazebo\README.md`
- Swarm-Formation：https://github.com/ZJU-FAST-Lab/Swarm-Formation
- 阶段 1 构建日志：`/home/noah/code/UAV/swarm-formation-px4-gazebo/swarm_formation_upstream_build.log`
- 构建：`cd /home/noah/code/UAV/swarm-formation-px4-gazebo/Swarm-Formation && docker build --progress=plain -t uav/swarm-formation:noetic-upstream .`
- 启动：`docker run -d --name sf-original --network host uav/swarm-formation:noetic-upstream bash -lc 'source /workspace/devel/setup.bash && exec roslaunch ego_planner normal_hexagon.launch'`
- 触发：`docker exec sf-original bash -lc 'source /workspace/devel/setup.bash && rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 10.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"'`
- PX4 v1.17 Gazebo：https://docs.px4.io/v1.17/en/sim_gazebo_gz/
- PX4 v1.17 release：https://docs.px4.io/main/en/releases/1.17 与 https://github.com/PX4/PX4-Autopilot/releases
- PX4 路径：`/home/noah/code/UAV/swarm-formation-px4-gazebo/PX4-Autopilot`；镜像定义：`/home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/px4/`
- PX4 构建日志：`/home/noah/code/UAV/swarm-formation-px4-gazebo/px4_sitl_build.log`；镜像日志：`/home/noah/code/UAV/swarm-formation-px4-gazebo/px4_gazebo_image_build.log`
- PX4 独立单机底座启动：`cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/px4 && docker compose up -d`（容器名 `px4-gazebo`；与阶段 2 Compose 串行使用）
- 阶段 2 详细说明：`/home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2/README.md`
- 阶段 2 镜像构建：`cd /home/noah/code/UAV/swarm-formation-px4-gazebo && docker build --network host --build-arg HTTP_PROXY=http://127.0.0.1:7890 --build-arg HTTPS_PROXY=http://127.0.0.1:7890 -f deployment/stage2/Dockerfile.planner-px4 -t uav/swarm-formation:noetic-px4 deployment/stage2`
- 单机：`cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2 && docker compose -f compose.single.yaml up -d && ./verify_single.sh`
- 七机：`cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2 && docker compose -f compose.seven.yaml up -d && ./verify_seven.sh`
- 停止当前七机：`cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2 && docker compose -f compose.seven.yaml down`
- Swarm 七机容器名：`px4-gazebo-seven`、`sf-px4-seven`；验收时实测资源约 PX4/Gazebo 6.4 CPU 核/303 MiB，ROS planner 1.5 CPU 核/5.95 GiB
- 选型总结（Notion）：https://app.notion.com/p/3e98daea7f2781f2ad91d6f2129c4039
