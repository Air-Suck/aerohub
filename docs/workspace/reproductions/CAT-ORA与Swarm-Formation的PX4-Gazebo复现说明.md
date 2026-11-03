# CAT-ORA 与 Swarm-Formation 的 PX4/Gazebo 复现说明

## 1. 复现范围

本项目对两个多无人机 planner 做了两级复现：

1. 在算法原生 ROS1 环境中运行上游程序，确认规划接口和输出正常。
2. 去掉理想运动后端，接入 MAVROS、PX4 SITL 和 Gazebo Harmonic，由 PX4 完成飞控闭环，由 Gazebo 计算机体、电机和碰撞动力学。

最终 demo 包含：

- CAT-ORA：16 架 x500 从 4×4 方阵重构为圆环。
- Swarm-Formation：7 架 x500 保持六边形队形，穿越按官方 `normal_hexagon` 参数生成的随机障碍场（60 根圆柱、20 个椭圆环）。

两套 demo 都来自 Gazebo 中 PX4 驱动的刚体运动，不是离线轨迹动画。

## 2. 版本与运行环境

| 项目 | 固定版本 |
| --- | --- |
| CAT-ORA | commit `4bc41f593eebde65dd74108a97e80430a145dfa0` |
| Swarm-Formation | commit `967a4bdfae949e994691f8ffc87dbb0147cbebb7` |
| PX4-Autopilot | v1.17.0，commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| Gazebo | Gazebo Harmonic，`gz-sim 8` |
| planner 环境 | Ubuntu 20.04、ROS1 Noetic、MAVROS |
| 宿主环境 | Ubuntu 22.04、Docker 28、Docker Compose v2 |

已验证机器为 `RKOH_Remote`，20 核 CPU、31 GiB 内存、无 NVIDIA GPU。宿主不需要安装 ROS 或 Conda。ROS、MAVROS 和算法依赖均在 Docker 镜像内，PX4 镜像中的 Python 环境位于 `/opt/px4-sitl-venv`。

远端工程结构：

```text
/home/noah/code/UAV/
├── catora-px4-gazebo/
│   ├── catora/
│   ├── PX4-Autopilot/
│   ├── .px4-sim-cache/
│   └── deployment/
├── swarm-formation-px4-gazebo/
│   ├── Swarm-Formation/
│   ├── PX4-Autopilot/
│   ├── .px4-sim-cache/
│   └── deployment/
└── demo/
```

本地保存部署文件和成品视频，算法及 PX4 源码树位于 RKOH。

## 3. 公共架构

### 3.1 容器划分

每个闭环复现由两个容器组成：

- planner 容器：ROS1 Noetic、算法、MAVROS、适配器。
- PX4/Gazebo 容器：PX4 SITL、Gazebo Harmonic、x500 模型。

两个容器使用 `network_mode: host`。MAVROS 通过 MAVLink UDP 连接各 PX4 实例。Gazebo 以 headless 模式运行，并通过 Mesa llvmpipe 做软件渲染。

### 3.2 闭环边界

```text
planner 输出
    ↓
ROS1 适配器
    ↓
MAVROS PositionTarget
    ↓
MAVLink UDP
    ↓
PX4 位置控制器
    ↓
Gazebo x500 刚体和电机动力学
    ↓
MAVROS odometry
    ↓
planner 反馈输入
```

两套系统共用宿主网络、ROS master、MAVLink 和 Gazebo 端口，必须串行运行。

## 4. 准备源码和部署文件

以下命令在 RKOH 上执行。若 GitHub 需要本机转发代理，先保持带 RemoteForward 的 SSH 会话存活，再设置：

```bash
export HTTP_PROXY=http://127.0.0.1:7890
export HTTPS_PROXY=http://127.0.0.1:7890
```

### 4.1 CAT-ORA

```bash
mkdir -p /home/noah/code/UAV/catora-px4-gazebo
cd /home/noah/code/UAV/catora-px4-gazebo

git clone https://github.com/ctu-mrs/catora.git
git -C catora checkout 4bc41f593eebde65dd74108a97e80430a145dfa0

git clone --branch v1.17.0 --depth 1 https://github.com/PX4/PX4-Autopilot.git
git -C PX4-Autopilot submodule update --init --recursive --depth 1
mkdir -p .px4-sim-cache
```

从 Windows 工作区同步部署文件：

```powershell
scp -r F:\UAV\catora-px4-gazebo\deployment RKOH_Remote:/home/noah/code/UAV/catora-px4-gazebo/
scp F:\UAV\catora-px4-gazebo\README.md RKOH_Remote:/home/noah/code/UAV/catora-px4-gazebo/
```

### 4.2 Swarm-Formation

```bash
mkdir -p /home/noah/code/UAV/swarm-formation-px4-gazebo
cd /home/noah/code/UAV/swarm-formation-px4-gazebo

git clone https://github.com/ZJU-FAST-Lab/Swarm-Formation.git
git -C Swarm-Formation checkout 967a4bdfae949e994691f8ffc87dbb0147cbebb7

git clone --branch v1.17.0 --depth 1 https://github.com/PX4/PX4-Autopilot.git
git -C PX4-Autopilot submodule sync --recursive
git -C PX4-Autopilot submodule update --init --recursive --depth 1 --force
mkdir -p .px4-sim-cache
```

同步部署文件：

```powershell
scp -r F:\UAV\swarm-formation-px4-gazebo\deployment RKOH_Remote:/home/noah/code/UAV/swarm-formation-px4-gazebo/
scp F:\UAV\swarm-formation-px4-gazebo\README.md RKOH_Remote:/home/noah/code/UAV/swarm-formation-px4-gazebo/
```

最终 demo 资产对两套工程共用：

```powershell
scp -r F:\UAV\demo RKOH_Remote:/home/noah/code/UAV/
```

如果远端已有 `demo` 目录，可只同步本次修改的脚本、SDF 和 Compose override，避免复制本地播放器缓存或旧视频。

## 5. 构建 PX4/Gazebo 公共镜像

两套工程使用相同镜像名 `uav/px4-gazebo:v1.17.0-harmonic`。任选一个工程构建一次即可：

```bash
cd /home/noah/code/UAV/catora-px4-gazebo
docker build --network host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
  --build-arg PX4_VERSION=v1.17.0 \
  -f deployment/px4/Dockerfile \
  -t uav/px4-gazebo:v1.17.0-harmonic deployment/px4
```

首次使用还要编译 PX4 SITL 和 Gazebo 插件：

```bash
cd /home/noah/code/UAV/catora-px4-gazebo/deployment/px4
docker compose up
# 出现 "Startup script returned successfully" 后按 Ctrl-C
docker compose down
```

首次构建会下载 PX4 子模块、Micro-CDR 和 PX4-OpticalFlow。下载中断后可重新执行，CMake 会复用已经完成的目标。

## 6. CAT-ORA 复现

### 6.1 算法作用

CAT-ORA 接收初始编队、目标编队、采样周期、最大速度和最大加速度，输出每架无人机的离散三维轨迹。它本身不直接控制飞行器，因此 PX4 复现增加了轨迹执行器。

最终数据链为：

```text
/mrs_formation_reshaping/get_reshaping_trajectories
    ↓ 16 条离散轨迹，采样周期 0.2 s
catora_px4_executor.py
    ↓ 25 Hz 插值，50 Hz 位置目标
/uavN/mavros/setpoint_raw/local
    ↓
PX4 OFFBOARD
    ↓
Gazebo x500
```

### 6.2 上游算法冒烟

构建原版算法镜像：

```bash
cd /home/noah/code/UAV/catora-px4-gazebo
docker build --network host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
  -f deployment/stage1/Dockerfile \
  -t uav/catora:noetic-upstream .
```

运行服务级验收：

```bash
cd /home/noah/code/UAV/catora-px4-gazebo/deployment/stage1
docker compose up -d
./verify.sh
docker compose down
```

验收脚本会调用 CAT-ORA 服务，检查轨迹数量、航点数量、起终点集合、采样速度和最小机间距。该步骤只验证算法，不启动 PX4 和 Gazebo。

### 6.3 构建 PX4 执行器

```bash
cd /home/noah/code/UAV/catora-px4-gazebo
docker build --network host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
  -f deployment/stage2/Dockerfile \
  -t uav/catora:noetic-px4 .
```

镜像内包含 CAT-ORA、MAVROS 和 `catora_px4_backend`。执行器会关闭无 QGC 时的 data-link 解锁限制，连续发送 setpoint，再请求 OFFBOARD 和 arm。

### 6.4 16 机方阵到圆环

初始位置是 4×4 方阵：

```text
x ∈ {-12, -9, -6, -3} m
y ∈ {-4.5, -1.5, 1.5, 4.5} m
z = 1.5 m
```

目标是圆心 `(-7.5, 0)`、半径 `6 m`、高度 `1.5 m` 的 16 点圆环。规划参数为：

```text
trajectory_dt = 0.2 s
max_velocity = 1.2 m/s
max_acceleration = 0.6 m/s²
execution_time_scale = 20
```

`execution_time_scale` 用于补偿 16 机软件渲染时仿真慢于墙钟的问题，同时缩放轨迹持续时间和采样索引。

PX4 实例号超过 9 时，上游脚本会把多个 offboard 返回端口折叠到 `14549`。本项目把 16 路 MAVROS 返回端口改为 `14640…14655`，对应 PX4 端口 `14580…14595`，避免后 6 路连接冲突。

运行闭环验收：

```bash
cd /home/noah/code/UAV/catora-px4-gazebo/deployment/stage2
docker compose up -d
./verify.sh
docker compose down
```

通过条件：

- 16 个 PX4 进程和 16 个 Gazebo `x500` 模型存在。
- 16 路 MAVROS 都是 `connected + armed + OFFBOARD`。
- planner 最大终点误差小于 `1 m`。
- Gazebo 圆环最大终点误差小于 `1 m`。
- 最终最小机间距大于 `1.5 m`。

最终实测的 Gazebo 最大圆环误差为 `0.107 m`，最小机间距为 `2.267 m`。

### 6.5 录制最终 CAT-ORA demo

```bash
cd /home/noah/code/UAV
./demo/run_catora_demo.sh
```

脚本会完成以下操作：

1. 使用 `catora_demo.sdf` 启动 headless Gazebo。
2. 等待 16 机完成起飞并进入 ready 状态。
3. 订阅 `/demo/camera/image`，将图像保存为 PPM 帧。
4. 调用 stage2 `verify.sh`，只有闭环验收通过才继续编码。
5. 用 OpenCV 添加标题，再用 FFmpeg/libx264 生成兼容 MP4并做完整解码。

输出文件：

```text
/home/noah/code/UAV/demo/catora_px4_gazebo_headless.mp4
```

## 7. Swarm-Formation 复现

### 7.1 上游与动力学后端的差异

上游 Swarm-Formation 默认使用 `poscmd_2_odom`，位置指令会直接变成下一帧里程计。这种后端适合验证 planner，但没有飞控和机体动力学。

PX4 版本增加 `use_fake_drone` 开关并关闭 `poscmd_2_odom`，使用 `px4_backend_bridge` 完成两向转换：

```text
/drone_N_planning/pos_cmd
    ↓ position_command_bridge.py
/uavN/mavros/setpoint_raw/local
    ↓
PX4 + Gazebo
    ↓ /uavN/mavros/local_position/odom
position_command_bridge.py
    ↓
/drone_N_visual_slam/odom
```

每个 PX4 的 local x/y 原点位于出生点。bridge 在 odometry 上加编队偏移，在 planner setpoint 上减同一偏移，使 planner 始终使用世界坐标。z 不加偏移。

bridge 原样转发 planner 的位置、速度、加速度和 yaw，除出生点坐标偏移外不裁剪或改写轨迹。没有有效 planner 指令时，各机先在 `1.0 m` 高度保持；收到非有限指令时保持当前位置。这样 PX4 能利用速度和加速度前馈跟踪曲线路径，避免只发位置目标产生的滞后和切角。

### 7.2 上游算法冒烟

```bash
cd /home/noah/code/UAV/swarm-formation-px4-gazebo/Swarm-Formation
docker build --progress=plain \
  -t uav/swarm-formation:noetic-upstream .

cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage1
docker compose up -d
./verify.sh
docker compose down
```

验收脚本启动 `normal_hexagon.launch`，发布中心目标，确认 planner 约 100 Hz 输出位置指令，并检查虚拟 odometry 已产生预期位移。

### 7.3 构建 PX4 planner 镜像

```bash
cd /home/noah/code/UAV/swarm-formation-px4-gazebo
docker build --network host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
  -f deployment/stage2/Dockerfile.planner-px4 \
  -t uav/swarm-formation:noetic-px4 deployment/stage2
```

构建时会应用 `swarm_formation_px4.patch`，并设置最终 demo 参数：

| 参数 | 值 |
| --- | ---: |
| `max_vel` | `0.3 m/s` |
| `max_acc` | `0.4 m/s²` |
| `swarm_scale` | `1.0` |
| `weight_obstacle` | `100000` |
| `weight_swarm` | `50000` |
| `weight_formation` | `15000` |
| `obstacle_clearance` | `0.9 m` |
| `swarm_clearance` | `0.5 m` |
| `flight_type` | `2`，预设单终点 |
| `realworld_experiment` | `true`，等待触发 |

补丁还会给每架无人机的目标叠加六边形相对偏移，并修复安全重规划中的未初始化时间、formation cost 丢失、并发轨迹写入和地图外空间误判为空闲等问题。二维树林实验把障碍 ESDF 的 z 梯度置零，避免从树下方绕行。

### 7.4 基础单机与七机验收

单机：

```bash
cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2
docker compose -f compose.single.yaml up -d
./verify_single.sh
docker compose -f compose.single.yaml down
```

七机：

```bash
cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2
docker compose -f compose.seven.yaml up -d
./verify_seven.sh
docker compose -f compose.seven.yaml down
```

七机实例 `i=0…6` 使用：

```text
PX4 UDP:       14580+i
MAVROS UDP:    14540+i
MAV_SYS_ID:    i+1
Gazebo model:  x500_i
```

验收要求 7 个 PX4、7 个 x500、7 路 MAVROS 全部进入 OFFBOARD 并解锁，同时检查 planner 指令、MAVROS odometry 和 Gazebo world pose。

### 7.5 随机树林场景

最终场景由 `forest_layout.py`、`forest_map_publisher.py` 和 `swarm_formation_demo.sdf` 共同定义。布局沿用官方 `random_forest_sensing` 的均匀分布和尺寸参数；上游使用 `random_device`，为让录制可重复，本项目固定 seed `31`：

- 地图尺寸：`34×15×3 m`。
- 圆柱：60 根，基础宽度 `0.5 m`，随机缩放 `0.5–1.5`，高度 `2–3 m`，柱心最小距离 `0.8 m`。
- 椭圆环：20 个，横向半径 `1.0 m`，竖向半径 `1.0–1.2 m`，中心高度 `0.7–3.0 m`。
- 点云分辨率：`0.1 m`；圆柱和圆环不额外膨胀，由 planner 的 `obstacle_clearance=0.9 m` 提供安全距离。
- planner 点云另含覆盖任务区域的 `z=0` 地面层，防止轨迹从障碍下方或地面以下穿过。

SDF 中的圆柱坐标必须与 ROS 点云坐标逐项一致。否则画面中的树和 planner 感知的障碍不在同一位置。

7 架无人机使用 `1.0×` 六边形，相邻标称距离为 `2.0 m`。整个任务只给一个全局目标：

```text
(24, 0, 1.2)
```

障碍由同一生成器均匀采样，不按航迹排列。seed `31` 下四象限圆柱数为 `18/14/13/15`，18 个等面积网格每格有 `1–5` 根圆柱；3 根圆柱直接阻挡七机的名义直线路径。因此局部无人机必须避障，编队中心仍沿全局目标方向前进。

### 7.6 无录像验收

先关闭录制验证飞行，调试速度更快：

```bash
cd /home/noah/code/UAV
SWARM_RECORD=0 ./demo/run_swarm_demo.sh
```

脚本会等待 7 路 MAVROS 全部达到 `connected + armed + OFFBOARD`，用确定性点云替换 stage2 空地图，然后发布 `/traj_start_trigger`。结果写入：

```text
/home/noah/code/UAV/demo/swarm_forest_metrics.json
```

硬门槛为：

```text
complete                          = true
final_center_x_m                  > 22.0
blocked_nominal_path_tree_count   >= 1
nominal_path_min_clearance_m      < 0.0
max_local_avoidance_deviation_m   > 0.2
min_vehicle_separation_m          > 0.8
min_command_vehicle_separation_m  > 0.8
min_obstacle_clearance_m          > 0.1
min_command_obstacle_clearance_m  > 0.1
min_altitude_m                    > 0.5
min_command_altitude_m            > 0.6
max_formation_rms_m               < 1.5
final_formation_rms_m             < 0.3
```

这些指标同时证明任务到达终点、直线路径确有障碍、实际轨迹发生横向绕行，并且没有碰树、坠地或破坏队形。

### 7.7 录制最终 Swarm demo

```bash
cd /home/noah/code/UAV
./demo/run_swarm_demo.sh
```

相机按订阅激活，配置为 `640×360 @ 3 Hz`。必须保留 `always_on=false`，让 7 架无人机先完成冷启动和起飞，再由图像订阅激活软件渲染。更高分辨率和帧率会让无 GPU 主机的软件渲染及 PPM 写盘占用控制循环时间。

任务脚本按墙钟时间记录七机实际 odometry，编码器用 PPM 文件时间与采样对齐，再按固定相机参数投影当前机位。画面使用上游官方拓扑：中心机分别连接 6 架外圈机，外圈首尾连接为六边形，共 12 条半透明红色直线。不绘制圆点、机号或历史轨迹。连线端点来自 Gazebo 实际运动，不是 planner 命令。安全验收通过后，FFmpeg 使用运动补偿光流将画面插帧到 12 fps；绘制和插帧都在飞行结束后执行，不占用 PX4 控制周期。

脚本只有在任务指标全部通过后才会编码：

```text
/home/noah/code/UAV/demo/swarm_formation_px4_gazebo_headless.mp4
```

最终实测：

| 指标 | 结果 |
| --- | ---: |
| 最终编队中心 | `(23.923, 0.042) m` |
| 最大单机局部偏移 | `1.545 m` |
| 最大编队 RMS | `1.184 m` |
| 最终编队 RMS | `0.096 m` |
| 实际/命令最小机间距 | `1.065/0.965 m` |
| 实际/命令最小障碍净空 | `0.147/0.204 m` |
| 实际/命令最低高度 | `0.781/0.707 m` |

指标文件同时记录最近障碍事件的无人机、障碍类型、编号和采样序号。`max_tracking_error_m` 仅比较回调时刻的最新命令与 odometry，未按消息时间戳对齐，因此不作为硬门槛；无碰撞验收使用实际轨迹计算出的净空、间距和高度。

## 8. 视频链路与验收

headless 录制链路：

```text
Gazebo server + Ogre2 + Mesa EGL
    ↓ /demo/camera/image
capture_gz_images
    ↓ PPM 图片序列
encode_video.py
    ↓ 实际 odometry 官方 12 边编队图、标题和 3 fps 中间视频
encode_compatible_mp4.sh
    ↓ FFmpeg minterpolate + libx264
12 fps H.264 Constrained Baseline + yuv420p + faststart MP4
```

检查现有视频：

```bash
docker run --rm \
  -v /home/noah/code/UAV/demo:/demo \
  --entrypoint python3 uav/catora:noetic-px4 \
  /demo/verify_videos.py \
  /demo/catora_px4_gazebo_headless.mp4 \
  /demo/swarm_formation_px4_gazebo_headless.mp4
```

最终文件：

| 文件 | 规格 | SHA-256 |
| --- | --- | --- |
| `catora_px4_gazebo_headless.mp4` | 1280×720，10 fps，15.6 s | `f12c825b86cedf9a951ef0bef5561b181f35da8541c4a72be31148dbc0b24978` |
| `swarm_formation_px4_gazebo_headless.mp4` | 640×360，12 fps，50.75 s | `7d6fab0863b3d44afe92fbc504c0b403c10427054ee350aef8cb6bd16124330a` |

两份视频均已通过 FFmpeg 全帧解码。远端成品位于 `/home/noah/code/UAV/demo`，本地副本位于 `F:\UAV\demo\deliverables`。

## 9. 常见问题

### 9.1 多套 demo 不能同时运行

症状包括 MAVROS 连接失败、PX4 实例退出、Gazebo 端口占用。先停止另一套 demo，再重新启动当前任务：

```bash
cd /home/noah/code/UAV/catora-px4-gazebo/deployment/stage2
docker compose down

cd /home/noah/code/UAV/swarm-formation-px4-gazebo/deployment/stage2
docker compose -f compose.seven.yaml down
```

若历史的 Aerostack2/CAT-ORA 集成容器仍在自动重建，还要停止 `uav-px4-multi`、`uav-as2-multi` 和 `aerostack2-cat-ora`。不要停止与本项目无关的容器。

### 9.2 PX4 实例已启动，但 MAVROS 暂时未就绪

多机冷启动期间会短暂出现 `NAV_DLL_ACT unknown`、`parameter receiving not complete` 或 time synchronizer reset。以验收脚本最终读取的 MAVROS state 为准，不要仅凭启动日志判断失败。

### 9.3 Swarm 有一架无人机没有离地

确认相机传感器为 `always_on=false`，并检查 7 路 MAVROS 是否都已解锁。若某架机长期处于 OFFBOARD 但 `armed: False`，停止本轮并完整重启相关 Compose；失败运行不能进入录制和编码阶段。

### 9.4 画面中的树与避障轨迹不一致

同时检查 `demo/forest_map_publisher.py` 和 `demo/swarm_formation_demo.sdf`。树坐标、实体半径和点云膨胀设置必须保持同步。

### 9.5 视频能被 OpenCV 打开，但系统播放器无法播放

不要直接分发 `mp4v` 中间文件。最终文件必须经过 `encode_compatible_mp4.sh` 转码，并满足：

```text
codec   = H.264/AVC
profile = Constrained Baseline
pix_fmt = yuv420p
fourcc  = avc1
moov    = faststart
```

### 9.6 PX4 工作树占用大量磁盘

多机长期运行会在 `build/px4_sitl_default/rootfs/0…N/log` 中产生大量 ULog。复制或归档源码时应排除各实例的 `log` 目录。删除日志前先确认目标路径位于对应 PX4 工程内。

## 10. 主要文件

CAT-ORA：

- `catora-px4-gazebo/deployment/stage1/`：上游算法镜像和服务验收。
- `catora-px4-gazebo/deployment/stage2/`：16 机 MAVROS 执行器、Compose 和闭环验收。
- `catora-px4-gazebo/deployment/px4/run_multi_px4.sh`：16 个 PX4/Gazebo 实例和端口分配。
- `demo/catora_demo.sdf`：CAT-ORA 录制场景。
- `demo/run_catora_demo.sh`：最终 demo 入口。

Swarm-Formation：

- `swarm-formation-px4-gazebo/deployment/stage1/`：上游算法复现。
- `swarm-formation-px4-gazebo/deployment/stage2/swarm_formation_px4.patch`：关闭理想后端并适配预设编队航点。
- `swarm-formation-px4-gazebo/deployment/stage2/px4_backend_bridge/`：planner 与 MAVROS 的双向桥。
- `demo/forest_map_publisher.py`：确定性树林点云。
- `demo/swarm_formation_demo.sdf`：与点云一致的 Gazebo 圆柱和相机。
- `demo/swarm_forest_mission.py`：任务触发、带墙钟时间的七机实际轨迹记录和指标计算。
- `demo/encode_video.py`：把当前实际机位投影为官方 12 边编队图，不添加圆点、机号或历史轨迹。
- `demo/run_swarm_demo.sh`：无录像验收和最终录制入口。

公共录制文件：

- `demo/capture_gz_images.cpp`
- `demo/encode_video.py`
- `demo/encode_compatible_mp4.sh`
- `demo/verify_videos.py`

## 11. 最短复验流程

镜像和 PX4 SITL 已经构建时，只需在 RKOH 串行执行：

```bash
cd /home/noah/code/UAV

./demo/run_catora_demo.sh
SWARM_RECORD=0 ./demo/run_swarm_demo.sh
./demo/run_swarm_demo.sh

docker run --rm \
  -v /home/noah/code/UAV/demo:/demo \
  --entrypoint python3 uav/catora:noetic-px4 \
  /demo/verify_videos.py \
  /demo/catora_px4_gazebo_headless.mp4 \
  /demo/swarm_formation_px4_gazebo_headless.mp4
```

若三个命令都通过，说明 CAT-ORA 16 机重构、Swarm 七机随机树林穿越、视频编码和播放器兼容性均已复现。
