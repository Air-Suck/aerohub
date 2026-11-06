> 历史快照：2026-11-03 合并前版本，仅用于追溯。当前说明统一维护在[第一阶段接口与配置设计](../../Aerostack2统一Planner第一阶段接口与配置设计.md)，快照中的旧结论和命令不作为当前使用依据。

# Aerostack2 统一 Planner 使用指南

## 1. 当前可用能力

`aerostack2-integration` 已将 CAT-ORA 和 Swarm-Formation 接入同一套 Planner 运行框架。一次任务只运行一个 Planner，通过任务 YAML 选择算法。

两套算法共用：

- `START / PAUSE / RESUME / CANCEL / GET_STATUS` 生命周期；
- `TimedTrajectorySegment` 轨迹接口；
- Output Gateway；
- Aerostack2、PX4 v1.17.0 SITL 和 Gazebo Harmonic 执行链路；
- 任务完成、超时和失败后的悬停处理。

第一阶段用于仿真验证。它还不包含 Planner 热切换、自动选优、远程 Model Hub、真机和国产操作系统适配。

当前 `START / PAUSE / RESUME / CANCEL / GET_STATUS` 是项目自建的 Planner 生命周期服务，不是 Aerostack2 原生 `BehaviorServer<ActionT>`。Aerostack2 负责逐机 platform、状态估计、motion controller、运动 behavior 组件和 `motion_reference` 执行。后续若改成原生 AS2 Behavior，可保留 Planner adapter 和轨迹契约，在 framework 外增加 Action Goal/Feedback/Result 及 AS2 生命周期回调映射。

## 2. 运行环境

在 Linux 仿真机 RKOH 上运行，工程目录为：

```text
/home/noah/code/UAV/aerostack2-integration
```

可以从开发机登录：

```bash
ssh RKOH_Remote
cd /home/noah/code/UAV/aerostack2-integration
```

完整部署依赖以下本地镜像：

```text
uav/aerostack2-px4:humble-v1.17
uav/px4-gazebo:v1.17.0-harmonic
uav/catora:noetic-upstream
uav/swarm-formation:noetic-px4
```

其中 PX4/Gazebo、CAT-ORA 和 Swarm-Formation 基础镜像来自两个原始复现工程。首次部署前应先按以下文档构建：

- `catora-px4-gazebo/README.md`
- `swarm-formation-px4-gazebo/README.md`

## 3. 从零准备和第一次构建

全新环境必须先拉取平台镜像需要的三个锁版仓库：

```bash
cd /home/noah/code/UAV/aerostack2-integration
bash ./deployment/prepare_sources.sh
```

该脚本只依赖 Bash 和 Git，不要求宿主安装 ROS 2 或 `vcstool`。它会准备并校验：

| 源码 | Commit |
| --- | --- |
| Aerostack2 | `19c397309c6a41b279e6b551818ec12605cfcecc` |
| `as2_platform_pixhawk` | `fcba062219935acef69dfa009c8be376c707ee8a` |
| `px4_msgs` | `86d8239e962f6939e05c3737784f60c02fa884db` |

需要代理时，先设置：

```bash
export HTTP_PROXY=http://127.0.0.1:7890
export HTTPS_PROXY=http://127.0.0.1:7890
git config --global http.proxy "$HTTP_PROXY"
git config --global https.proxy "$HTTPS_PROXY"
```

源码准备成功后，可以分别构建镜像：

在集成工程目录执行：

```bash
docker build -f deployment/platform/Dockerfile \
  -t uav/aerostack2-px4:humble-v1.17 .

docker compose -f deployment/integration/compose.yaml build
```

也可以从空源码目录一次完成上述流程：

```bash
bash ./deployment/build_from_zero.sh
```

需要排除旧 Docker 缓存影响时使用：

```bash
NO_CACHE=1 bash ./deployment/build_from_zero.sh
```

第二条命令会构建统一运行时和两个算法的集成镜像。运行时镜像构建期间会编译五个 ROS 2 包，并运行 32 项测试。

`docker build` 会再次读取三个仓库的 Git commit。目录缺失或版本不一致时构建会直接失败，不会生成版本不明的镜像。

如果 PX4 源码或仿真缓存不在 Compose 的默认相对路径，可显式指定：

```bash
export PX4_SOURCE_DIR=/absolute/path/to/PX4-Autopilot
export PX4_SIM_CACHE_DIR=/absolute/path/to/.px4-sim-cache
docker compose -f deployment/integration/compose.yaml up -d --no-build
```

## 4. 启动仿真

### 4.1 一键启动并验收

已有全部镜像后，推荐使用：

```bash
bash ./deployment/integration/run_planner.sh cat_ora
```

或：

```bash
bash ./deployment/integration/run_planner.sh swarm_formation
```

脚本会依次完成：

1. 停止本工程上一次运行留下的容器；
2. 启动六个 Compose 服务；
3. 等待 `/planner/control`；
4. 顺序完成七机 Arm、Offboard 和 Takeoff；
5. 启动所选 Planner；
6. 轮询任务，直到 `SUCCEEDED/NONE` 或失败；
7. 失败时输出六个容器的末尾日志。

两个 Planner 的验收应串行执行。脚本每次都会重置仿真，避免上一条轨迹和当前位置影响下一次结果。

### 4.2 手动启动

```bash
docker compose -f deployment/integration/compose.yaml up -d --no-build
docker compose -f deployment/integration/compose.yaml ps
```

正常情况下应看到六个服务处于 `Up`：

```text
planner-as2-seven
planner-px4-seven
planner-clock
planner-runtime
planner-cat-ora
planner-swarm-formation
```

检查 ROS 2 仿真时钟：

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  ros2 topic hz /clock
'
```

正常频率约为 100 Hz。按 `Ctrl+C` 结束频率检查，不会停止仿真。

## 5. 七机起飞

两个示例都使用七机仿真环境。等待 PX4、AS2 和定位数据就绪后执行：

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  exec fleet_takeoff.py \
    --vehicles drone0 drone1 drone2 drone3 drone4 drone5 drone6 \
    --height 1.5 --speed 0.5 --timeout 90
'
```

脚本会等待所有车辆定位就绪，然后逐机执行 Arm、Offboard 和 Takeoff。七机仿真负载较高，不要改为并发起飞。

## 6. 运行 CAT-ORA

CAT-ORA 示例规划 `drone0` 至 `drone3` 的四机任务。其余三架无人机保持当前位置。

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  ros2 service call /planner/control \
    uav_planner_interfaces/srv/ControlPlanner \
    "{command: 0, task_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/cat_ora_four_px4.yaml, deployment_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/deployment_four_px4.yaml}"
'
```

服务接受请求后，返回的 `task_id` 用于区分本次任务。任务运行结束后，`overall_status` 应变为 `3`，即 `SUCCEEDED`。

CAT-ORA 原算法允许重新分配目标，但当前公共任务契约采用固定的逐机目标。若算法输出的车辆与目标对应关系发生变化，adapter 会返回 `FAILED/NO_SOLUTION`，不会执行换配后的轨迹。

## 7. 运行 Swarm-Formation

不要在 CAT-ORA 执行后的同一场仿真中直接开始验收。先重置全部服务，再重新执行第 5 节的七机起飞：

```bash
docker compose -f deployment/integration/compose.yaml down
docker compose -f deployment/integration/compose.yaml up -d --no-build
```

起飞完成后启动七机 Swarm-Formation 任务：

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  ros2 service call /planner/control \
    uav_planner_interfaces/srv/ControlPlanner \
    "{command: 0, task_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/swarm_formation_seven_px4.yaml, deployment_config_path: /opt/uav_planner_ws/install/share/uav_planner_framework/examples/deployment_seven_px4.yaml}"
'
```

Swarm-Formation adapter 会把滚动 `PolyTraj` 转换为短时 `TimedTrajectorySegment`，并校验轨迹是否包含完整机群、时间是否一致、首点是否连续。

## 8. 查询和控制任务

控制接口为：

```text
/planner/control
```

命令值如下：

| 命令 | 数值 | 用途 |
| --- | ---: | --- |
| `START` | 0 | 读取任务和部署配置，启动所选 Planner |
| `PAUSE` | 1 | 丢弃旧轨迹、悬停并停止累计任务运行时间 |
| `RESUME` | 2 | 使用原 `task_id`，从最新实际状态重新规划 |
| `CANCEL` | 3 | 取消任务并悬停；取消后的任务不能恢复 |
| `GET_STATUS` | 4 | 查询当前任务状态 |

查询状态：

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  ros2 service call /planner/control \
    uav_planner_interfaces/srv/ControlPlanner "{command: 4}"
'
```

暂停、恢复和取消只需替换 `command`：

```bash
# PAUSE
ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner "{command: 1}"

# RESUME
ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner "{command: 2}"

# CANCEL
ros2 service call /planner/control uav_planner_interfaces/srv/ControlPlanner "{command: 3}"
```

以上三条命令需要在已加载 ROS 2、Aerostack2 和集成工作空间的 `planner-runtime` 容器中执行。

状态值如下：

| 状态 | 数值 | 含义 |
| --- | ---: | --- |
| `REJECTED` | 0 | 启动前校验失败，或尚未启动任务 |
| `RUNNING` | 1 | 正在运行 |
| `PAUSED` | 2 | 已暂停，可以恢复 |
| `SUCCEEDED` | 3 | 全部无人机到达目标并持续满足条件 1 秒 |
| `FAILED` | 4 | Planner、轨迹或超时错误 |
| `CANCELED` | 5 | 已取消 |

常见 `failure_reason` 包括 `INVALID_TASK`、`NO_SOLUTION`、`TRAJECTORY_TIMEOUT`、`TASK_TIMEOUT`、`INVALID_TRAJECTORY_SET`、`PLANNER_ERROR` 和 `OUTPUT_ADAPTER_ERROR`。

## 9. 修改任务配置

任务样例位于：

```text
planner_framework/examples/cat_ora_four_px4.yaml
planner_framework/examples/swarm_formation_seven_px4.yaml
```

任务文件的最小结构为：

```yaml
task:
  name: example_task
planner:
  id: cat_ora
  version: 4bc41f593eebde65dd74108a97e80430a145dfa0
  parameters: {}
vehicles: [drone0]
targets:
  drone0: {x: 0.0, y: -4.0, z: -1.5}
completion:
  position_tolerance: 0.5
  hold_duration: 1.0
  task_timeout: 120.0
```

坐标完全沿用 PX4 `LOCAL_NED`：

- `x` 指向北；
- `y` 指向东；
- `z` 向下为正，因此飞行高度 1.5 m 通常写为 `z: -1.5`；
- yaw 使用弧度，`0` 指北，`π/2` 指东。

`planner.parameters` 可以省略。省略的参数来自对应 Planner 的默认模板，任务文件只需填写需要覆盖的项目。映射递归合并，数组整体替换，未知参数会使任务以 `INVALID_TASK` 被拒绝。

Planner 的固定版本为：

| Planner | `id` | `version` |
| --- | --- | --- |
| CAT-ORA | `cat_ora` | `4bc41f593eebde65dd74108a97e80430a145dfa0` |
| Swarm-Formation | `swarm_formation` | `967a4bdfae949e994691f8ffc87dbb0147cbebb7` |

修改源码目录中的任务样例后，需要重新构建 runtime 镜像，安装目录中的副本才会更新。临时任务也可以挂载进容器，并把 `task_config_path` 指向容器内的绝对路径。

## 10. 部署配置

部署配置记录每架无人机的 PX4 局部原点、Aerostack2 namespace 和定位 topic：

```yaml
drone0:
  px4_origin_in_fleet_ned: [-4.5, -12.0, 0.0]
  namespace: /drone0
  topics:
    pose: /drone0/self_localization/pose
```

任务中的每架无人机都必须出现在部署配置中。`px4_origin_in_fleet_ned` 用于机群共享 NED 坐标与逐机 PX4 局部 NED 坐标之间的转换。更改 Gazebo 出生位置时，必须同步修改该值。

## 11. 结果、配置和日志

每次成功启动任务后，框架都会生成一份 resolved config，把默认参数、任务覆盖、部署信息、唯一 `task_id` 和仿真时钟配置固化下来。

宿主机目录：

```text
artifacts/resolved/
```

容器内目录：

```text
/var/lib/uav-planner/resolved/
```

每个任务通常生成：

```text
<task_id>.yaml
<task_id>.planner.log
<task_id>.output.log
```

查看运行时日志：

```bash
docker logs --tail 200 planner-runtime
docker logs --tail 200 planner-cat-ora
docker logs --tail 200 planner-swarm-formation
docker logs --tail 200 planner-as2-seven
docker logs --tail 200 planner-px4-seven
```

查看当前任务发布的状态：

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  ros2 topic echo /planner/status
'
```

## 12. 失败时的行为

出现暂停、取消、Planner 错误、轨迹超时或任务超时时，框架会：

1. 丢弃尚未执行的旧轨迹；
2. 向每架无人机发布当前实际位置的悬停参考；
3. 停止 Planner adapter；
4. 更新任务状态和失败原因。

框架不会自动降落或解除武装。当前悬停处理是 Planner framework 内的安全动作，还不是独立的 Aerostack2 Hover Behavior。

## 13. 停止和重置

停止全部服务：

```bash
docker compose -f deployment/integration/compose.yaml down
```

完整重置仿真：

```bash
docker compose -f deployment/integration/compose.yaml down
docker compose -f deployment/integration/compose.yaml up -d --no-build
```

如修改了运行时代码、Planner manifest 或安装进镜像的任务样例，执行：

```bash
docker compose -f deployment/integration/compose.yaml build runtime clock
docker compose -f deployment/integration/compose.yaml up -d --no-build --force-recreate runtime clock
```

## 14. 常见问题

### 服务已启动，但 ROS 2 topic 只有发现信息，没有数据

确认相关容器都设置：

```text
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ROS_DOMAIN_ID=80
```

本工程使用 host network，但不同容器的 private IPC 不能共享 Fast DDS 共享内存通道。

### 七机起飞随机卡住

使用 `fleet_takeoff.py` 的顺序起飞流程，不要并发调用七个 Takeoff action。

### 任务启动后立即 `TRAJECTORY_TIMEOUT`

先检查 `<task_id>.planner.log`，再检查 `/clock` 是否持续发布。CAT-ORA 的首次轨迹生成时间较长，四机示例已把 `trajectory_timeout` 覆盖为 30 秒。

### Swarm-Formation 返回 `INVALID_TRAJECTORY_SET`

检查 Planner 日志中的机群完整性、轨迹长度和首点连续性错误。当前 `max_start_distance` 默认是 1.5 m，用于拒绝与实际位置不连续的滚动轨迹。

### 任务没有成功，但无人机已经接近目标

成功条件不是轨迹结束，而是所有无人机同时进入三维位置容差，并连续保持 1 秒。任意无人机离开容差后，保持时间重新累计。

## 15. 已知边界

- 当前 ROS 地图生成器使用 `obs_num=0`，验收没有证明虚拟点云障碍物与 Gazebo 碰撞体一致。
- 同一时间只能激活一个 Planner。
- 失败后只悬停，不会自动 Land 或 Disarm。
- 第一阶段没有独立 Mission Agent、Model Hub 服务或 Planner 自动选择功能。
- 当前结果只覆盖 PX4 SITL 和 Gazebo，不等同于真机验收。

设计定义见 `docs/Aerostack2统一Planner第一阶段接口与配置设计.md`。最终测试与冷启动证据见 `aerostack2-integration/artifacts/resolved/FINAL_VERIFICATION.md`。
