# Aerostack2 统一 Planner 集成

本工程把 CAT-ORA 和 Swarm-Formation 注册为同一类 `planner` 模块。任务配置每次选择一个 Planner，两者共用任务校验、项目内的 Planner 生命周期、`TimedTrajectorySegment`、Output Gateway、Aerostack2 和 PX4/Gazebo 执行链路。

当前生命周期由 `/planner/control` 服务实现，语义与 Behavior 一致，但还不是 `as2_behavior::BehaviorServer`。Aerostack2 在本工程中负责 Pixhawk platform、状态估计、运动控制和 `motion_reference` 执行。

第一阶段不包含 Planner 并行协作、在线热切换、自动选优、远程 Model Hub、真机或国产操作系统验收。

## 目录

- `planner_interfaces`：统一 ROS 2 消息与 `ControlPlanner` 服务
- `planner_framework`：Registry、配置解析、生命周期、完成判定和 Output Gateway
- `cat_ora_adapter`：CAT-ORA ROS 1 服务适配
- `swarm_formation_adapter`：Swarm-Formation ROS 1 滚动轨迹适配
- `as2_setpoint_adapter`：PX4 本地 NED setpoint 到 AS2 本地 ENU reference
- `deployment/integration/compose.yaml`：完整七机仿真部署
- `planner_framework/examples`：任务与机群部署配置
- `artifacts/resolved`：运行时生成的 resolved config 和进程日志

## 前置镜像

完整 Compose 使用以下本地镜像：

```text
uav/aerostack2-px4:humble-v1.17
uav/px4-gazebo:v1.17.0-harmonic
uav/catora:noetic-upstream
uav/swarm-formation:noetic-px4
```

全新环境可以用一个入口准备源码并构建平台、runtime 和两个 Planner 集成镜像：

```bash
bash ./deployment/build_from_zero.sh
```

完整无缓存复验使用 `NO_CACHE=1 bash ./deployment/build_from_zero.sh`。

PX4/Gazebo 和两个 ROS 1 算法基础镜像分别来自相邻的 `catora-px4-gazebo` 与 `swarm-formation-px4-gazebo` 复现工程。对应 README 记录了构建步骤。

## 从零准备上游源码

平台镜像使用三个锁定版本的上游仓库。若只需准备源码，执行：

```bash
cd aerostack2-integration
bash ./deployment/prepare_sources.sh
```

脚本只依赖 Bash 和 Git，会拉取并校验以下 commit：

```text
aerostack2             19c397309c6a41b279e6b551818ec12605cfcecc
as2_platform_pixhawk   fcba062219935acef69dfa009c8be376c707ee8a
px4_msgs               86d8239e962f6939e05c3737784f60c02fa884db
```

如果目录已存在，脚本只接受完全相同的 commit，避免构建时混入其他版本。需要代理时，在运行脚本前设置 `HTTP_PROXY`、`HTTPS_PROXY` 和 Git 的代理配置。

## 构建与启动

在 Linux 仿真机执行：

```bash
cd aerostack2-integration
docker compose -f deployment/integration/compose.yaml build
docker compose -f deployment/integration/compose.yaml up -d --no-build
docker compose -f deployment/integration/compose.yaml ps
```

应看到以下六个服务处于 `Up`：

```text
planner-as2-seven
planner-px4-seven
planner-clock
planner-runtime
planner-cat-ora
planner-swarm-formation
```

Compose 使用 host network、`ROS_DOMAIN_ID=80` 和 Fast DDS UDP。Gazebo 原生时钟经 `planner-clock` 发布为 ROS 2 `/clock`，AS2 与 Planner runtime 均使用仿真时间。

已有镜像时，可以用一条命令完成干净启动、七机顺序起飞、Planner 启动和终态验收：

```bash
bash ./deployment/integration/run_planner.sh cat_ora
bash ./deployment/integration/run_planner.sh swarm_formation
```

脚本每次会先停止本工程的旧容器，因此两个命令应串行执行。失败时会自动汇总六个容器的末尾日志。

## 七机起飞

等待 PX4、AS2 和定位 topic 就绪后执行：

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

脚本等待全部定位后，按机顺序执行 Arm、Offboard 和 Takeoff。七机高负载下不要并发发送 Takeoff action。

## 运行 CAT-ORA

CAT-ORA 示例使用 `drone0..drone3`。其余已起飞车辆继续由各自 AS2 控制器保持位置。

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

CAT-ORA 会优化目标分配，但公共任务契约采用固定逐机目标。adapter 会核对每条轨迹终点；若算法改变车辆与目标的对应关系，任务以 `FAILED/NO_SOLUTION` 结束，不会执行换配后的轨迹。

## 运行 Swarm-Formation

验收时先重置仿真并重新执行七机起飞，再启动七机任务：

```bash
docker compose -f deployment/integration/compose.yaml down
docker compose -f deployment/integration/compose.yaml up -d --no-build
```

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

Swarm adapter 会把滚动 `PolyTraj` 转成短时 `TimedTrajectorySegment`，并在发布前校验机群完整性和轨迹首点连续性。

当前 ROS 地图生成器使用 `obs_num=0`。这套验收覆盖真实 PX4/Gazebo 动力学和队形闭环，不代表虚拟点云障碍物已经与 Gazebo 碰撞体对齐。

## 状态与控制

查询当前任务：

```bash
docker exec planner-runtime bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/aerostack2_ws/install/setup.bash
  source /opt/uav_planner_ws/install/setup.bash
  ros2 service call /planner/control \
    uav_planner_interfaces/srv/ControlPlanner "{command: 4}"
'
```

命令值为 `START=0`、`PAUSE=1`、`RESUME=2`、`CANCEL=3`、`GET_STATUS=4`。状态值为 `REJECTED=0`、`RUNNING=1`、`PAUSED=2`、`SUCCEEDED=3`、`FAILED=4`、`CANCELED=5`。

暂停、取消、Planner 失败、轨迹超时和任务超时时，框架会丢弃旧轨迹并发布当前实际位置的机群悬停参考。失败后不会自动降落或解除武装。

## 测试

`deployment/runtime/Dockerfile` 在镜像构建阶段编译全部五个 ROS 2 包，并运行：

- Planner framework：22 项
- CAT-ORA adapter：3 项
- Swarm-Formation adapter：5 项
- AS2 setpoint adapter：2 项

单独运行依赖无关测试：

```bash
python3 -m unittest discover -s planner_framework/tests -v
```

设计与验收边界见 [接口与配置设计](docs/Aerostack2统一Planner第一阶段接口与配置设计.md)，最终构建和冷启动证据见 [最终验证](artifacts/resolved/FINAL_VERIFICATION.md)。

完整操作流程、任务配置说明和排障方法见 [使用指南](docs/Aerostack2统一Planner使用指南.md)。全部文档见 [文档导航](docs/README.md)。
