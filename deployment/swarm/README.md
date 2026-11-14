# Swarm Infra 使用说明

`deployment/swarm/` 是与具体算法无关的多无人机部署基础设施，负责统一生成机群拓扑、监管 PC UAV 进程，并为 AirSim 提供实时机群质心顶视图。编队、规划、控制和任务 Agent 仍由具体应用实现。

## 1. 部署模型

一份 YAML 配置是车辆数量和部署阶段的权威来源。基础设施明确区分两个阶段：

```text
pc_simulation（前期）   N 个 onboard-computer Docker，每机一个
hybrid_board（上板期）  1 个真实算力板 + N-1 个 onboard-computer Docker
工作站                  1 个 ground station / task client
AirSim                  N 个由同一配置生成的车辆
```

- `deployment.stage` 选择 `pc_simulation` 或 `hybrid_board`；默认模板使用前者。
- `fleet.vehicle_count` 决定机群规模。
- `pc_simulation` 中全部 UAV 各自运行一个独立的 onboard-computer 容器，由独立线程监管。
- `hybrid_board` 中 `fleet.board_vehicle_index` 选择唯一运行在真实算力板上的 UAV，其余 UAV 保持独立 onboard-computer 容器。
- `board_center` 与 `board_worker` 仅属于上板期双角色验收；前期集中式拓扑使用 `onboard_center`。
- 地面站和外部计算固定在 PC。PC 上的远程 launcher 不改变板端进程的归属。

## 2. 目录

| 路径 | 用途 |
| --- | --- |
| `config.example.yaml` | 通用编排配置模板 |
| `configs/swarm_open_field.yaml` | Swarm-Formation AirSim Demo 配置 |
| `topology.py` | 校验配置、生成拓扑和 AirSim settings |
| `orchestrator.py` | 通用进程编排，不包含真实飞行逻辑 |
| `view.py`、`overhead_view.py` | 初始顶视配置和实时质心相机跟踪 |
| `tests/` | 无需 RoboPi、Docker 或 AirSim 的单元测试 |

## 3. 主机依赖

主机 Python 需要 PyYAML 和 msgpack：

```bash
sudo apt-get install python3-yaml python3-msgpack
```

无法使用系统包时：

```bash
python3 -m pip install --user -r deployment/swarm/requirements.txt
```

AirSim 验收使用有屏模式：

```bash
export DISPLAY=:3
export AIRSIM_RENDER_MODE=windowed
```

真实 RoboPi Demo 还需要项目 `.env` 中已有的 HDC、`ROBOPI_IP`、`PC_IP` 和 `FLIGHT_BOARD_IMAGE` 配置。地址和认证信息不得写入共享配置。

## 4. 配置

复制 `config.example.yaml` 创建新应用配置，并至少设置：

```yaml
fleet:
  vehicle_count: 3
  coordinator_vehicle_index: 0
  initial_spacing_m: 4.0

deployment:
  stage: pc_simulation

coordination:
  mode: centralized
  centralized_cases: [onboard_center]

ground_station:
  required: true
  placement: pc
```

`nodes.onboard_computer.image` 必须指向项目规定的共享 onboard-computer 镜像；算法和 Adapter 通过宿主工作区 overlay 挂载并显式 source，不为每架无人机构建算法专用镜像。`nodes.onboard_computer.launch_command` 与 `ground_station.launch_command` 供通用编排器使用，可引用：

```text
{vehicle} {vehicle_index} {airsim_vehicle} {center_vehicle}
{scenario} {vehicle_count} {onboard_computer_image} {output}
```

`airsim.ports` 的 base 值按车辆索引递增。不同应用必须避免与已有 AirSim、PX4、XRCE Agent 和 ROS Domain 冲突。

先执行只读检查：

```bash
python3 -m deployment.swarm.topology validate \
  --config deployment/swarm/configs/swarm_open_field.yaml

python3 -m deployment.swarm.topology plan \
  --config deployment/swarm/configs/swarm_open_field.yaml

python3 -m deployment.swarm.topology render-settings \
  --config deployment/swarm/configs/swarm_open_field.yaml \
  --output log/swarm-infra/settings.json
```

默认 `plan` 必须显示 `board_vehicle=null`、N 个 `onboard_computer_vehicles` 和位于 PC 的 ground station。切换为 `hybrid_board` 后，必须显示恰好一个 `board_vehicle`、N-1 个 `onboard_computer_vehicles` 及两个板端角色场景。

## 5. 通用进程编排

该入口适合开发新应用时检查部署命令和进程监管，不启动 PX4/AS2 飞行闭环：

```bash
python3 -m deployment.swarm.orchestrator run \
  --config deployment/swarm/config.example.yaml \
  --scenario onboard_center \
  --output log/swarm-infra/onboard-center

python3 -m deployment.swarm.orchestrator run-matrix \
  --config deployment/swarm/config.example.yaml \
  --output log/swarm-infra/matrix
```

`report.json` 和 `matrix-report.json` 证明启动拓扑、线程/进程独立性及冒烟窗口，不证明算法效果或飞行动力学。

## 6. Swarm-Formation 上板期 Demo

真实入口位于 `deployment/airsim/demos/swarm_open_field/`，使用 AirSim + PX4 SITL + AS2。该入口显式使用 `hybrid_board` 配置：一个 UAV compute node 部署到真实算力板，其余节点各自在 PC 的 onboard-computer Docker 中运行。前期纯 PC 验证必须先使用 `pc_simulation`，不能把上板期拓扑当作默认开发拓扑。

### 6.1 构建

```bash
docker build --network none --pull=false \
  -f deployment/airsim/demos/swarm_open_field/Dockerfile \
  -t uav/swarm-hybrid-compute:local .

bash tools/robopi/build_lib.sh swarm-formation
```

第一条命令构建 PC compute 镜像；第二条交叉编译 RoboPi overlay。运行入口会部署当前 overlay，但不会替代缺失的 `.env` 或镜像。

### 6.2 预飞

预飞不解锁、不发运动目标，检查真实传感器、地图健康、DDS 端点、PC 独立 worker 和两种中心位置：

```bash
DISPLAY=:3 AIRSIM_RENDER_MODE=windowed \
python3 deployment/airsim/demos/swarm_open_field/run_role_matrix.py \
  --preflight-only \
  --output log/swarm-infra/role-preflight-$(date +%Y%m%d-%H%M%S)
```

### 6.3 完整双角色飞行

```bash
DISPLAY=:3 AIRSIM_RENDER_MODE=windowed \
python3 deployment/airsim/demos/swarm_open_field/run_role_matrix.py \
  --output log/swarm-infra/role-flight-$(date +%Y%m%d-%H%M%S)
```

成功条件：

- `role-matrix.json` 中 `passed=true`；
- `board_center` 和 `board_worker` 返回码均为 0；
- 两个 `result.json` 均为 `planner_status=SUCCEEDED`；
- 所有 UAV `landed_and_disarmed=true`；
- `pc-worker-topology.json` 中线程 ID 和 Docker PID 各不相同；
- `pc-worker-final.json` 无错误、无线程或容器残留；
- `board-cleanup.json` 的 `remaining` 为空。

单独运行 `run_robopi.py --coordination-scenario ...` 只用于调试，不能代替双角色验收。

### 6.4 故障矩阵

故障入口在无人机起飞且真实轨迹已经送达后注入进程故障：

```bash
DISPLAY=:3 AIRSIM_RENDER_MODE=windowed \
python3 deployment/airsim/demos/swarm_open_field/run_fault_matrix.py \
  --output log/swarm-infra/fault-matrix-$(date +%Y%m%d-%H%M%S)
```

| 场景 | 拓扑 | 注入 |
| --- | --- | --- |
| `worker_planner_exit` | `board_center` | 停止一台 PC worker 的原生 planner |
| `center_proxy_exit` | `board_worker` | 停止 PC 中心机的协调 proxy |

成功要求 `fault-matrix.json` 的 `passed=true`，并且两个结果都满足 Planner 明确进入 `FAILED`、所有 AS2 控制器确认 HOVER、所有 UAV 降落解除武装、清理无残留。

这里只停止算法/协调进程，保留 AS2、PX4 和通信安全链以执行受控降落。整台 compute 主机或整个容器突然失联无法再由该节点上的 AS2 执行 ROS 降落，必须单独配置和验证 PX4 数据链丢失 failsafe，不能用本矩阵替代。

## 7. 调整机群数量

修改配置中的 `fleet.vehicle_count`，并通过 `--swarm-config` 传给入口：

```bash
python3 deployment/airsim/demos/swarm_open_field/run_robopi.py \
  --swarm-config /path/to/swarm.yaml \
  --coordination-scenario board_center \
  --output log/swarm-infra/custom-flight
```

共享拓扑可以生成任意正整数规模，但当前 Swarm-Formation 原生算法仅支持连续命名的 `drone0..droneN-1`，且 `N` 必须在 3 到 7 之间。调整数量时，前期会产生 N 个 onboard-computer 容器；上板期会产生 N-1 个容器和一个真实算力板节点。配置生成成功不代表某个算法支持该规模。

## 8. AirSim 顶视图

AirSim manager 会为单机和多机加入 `swarm_overhead` 外部相机：

- 初始位置在全部无人机初始坐标质心正上方；
- Pitch 为 -90 度；
- AirSim 启动后通过 MessagePack RPC 持续跟踪实时质心；
- swarm 场景设置 `primary_view: true`，主窗口使用首架 UAV 的 FPV 观察相机；跟踪器把世界坐标中的质心顶视位姿转换为相对该 UAV 的相机位姿，因此主画面随质心连续移动。外部相机同时更新，用于保存无 UI 遮挡的图像证据；
- 跟踪器将更新次数、最新质心、相机位置、最大质心位移和 RPC 错误写入 `overhead-view-state.json`；
- `manager.py status` 只有场景、跟踪进程和新鲜的结构化相机更新都正常时才返回成功，进程存活本身不足以通过。

验收不能只检查 settings JSON，应保留真实主窗口、1024×1024 外部相机图片、`overhead-view-state.json`、跟踪器日志和最终 `state.json`。动态飞行验收还必须证明 `update_count` 持续增加且 `max_centroid_displacement_m` 大于场景预声明阈值。

## 9. 测试与证据

不依赖 Docker、RoboPi 或 AirSim 的回归：

```bash
python3 -m unittest discover -s deployment/swarm/tests -p 'test_*.py' -v
python3 -m unittest deployment.airsim.demos.catora_open_field.tests.test_sim_startup -v
```

`log/` 被 Git 忽略。每次运行必须使用新输出目录，保留配置产物、镜像 ID、拓扑、线程/PID、任务结果、DDS audit、AirSim 图片、清理状态和原始日志。不得用单元测试、静态配置或旧证据替代本次真实验收。
