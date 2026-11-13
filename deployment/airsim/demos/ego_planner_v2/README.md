# EGO：PC 单／双机与 RoboPi 单机

本目录只交付 PC 单／双机和 RoboPi 单机。混合 RoboPi Drone0＋PC Drone1 的监管入口
不在本 PR。历史飞行与本次整理检查分别见 [VALIDATION.md](VALIDATION.md)。

## 固定环境与构建

使用已有镜像，不解包或修改镜像：

- onboard：`ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9`
- flight-board：`ghcr.io/popc0rn7/aerohub/flight-board@sha256:1eff3190ca7870d0267ddb1531365dd7c68d85c6e52ade3d25b17b84b4758843`
- OHOS builder、SDK/目标依赖准备遵循 [oh-ros-build](../../../oh-ros-build/README.md)。

Windows 示例，仓库根目录执行；工作区由操作者选择，不能与 Swarm 共用：

```powershell
$env:EGO_PC_WORK = 'E:\aerohub-build\ego-pr1-pc'
python deployment/airsim/demos/ego_planner_v2/run.py build --workspace $env:EGO_PC_WORK
```

`build_pc.sh` 在 `/work` 下分别构建原生包、公共接口/FollowTimedPath、framework/Adapter。
实际源码哈希保存在工作区 `source-manifest.json`。不需要旧迁移工作区或个人 Docker volume。
原生包只有六个；不要对整个 AeroHub 的 `src/` 执行不加选择的 colcon 构建。

## PC 仿真

操作者先启动适配的 AirSim UE 场景并 Play；入口不覆盖 UE 工程或替用户重置飞机。
`pc/settings.snapshot.json` 是经验证的双机场景契约，起点 NED `[0,0,0]`/`[0,-4,0]`，
PX4 ID135/136、TCP4560/4561、RPC41451。场景资源不包含在本仓库。

```powershell
$env:EGO_AIRSIM_HOST = '<Docker容器可达的AirSim宿主IPv4>'
python deployment/airsim/demos/ego_planner_v2/run.py pc-start --workspace $env:EGO_PC_WORK
deployment/airsim/demos/ego_planner_v2/pc/verify_ground.ps1 -Stage as2 -RunId ground-new
deployment/airsim/demos/ego_planner_v2/pc/run_flight.ps1 -Stage basic -RunId basic-new
deployment/airsim/demos/ego_planner_v2/pc/run_flight.ps1 -Stage ego -RunId ego-new -BasicRunId basic-new
# 双机路线前先取得本轮 Drone1 基础起降记录
deployment/airsim/demos/ego_planner_v2/pc/run_dual_flight.ps1 -Stage basic1 -RunId basic1-new
deployment/airsim/demos/ego_planner_v2/pc/run_dual_flight.ps1 -Stage crossing -RunId crossing-new -BasicRunId basic1-new
```

上述 flight 命令会实际发送仿真起降/运动指令。两机起点、净空、地面状态不满足时须停止
准备，不能修改门槛。历史 PC Demo 使用集中式兼容 Adapter；新的每机 API 由单机板端
及隔离真实 Adapter/AS2 回归覆盖，不把历史集中式双机飞行称为每机双机飞行。

故障入口 `pc/run_fault_flight.ps1 -Case cancel|planner_exit|cloud|odom -Fleet single|dual
-RunId <new>`；双机历史故障只注入 Drone0 并按集中式契约停止两机。
独立审计用 `run_dual_audit.ps1`、`run_crossing_audit.ps1`、`run_fault_audit.ps1`。
审计与飞行串行，实际停止/降落/disarmed 后才回收任务和计算容器；`run.py pc-stop`
还会检查两机 PX4 Disarmed 及无运行中的任务容器。失败时保留证据和恢复所需服务。

## RoboPi 单机

API18 / AArch64 / ROS2 Humble / Python3.12，目标 SOABI `cpython-312-aarch64-linux-ohos`。
先按共享 prepare.sh 创建独立、名称含 ego 的 OHOS_WORK_ROOT，SDK 和目标依赖用隔离副本，
依赖归档保留符号链接。不能指向 Swarm 工作区；准备脚本所需离线源码/SDK由操作者提供。

```bash
# 已按 oh-ros-build 文档准备环境变量与工作区；原系统/依赖导出保持只读。
python3 tools/robopi/relocate_ego_target_metadata.py "$OHOS_WORK_ROOT"
bash deployment/oh-ros-build/run.sh check
# 首次创建工作区后，准备锁定 AS2/Pixhawk 源码并构建必要运行包。
python3 tools/robopi/prepare_ohos_sources.py
bash deployment/oh-ros-build/build_as2.sh runtime
bash tools/robopi/build_lib.sh ego-planner-v2
bash tools/robopi/deploy_lib.sh --component as2 --include-ego   --as2-root /data/aerohub/validation/ego-v2-pr1 --prepare-only /absolute/path/ego-pr1.tar.gz
# 核对产物后去掉 --prepare-only 才会通过 HDC 安装；不传 --activate-config。
```

运行包仅含算法/Adapter/AS2/导航及必要接口，故障任务和审计不会被装入 install。
新候选位于 `/data/aerohub/validation/ego-v2-pr1`；旧已验收候选和系统 ROS 保留。
安装后在目标端加载该候选 board_env.sh，并运行 `verify_scoped_load.py <候选根>`；
需另行传入该检查脚本，它不会进入生产 install。
`prepare-only` 的打包检查不是新前缀板端飞行验收。

复制 `robopi/config.scoped.example.json` 到本地配置，填当前 HDC target、IPv4、domain；
`vehicle=drone0`。PC客户端加载上面构建的 EGO_PC_WORK。顺序执行：

```text
python deployment/airsim/demos/ego_planner_v2/robopi/run.py prepare --config <local.json> --output <new-output>
python deployment/airsim/demos/ego_planner_v2/robopi/run.py deploy --output <new-output>
python deployment/airsim/demos/ego_planner_v2/robopi/run.py pc-start --output <new-output>
python deployment/airsim/demos/ego_planner_v2/robopi/run.py start --output <new-output>
python deployment/airsim/demos/ego_planner_v2/robopi/run.py check --output <new-output>
python deployment/airsim/demos/ego_planner_v2/robopi/observe_ground.py --output <new-output> --duration 40
```

`deploy` 在已安装候选中放置本轮 session，并不代替前面的原生库安装。地面通过后显式调用
`mission-basic`、`mission-ego --baseline <new-output/mission-basic>`；原生降落且解除解锁后
依次 `stop`、`collect`、对应离线 audit、`pc-stop`。异常恢复专用入口
`robopi/run_fault_case.py --config <含fault_case的local.json> --output <new-output>`。
墙钟偏差含RTT上界≤0.5s；只有获得设备对时授权后才能使用 check_clock.py 的显式同步选项。
本入口不自动改网络、防火墙或 Docker Desktop 设置。

Windows Docker 使用 `udp_mapped`：每个参与者独立映射UDP端口并通告宿主可达地址；
PX4使用命名配置 `UXRCE_DDS_PTCFG=2`。HDC在线或ping通过不能替代真实DDS检查。
实验TCP路径保留为诊断历史，mission明确拒绝该配置。当前只支持 Drone0 单机。

## 不启动飞行的复验

宿主：`python deployment/airsim/demos/ego_planner_v2/robopi/test_offline.py`。
固定镜像中挂载仓库 `/input:ro`、外部构建 `/work:ro`、新证据目录可写，网络 none：
`bash /input/deployment/airsim/demos/ego_planner_v2/tests/verify_terminal_hold.sh <output> unit`
或 `replay`/完整矩阵；后两者启动真实 ROS/AS2，控制器与传感器仍是测试夹具。
这些检查不能替代 AirSim/PX4 动力学验收。

已知边界：历史验收没有证明长期退出可靠性；后续混合场景曾暴露 AS2 估计器退出竞态
及服务停止顺序问题，其修复与混合回归另行交付。本 PR 不包含未完成的修复。
