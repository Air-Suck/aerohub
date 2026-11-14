# ProjectAirSim 赛道单轮测试

使用已打包的 `BlocksPlannerCourse-linux-v7-wmm`、原版 flight_board PX4 和现有 AS2 执行层。
每次执行命令，停止上一轮已上锁的飞控，重启场景和 SITL，从起点飞一轮。
不在飞行中热换算法。启动器使用同一个文件锁，避免两个终端同时重置同一场景。

## 本机快速运行

在仓库根目录执行。YOPO 使用原生速度缩放，目标速度 5 m/s，不施加额外 2 m/s² 约束：

```bash
python3 deployment/airsim/demos/projectairsim_course/run_yopo.py
# 降速单轮，模型与执行端同步设定
python3 deployment/airsim/demos/projectairsim_course/run_yopo.py --speed 3
```

SUPER 保留 5 m/s、2 m/s² 配置：

```bash
python3 deployment/airsim/demos/projectairsim_course/run_super.py
```

历史本机入口 `log/projectairsim/manual-wmm5/run.py` 不随仓库分发；新部署使用上述正式入口。
每次命令只启动一轮，不自动重试。飞行中 Ctrl+C 请求结束任务并等待降落恢复。
若飞控仍解锁，入口拒绝重置。画面由运行者观察，录屏使用已有桌面录制工具。
启动、起飞、航线、停止、降落都需时间；端口释放时会明确打印等待信息。

YOPO、DiffPhys 只检查启动和输入、不解锁时，加 `--prepare-only`。这不是飞行成功证明。

## 共用与算法专属部分

| 项目 | 共用约定 |
| --- | --- |
| 场景 | 同一 120 m、24 根柱子的 Unreal 赛道；独立包提供几何与 HUD |
| 初始位姿 | 场景 NED [151.5, 0, -1.2896818542480468]，yaw 0 |
| 状态权威 | PX4 / AS2；真值不作为 Planner 控制输入 |
| 任务原点 | 每轮在地面测量；目标相对该原点 ENU [0, 120, 2]；DiffPhys 为 [0, 120, 3] |
| 起降 | 原生 Takeoff/Land Behavior，经 AS2 → PX4；不调用模拟器运动 API |
| 记录 | `log/projectairsim/` 下每轮独立时间戳目录，保留失败结果 |
| 本机通信 | ROS domain 188；原生 topic/service 9189/9190；HUD 9011 |
| PX4 | 现有 `aerohub-course-wmm-px4`，system 21 / instance 20，不修改固件或镜像 |

| 算法 | 平台输入 | 执行链 | 本赛道状态 |
| --- | --- | --- | --- |
| SUPER | LiDAR、PX4 odom、ROG 地图 | Adapter → FollowTimedPath | 已有真实飞行；5 m/s 一轮在约110.4 m后轨迹过期，未完成 |
| YOPO | 前视轴向深度、CameraInfo、PX4/AS2 pose/twist、真实相机TF | CUDA Inference → YOPO Planner → FollowTimedPath | front_center模板、标准Inference入口下，3m/s单轮到点、停车、Land/disarmed通过；非恒高验收 |
| IPC | 点云、odom、控制会话 | 既有 FollowReference external_acro → 角速度/推力 | 已接入；名义3m/s单轮到点、Land/disarmed通过，碰撞事件与完整控制映射验收未覆盖 |
| DiffPhys | 专属前视深度、状态、循环策略会话 | CUDA Inference → FollowReference attitude → 姿态/推力 | 标准Inference入口已实飞；5m/s、3m起飞后控制量超限退出，Land/disarmed通过；导航未通过 |

切换算法复用场景、SITL、任务原点、起降和证据格式；传感器、模型、Planner、执行接口和停止判据按算法契约保留。
YOPO 的原生到点定义是 `ARRIVED_5M` 后物理停止，不等同 SUPER 的 1.5 m 到点门槛。
实际速度分布与峰值须看记录；设定 5 m/s 不代表整段恒速或实测绝不超出 5。

## YOPO 输入与部署

`prepare_yopo_scene.py` 从本机 WMM 包读取机器人配置，并从匹配 ProjectAirSim 源码的
`robot_quadrotor_classic.jsonc` 复制原生 `front_center` 模板，生成独立 scene/robot/profile 到
`log/projectairsim/yopo-course-config/`；保留磁场覆盖，不改原归档。
相机为 `yopo_front`，保留上游机体前方0.46 m、pitch 0安装姿态；按YOPO输入契约修改为320×185、HFOV 90°，只启用浮点 DepthPlanar。
关闭本任务不消费的下视相机；保留 LiDAR 和飞控必需传感器。

标准输入为：

- `/drone0/sensor_measurements/yopo_front/depth_planar/image`
- `/drone0/sensor_measurements/yopo_front/depth_planar/camera_info`
- `/drone0/self_localization/pose`、`/drone0/self_localization/twist`
- `/tf`、`/tf_static`，光学系 `drone0/yopo_front_optical`

保留原图 stamp、轴向米制深度和真实光学 TF，不能用 DepthPerspective 改名代替。
Planner 检查相机几何、采样同步、新鲜度、推理服务和 Behavior 就绪，再允许任务开始。
YOPO worker 的 `YOPO_VELOCITY=5` 与 Planner 的 `max_speed: 5.0` 同时设置。
不修改训练权重，不将原生加速度缩放称作独立 2 m/s² 限制。

模型依赖按仓库标准 Inference 入口准备：初始化锁定的 `src/tool/navigation/planner/yopo/upstream`，
通过 `deployment/inference/setup_models.py` 或模型 `setup.sh` 准备本模型 `.venv` 和
`artifacts`。赛道入口复用 `deployment/inference/start_yopo.sh`，只设置本轮服务端口和
速度；不再接受赛道专有的源码、解释器或模型目录覆盖。模型启动校验标准源码文件哈希。
worker 使用 CUDA、18081 端口，每轮独立进程；按 PID 与启动时刻确认归属后回收。

ROS 包在固定 `uav/apps-super-forest:onboard-v0.1.1` 镜像中补编译，不重建共享镜像：

```bash
docker run --rm --network host -v "$PWD:/repo" --entrypoint bash \
  uav/apps-super-forest:onboard-v0.1.1 -lc \
  'source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && source /opt/uav_execution_ws/install/setup.bash && bash /repo/deployment/airsim/demos/yopo_single/build.sh'
```

运行明确加载本工作区的 `log/yopo-single/ros_ws/install` 和固定镜像的 AS2/FollowTimedPath。
`yopo_compute.py` 监管 AS2、Inference ROS Adapter 和 Planner；任务仅通过 ROS 调用。
本轮是 PC 共置测试，不表示 RoboPi 四方部署验收。

## 观察与结果

每轮打印证据目录。SUPER 保存 `flight.jsonl`、地图 bag 和 `result.json`；
YOPO 保存 `worker.log`、worker metadata、`mission.jsonl`、`plans.jsonl`、
起飞后的原始深度快照及 `result.json`，最终 PX4 状态单独保存。
任务成功、到点、停车和上锁分别看结果字段，不能仅看进程退出。
HUD 的 FINISHED 表示计时结束，失败也可能结束计时；不是到达终点标志。
目前航线计时从提交导航目标开始，不包含起飞阶段。

原生碰撞事件尚未桥接：本入口明确 `collision_coverage: unverified`。
运行者看到的画面可用于判断本轮现象，但缺少原生碰撞事件时不宣称自动无碰撞验收通过。
磁场修正已通过地面航向检查；各算法飞行结论独立记录。

停止后 HUD HTTP 端口可能保留 TIME_WAIT。入口按实际 bind 检查端口释放，最长等待90秒；
不因无 LISTEN 就认为可立即重启，也不杀未知占用进程。超时按具体端口检查归属。

旧入口及算法契约见 [YOPO](../yopo_single/README.md)、[IPC](../ipc_navigation/README.md)、
[DiffPhys](../diffphys_forest/README.md)。这些旧入口仍针对 AirSimNH，不能直接用于本包。

## IPC 名义3m/s赛道运行

```bash
python3 deployment/airsim/demos/projectairsim_course/run_ipc.py
```

每次从起点重启赛道和原版PX4。保持Chase主视角，IPC消费LiDAR而非前视相机。
`--prepare-only`只运行未解锁输入/模式与影子会话探针；其结果不代表空中求解通过。
默认影子探针在地面可能因起点占据拒绝路径，须保留结果。

沿用既有IPC平衡配置：名义巡航3、参考3.6、参考加速1m/s²、MPC逐轴速度4、
加速度3、jerk10、固定yaw、自由XYZ；目标相对本轮地面ENU [0,120,2]。
无独立高度越界终止，仍保留横向[-6,6]和前后[-2,123]m边界、0.6rad倾角终止。
点云原始输入`/drone0/sensor_measurements/lidar1/points`，NavigationInputs真实变换到earth；
IPC里程计及会话目标也为earth。任务中的odom目标显式经TF变换，不改frame冒充变换。
接管前读取PX4实际vehicle_attitude_setpoint悬停推力，不使用YOPO或旧飞行的油门。
原生Takeoff → 现有FollowReference external_acro → AS2角速度/推力链 → PX4；
原生Land并独立导出最终vehicle_status。

首次准备（复用固定 `uav/apps-super-forest:repo-native-20261109`，不重建镜像）：

```bash
mkdir -p log/ipc/native-deps
git clone --branch v0.6.2 --recurse-submodules https://github.com/osqp/osqp.git log/ipc/native-deps/osqp
git clone --branch v0.6.3 https://github.com/robotology/osqp-eigen.git log/ipc/native-deps/osqp-eigen
bash deployment/airsim/demos/ipc_control/build_overlay.sh
bash deployment/airsim/demos/ipc_navigation/build_core.sh
bash deployment/airsim/demos/ipc_navigation/build_node.sh
```

既有脚本校验完整锁定提交，overlay同时重编译匹配的controller/platform/FollowReference。
构建日志在log/ipc/；核芯5项数值测试、ROS构建5项测试和实际平台控制帧配对测试通过。
运行只读挂载本工作区overlay及native-deps，不混入其他工作区的插件二进制。

## DiffPhys 目标5m/s赛道入口

```bash
python3 deployment/airsim/demos/projectairsim_course/run_diffphys.py
```

运行前构建：`bash deployment/airsim/demos/diffphys_forest/build.sh`。
复用固定repo-native-20261109镜像与外部`log/diffphys/attitude-overlay`，该overlay包含
姿态命令源stamp/配对补丁、FollowReference及DiffPhys ROS接口，不混用IPC旧二进制。
模型依赖使用标准 `src/basic_function/inference/models/diffphys/{.venv,artifacts}` 与锁定算法子模块，
赛道入口复用 `deployment/inference/start_diffphys.sh`，显式选择 high_speed、CUDA和18082端口。
不再通过 DIFFPHYS_MODEL_DIR 改变模型或算法源码定位；两种profile均保留来源/权重哈希校验。

模型为官方high_speed，目标5m/s、10m目标前视、指令倾角锥0.5rad；真实姿态超过0.6rad
记失败。深度相机为640x480、HFOV91°、pitch20°、机体FRD [0.5,0,-0.1]，15Hz。
原生camera→标准ROS轴向米深度→82%中心裁剪/nearest48x36→CUDA逆深度/pool3，
保留原始标定/TF和采样stamp；主窗口独立Chase。该视觉任务关闭无消费者LiDAR。
ProjectAirSim标准深度发布为BEST_EFFORT，Planner显式`camera_reliable:=false`匹配；
CameraInfo采用RELIABLE/TRANSIENT_LOCAL匹配一次性标定，不能直接套旧相机的QoS。

原生Takeoff3m后，实测本轮PX4悬停油门；策略通过既有FollowReference ATTITUDE执行。
进入目标6m范围后取消策略，确认Action结束及POSITION模式，再原生GoTo2.5m/s收敛到
地面相对ENU [0,120,3]，误差≤1.5m、速度≤0.3m/s驻留2仿真秒，最后Land/disarmed。
目标速度是策略输入，不表示严格实测上限；独立记录峰值和实际分布。
横向/前后赛道边界保留，高度不设独立任务终止阈值。原生碰撞事件仍未桥接。

`--prepare-only`不解锁：真实深度/状态/几何、CUDA模型服务ready检查；悬停油门仅使用
明确标注的地面占位数，不发送飞行控制，不视作实际悬停标定或真实飞行通过。
已执行：实际平台ATTITUDE配对测试、5项解码对照、真实CUDA官方预处理及30帧循环
推理对照通过；飞行结果须查看各轮result.json，不从其他森林的历史结果继承通过。

DiffPhys 大图传输使用随入口提供的 `diffphys-dds.xml`：桥、标准接口、算法与任务均
显式加载 32 MiB Fast DDS SHM transport。默认传输在本主机出现大图秒级缺口；同输入
15 秒对照中，该配置将原始/标准图最大采样间隔恢复至 72 ms。不要靠增大控制超时
掩盖传输丢帧；只允许原生 Pair0 话题的一个消费者，诊断探针须与桥顺序运行。

## 最新验证与剩余工作

2026-11-14，标准源码/模型定位及统一启动脚本已完成CUDA数值对照、真实相机记录HTTP重放及两算法实飞：

| 算法/配置 | 实飞结果 | 证据目录（log/projectairsim/） |
| --- | --- | --- |
| YOPO 3m/s、起飞2m | 到达5m门、停车、Land、上锁通过；速度中位2.981、峰值3.268m/s；后半程升高，不代表恒高 | yopo-course-20261114-190858-790815 |
| DiffPhys 5m/s、起飞3m | 策略控制量被拒后watchdog退出；Land、上锁通过；未到终点 | diffphys-course-20261114-191143 |

DiffPhys仍需解决掉高与控制量超限。原生碰撞事件及完整故障矩阵未验收，不宣称完整复现成功。
准备与运行均使用仓库锁定子模块、标准模型目录和现有Inference启动脚本；已移除赛道专属源码/模型路径配置。
历史失败、配置变迁和详细结果见 [验证记录](validation.md)。
