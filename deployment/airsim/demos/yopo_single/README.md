# YOPO 单机 AirSim + PX4 Demo

实现基线为 `plan_from_reference=True`，到达门槛为实际位置距目标<5 m；不是原 Demo 默认 measured 分支，也不提供精确到点。接口与时间线见 [Planner](../../../../src/tool/navigation/planner/yopo/README.md)，模型准备见 [YOPO Inference](../../../../src/basic_function/inference/models/yopo/README.md)。

四方职责：AirSim 提供场景/传感器，flight_board 提供 PX4 SITL/Agent/时钟，独立sensor容器提供相机桥；onboard 镜像中外部 overlay 提供 Planner、Inference ROS Adapter、AS2/FollowTimedPath；工作站镜像只运行任务客户端和仿真碰撞评分。首期共置于 NVIDIA 主机，独立模型 venv 的 GPU worker 在同主机，内部 HTTP 只监听本机。不是 RoboPi 真机验收。

## 准备和构建

前置：已配置 `config/resources.local.yaml` 中 AirSimNH 场景、可用 Vulkan/NVIDIA 渲染与 CUDA、Docker、uv。复用 `onboard_computer:v0.1.0`、`flight_board:v0.1.0`、`osrf/ros:humble-desktop-full-jammy`，不重建共享镜像。AS2 checkout 与 gitlink 不一致，以下构建明确使用**镜像自身** AS2 头文件/库；本轮没有切换用户子模块。镜像 ID、头文件哈希与源码清单见验收记录。

```bash
bash src/basic_function/inference/models/yopo/setup.sh --check-cuda
python3 deployment/airsim/demos/yopo_single/run.py prepare-workstation
```

在固定 onboard 容器中，将本仓库挂载到 `/repo`，加载该镜像 ROS/AS2，再执行：

```bash
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
bash /repo/deployment/airsim/demos/yopo_single/build.sh
source /repo/log/yopo-single/ros_ws/install/setup.bash
cd /repo/log/yopo-single/ros_ws
colcon build --base-paths /repo/src/behavior/follow_path --packages-select uav_follow_path_execution
```

运行时加载同一 install overlay。工作站仅从 onboard 规范定义重新生成 Takeoff/Land/PlatformInfo 所需的最小 AS2 接口集合，不复制 AS2 二进制。YOPO 接口变化后同时重建 Adapter/Planner 和调用方 overlay。

## 分角色运行

算力角色先启动 GPU worker（另一个终端；端口与本 Demo 一致）：

```bash
YOPO_VELOCITY=2 INFERENCE_PORT=18081 bash deployment/inference/start_yopo.sh
```

然后部署与任务分开执行：

```bash
python3 deployment/airsim/demos/yopo_single/run.py start --output log/yopo-single/run-01
python3 deployment/airsim/demos/yopo_single/run.py mission --output log/yopo-single/run-01 --case ground --ground-only
python3 deployment/airsim/demos/yopo_single/run.py mission --output log/yopo-single/run-01 --case open --goal 0 15 2
python3 deployment/airsim/demos/yopo_single/run.py stop --output log/yopo-single/run-01
```

先检查 `ground/result.json.success`，再起飞。`mission` 退出码不代替结果；必须检查 `result.json`。每轮动力学测试优先使用独立新实例，成功三次前不能写成三次验收通过。默认独立端口 RPC45471、TCP4780/4790、UDP14840/14880、Agent8961、DDS domain按轮次从110..170选择并记录于`domain.json`（旧诊断轮使用196），启动前检查占用，不停止其他部署；解锁前还要求clock/pose/twist/status各唯一发布者并收到至少25条单调时钟。单车私有 domain 统一使用全局 TF；不混入其他 Demo 的 per-vehicle TF remap。`controller.yaml` 是本Demo独立控制参数：横向位置kp=0.3、ki=0，原共享值1/0.01在本轮停止中出现衰减振荡，不能满足停止门槛；未修改共享配置。

`settings.json` 是共置且 pitch=0 的 DepthPlanar 相机示例，320×185、水平90°，与训练几何匹配后 resize；不能替换为 DepthPerspective。原点与相机 TF 校验失败时 Planner ready=false。

## 取消、故障和恢复

工作站调用 `/drone0/yopo/cancel`（Trigger）后等待 `yopo/status.state=STOPPED`。服务成功只代表请求停止；STOPPED 还要求停止应答、控制器悬停/位置模式、定位新鲜、速度<0.2 m/s 连续1 s。10 s 内无法确认则 FAULT。任务脚本随后调用原生 Land，确认解除武装，并尝试退出 Offboard（额外请求结果单独记录，不以服务拒绝推断实际模式）。Land 失败会记录 recovery_error；保留 flight_board、clock、场景，先修复/执行仿真恢复，不能把强制结束场景算作成功降落。

`run.py stop` 要求最新任务报告 disarmed，且只清理带本次输出目录标签的容器和带进程身份记录的中继/场景。算力启动失败、平台仍在地面时，可检查其日志后对**已退出**容器用 `restart-compute`；不支持飞行中热换。worker 停止/过期输入/人工取消的正式动力学验收应分别记录，不能用测试替身证明实际飞行恢复。

## 验收记录

当前结果与边界见 [validation.md](validation.md)。任务证据在 `log/yopo-single/`，过程在 `skill/temp/yopo-single/`；数值/接口/服务链与实际动力学分开记录。

故障用例同样从新实例开始，`--fault cancel` 由工作站经ROS取消；`--fault stale` 由编排器仅暂停本次独立sensor容器，任务结束后恢复，PX4和时钟持续运行；`--fault worker-exit --worker-pid <本次worker进程ID>` 在3个revision后验证进程身份并中断该worker。后者必须先记录本次worker PID，结束后显式重新准备/启动；不会终止其他模型或GPU进程。每轮都检查 `fault-injection.json`、停止原因、Land和disarmed，不能仅看脚本退出码。

原场景短程障碍任务使用 `start --scene forest`，沿用SUPER场景已知起点、只更换相机配置，默认目标仍为相对固定ENU `[0,15,2]`。起飞后保存真实视轴深度，若水平中心带没有15m内障碍则拒绝将其当作障碍任务；该观测门槛不证明障碍恰好在最终飞行路径上。必须结合深度与路径/碰撞记录说明实际覆盖。

恢复专用入口：

```bash
python3 deployment/airsim/demos/yopo_single/run.py mission --output log/yopo-single/run-01 --case recovery --recover-only
```

它仅执行停止请求和原生Land恢复，不发送新目标；取消拒绝不会阻断降落。`run.py stop`根据最新结果检查解除武装，恢复结果单独保留，不把失败任务改为成功。

## 显示屏现场观看

从当前桌面会话提供`DISPLAY`和所需的`XAUTHORITY`，对`run.py start --scene forest`设置`AIRSIM_RENDER_MODE=windowed`即可复用同一任务链。先确认AirSim窗口实际可见并完成`--ground-only`，再发送同一15m短程任务；不要因打开窗口改变速度、轨迹或停止门槛。用户继续观看时可保留已解除武装的场景，随后用同一输出目录的`stop`入口回收，并另行停止本次worker。个人显示配置不写入共享文件。

长距离可显式设置任务导航超时，例如本轮单次有屏通过的原森林长目标：

```bash
python3 deployment/airsim/demos/yopo_single/run.py mission --output <本轮输出目录> --case forest-long --goal -37.57 231.76 6 --timeout 300
```

`--timeout`只控制工作站等待导航结果（默认仍90s），不改变任何执行停止门槛或速度。配置2m/s并非恒速上限；单次235m有屏运行实测中位1.992m/s、峰值2.309m/s。长程记录在`log/yopo-single/live-forest-long-01/forest-long/summary.json`；单次通过不代替三次长程验收。

## 定向回归

模型数值基线见模型README；纯曲线/执行采样数学及ROS请求契约可分别运行：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/tool/navigation/planner/yopo/ros2_adapter src/basic_function/inference/models/yopo/.venv/bin/python -m unittest discover -s src/tool/navigation/planner/yopo/ros2_adapter/tests -v
PYTHONPATH=src/basic_function/inference/ros2_adapter python3 -m unittest discover -s deployment/inference/tests -p test_contract.py -v
```

在已加载匹配AS2和本项目overlay的固定ROS镜像中，运行`python3 deployment/airsim/demos/yopo_single/test_ros_planner.py`验证真实ROS传输与测试替身生命周期；使用独立网络/domain，该测试不执行SITL飞行。`guid_probe.py`仅作只读DDS身份诊断，不参与正常部署。
