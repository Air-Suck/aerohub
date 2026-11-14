# DiffPhys 官方 checkpoint 推理

本模块默认加载官方 v1.0 的 `swarm.pth`，保留有里程计、10维观测、6维输出和192维 GRU 隐状态。算法源码继续位于 `src/planner/diffphys/upstream/`，不运行训练。`DIFFPHYS_PROFILE=high_speed` 显式选择另一套 Flightmare 高速评估的 `base.pth` 与发布包模型源码，分别校验SHA256并严格加载；这不是swarm网络的可互换权重。

ROS Planner与ATTITUDE执行已接通，短程及部分长程诊断有真实AirSim＋PX4证据，但完整复现仍未完成，见[森林Demo](../../../../deployment/airsim/demos/diffphys_forest/README.md)。官方 `swarm/eval.py` 使用 SimpleFlight 与 AirSim 姿态/推力 API，不能当成本项目 AS2/PX4 执行链的验收入口。

本文后续旧数值序列证据适用于默认swarm。高速当前输入版本为
`high-speed-hfov91-crop82-nearest36x48-pool3-state10-explicit-h-v2`：Planner从
640×480、水平91°、Pitch20°的AirSim米制深度，按官方整数裁剪及nearest采样为36×48；
零值置24米，worker执行逆深度与pool3。归一化与像素选择可交换，CUDA30帧输出和h
对照逐值一致，入口为Planner的`tests/test_camera.py`。ROS源CameraInfo不伪造缩放标定。
旧79°全幅v1仅保留在历史证据中；当前仍用理想DepthPlanar，未复现双目SGM误差分布。
高速调用方观测使用实际姿态第三行与margin0.2；swarm使用水平基第三行与margin0.15。

## 准备

前置条件：Linux x86_64、uv、Python 3.8（uv 可管理）、初始化的锁定 upstream，以及已下载的官方 [validation_code.zip](https://github.com/HenryHuYu/DiffPhysDrone/releases/tag/v1.0)。CUDA 检查另外要求 NVIDIA GPU 与兼容驱动，不安装系统 CUDA Toolkit 或训练自定义算子。

```bash
bash src/inference/models/diffphys/setup.sh --archive /path/to/validation_code.zip --check-cuda
```

默认检查 `log/diffphys/assets/validation_code.zip`，亦接受完整下载、尚未改名的 `.zip.part`；文件扩展名不作为完整性证据。脚本按 [artifacts.json](artifacts.json) 校验完整归档及选定文件，仅提取 `swarm.pth`、模型/评估脚本和相机配置至忽略的 `artifacts/swarm/`。已存在但哈希不符时失败并保留文件；不会下载或启动评估器。校验和是本次从官方发布包建立的固定内容基线，并非作者签名。

环境安装至本目录 `.venv/`，准确依赖见 `pyproject.toml` 与 `uv.lock`。选用 Python3.8/PyTorch2.4.1+cu118，复用现有官方 CUDA wheel 缓存；官方评估说明为 Python3.8.18/PyTorch2.1.2。本轮对照在新环境中分别执行官方模型及当前上游模型，证明其输出兼容，不声称已对照旧2.1.2运行时。禁用 TF32，以完整 FP32 进行连续序列对照；训练源码的初始化缩放被严格加载的完整 checkpoint 覆盖。

公共选择入口登记为 `diffphys`。在已有用户 YAML 的 `inference.models` 中选择它，通过 `deployment/inference/setup_models.py --config <你的YAML> --check-cuda` 准备；不自动覆盖用户配置。

## 启动与停止

```bash
bash deployment/inference/start_diffphys.sh
```

内部默认 `http://127.0.0.1:8001`；`INFERENCE_HOST`、`INFERENCE_PORT` 配置绑定地址，`CUDA_VISIBLE_DEVICES` 选择 GPU。默认要求 CUDA，不自动回退 CPU。独立 CPU 检查显式设置 `DIFFPHYS_DEVICE=cpu`。

按 [公共 ROS 构建入口](../../../../deployment/inference/build_ros.sh)在稳定 Humble 镜像中补编译两个包，显式设置 `INFERENCE_ROS_WORKSPACE` 为独立工作区并 source 其 install overlay，然后：

```bash
python3 -m uav_inference.diffphys_node --ros-args \
  -r __ns:=/drone0 -p endpoint:=http://127.0.0.1:8001
```

ROS Adapter 与模型 worker 属于算力角色，工作站调用 `/<namespace>/inference/diffphys/infer`，类型为 `uav_inference_interfaces/srv/InferDiffphys`。`inference/diffphys/status` 发布 worker 就绪状态；它不代表相机标定或运动执行已就绪。跨网络命名空间必须配置真实可达 endpoint。

对本任务启动的 Adapter 和 worker 发 SIGINT，等待进程退出后再替换源码、权重或环境；恢复时重新 source 相应 overlay。服务不持有任务隐状态，调用方应在任务重启、状态重置或时钟回退后明确重新初始化。

## 输入输出契约

- 深度：严格36×48，DepthPlanar 视轴深度，正有限米值；ROS接受 `32FC1` 米和 `16UC1` 毫米，处理端序与行 padding。无法判断无效返回的物理语义时拒绝零/NaN/Inf，不擅自填洞。模型按官方评估执行 `3/clamp(depth,0.3,24)-0.6` 和3×3最大池化，得到12×16；不加入训练噪声。
- 观测：`[local_v_xyz, local_target_v_xyz, level_R_row2_xyz, margin_m]`。调用方从同一冻结快照构造水平航向对齐基：x为机头方向的水平投影，z向上，y=z×x。速度和限幅后的目标速度投影到这个基。**官方 swarm 评估使用水平基的第三行 `R[:,2]`，通常为[0,0,1]，不是完整机体姿态的第三行**；本接口保留实际评估契约，没有自行“修正”为训练实现。它也不提供无里程计分支。
- 几何：官方配置水平FOV79°、36×48、相机位置(0.25,0,-0.1)与Pitch10°（AirSim配置坐标），并使用x保持、y/z取反的局部世界/机体系转换；不能照搬YOPO的90°视场或共置相机。真正接入AS2前仍需将其映射到固定ENU与FLU/光学系，保留相机杆臂和深度语义。
- 时间：`state_header` 与深度各保留采样时刻/frame；ROS默认年龄≤0.5s、偏差≤50ms，拒绝未来观测；响应前重复检查新鲜度。此处没有YOPO的未来规划参考时间。
- 隐状态：请求为空/内部JSON为null表示重置，否则恰192个有限值；返回下一隐状态。worker自身不保存历史。调用方每车/每任务维护状态与世代，限制每车单个真实未结束请求，只在对应响应仍有效时接纳h_next；不因HTTP超时自动重试或推进h。拒绝/过期结果不能更新当前状态。任务目标变化是否重置须由后续Planner明确设计。
- 输出：6维原始网络输出，按官方reshape对应 `[set_x, est_x, set_y, est_y, set_z, est_z]`，以及192维h_next、request_id、模型版本和实际设备；不是轨迹或可直接发送的姿态/推力指令。

官方评估将两组三维量旋转回局部世界，再计算二者之差，加入重力和随机控制偏差，并构造姿态/归一化油门；`hover_thr=0.297` 是其SimpleFlight配置，不是PX4标定。本模型服务不承担这些运动后处理，不发布Motion Reference。现已提供FollowReference的显式姿态＋推力模式（见`src/behavior/follow_reference/README.md`）；后续DiffPhys接入仍需核查推力映射、资源互斥、命令覆盖到期、取消及悬停交接；不能把输出伪装成终端PVA硬塞FollowTimedPath。

## 复验与边界

```bash
src/inference/models/diffphys/.venv/bin/python src/inference/models/diffphys/test_baseline.py --device cpu
src/inference/models/diffphys/.venv/bin/python src/inference/models/diffphys/test_baseline.py --device cuda
python3 deployment/inference/tests/diffphys_cross_container.py
```

前两项从已验证的官方eval.py抽取无副作用的预处理/模型调用块，在40帧非恒等姿态、非零速度及深度边界输入下比较全部action和h，atol=1e-5/rtol=1e-4；另检查重置、分支独立性和无效输入拒绝。**不会导入或执行包含解锁/运动操作的官方eval.py**。

双容器脚本使用现有稳定工作站/onboard镜像、独立overlay和本次临时worker；20次GPU服务往返预设每次<100ms，并检查单位/端序/padding、重置和无效时间/状态输入。脚本不启动AirSim或调用飞行接口，结束后清理本次进程及容器。工作站接口overlay由脚本生成；onboard overlay需先构建在 `log/diffphys/ros_ws`。

验收摘要见 [validation.json](validation.json)，该文件记录swarm模型数值及合成ROS输入链，原始记录在 `log/diffphys/`。高速profile选择`DIFFPHYS_PROFILE=high_speed`，独立加载官方base.pth；Planner使用实际姿态第三行、margin0.2及官方高速相机裁剪，不能套用上述swarm水平姿态基约定。当前相机与PX4正常森林8m/s目标速度证据另见[森林Demo](../../../../deployment/airsim/demos/diffphys_forest/README.md)，不代表跨物理设备、多机效果或完整复现通过。

源码及权重保留官方出处和校验清单；所检查的DiffPhys上游和swarm发布目录未附明确许可证，不套用YOPO的MIT声明。本仓库仅保存薄适配与准备入口，官方发布文件保留为本地准备产物。
