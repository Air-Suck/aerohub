# DiffPhys 单机森林 Demo

2026-11-11：保留当前单相机配置，以**目标速度8 m/s的正常森林任务**作为本阶段已通过的验收基线。
证据为`log/diffphys/forest-long-40-takeoff4-repeat/`；单轮通过，实际速度中位6.359、
峰值8.917 m/s，不等于全程8 m/s或稳定重复通过。故障停车与更高速能力仍有未完成项，
不声明整体复现成功。问题、失败与上游差异统一见[已知问题](KNOWN_ISSUES.md)。

## 配置与执行链

| 项目 | 当前基线 |
| --- | --- |
| 权重 | 官方v1.0 high-speed `base.pth`，SHA256见模型artifacts.json |
| 场景与目标 | SUPER原森林起点NED [42.6,160.11,0]，目标AS2 ENU [-37.57,231.76,6]，原边界 |
| 起飞与引导 | 原生Takeoff 4m；目标速度8m/s；goal lookahead 10m |
| 姿态约束 | 指令倾角锥0.5rad，实际倾角验收上限0.6rad |
| 相机 | 一个DepthPlanar，640×480，HFOV91°，Pitch20°，AirSim位置[0.5,0,-0.1]m |
| 预处理 | 82%中心裁剪→nearest 48×36→逆深度归一化→pool3，零深度置24m |
| 时序 | 策略15Hz，ClockSpeed1；RELIABLE/KEEP_LAST1；关闭无消费者LiDAR与重复传感器桥 |
| 预算 | 到达及停稳共300墙钟秒，导航300仿真秒看门狗；任务600墙钟秒保护 |

链路为真实相机→ROS→CUDA Inference→Planner Adapter→ATTITUDE FollowReference
Behavior→AS2→PX4。运动不经AirSim RPC；RPC用于传感器和评分。任务与算力目前在同PC
容器中作为独立进程运行，未验收RoboPi四方拓扑。复用固定镜像和外部overlay，
不因源码更新重建共享镜像。

## 准备与运行

宿主需有uv、NVIDIA GPU、官方发布文件和可用图形会话。启动器使用的Python需具备
PyYAML、msgpack和msgpack-rpc-python（可复用已有uv环境）。

```bash
bash src/inference/models/diffphys/setup.sh --check-cuda
bash deployment/airsim/demos/diffphys_forest/build.sh
DIFFPHYS_PROFILE=high_speed INFERENCE_PORT=18082 bash deployment/inference/start_diffphys.sh
```

保持worker运行，在另一终端执行；每轮使用新证据目录：

```bash
AIRSIM_RENDER_MODE=windowed python deployment/airsim/demos/diffphys_forest/run.py \
  --diffphys-long --output log/diffphys/forest-8mps-new
python deployment/airsim/demos/diffphys_forest/analyze.py log/diffphys/forest-8mps-new
```

`--diffphys-long`默认填入上述8m/s、4m起飞、10m前视、.5rad指令锥配置，显式参数可覆盖。
不传long仍保留官方swarm权重的短程接口；启动worker时profile必须匹配。
稳定镜像为`uav/apps-super-forest:repo-native-20261109`与`flight_board:v0.1.0`，
默认overlay为`log/diffphys/attitude-overlay`，同一工作区不并行构建或运行。

AS2构建输入来自固定镜像：build.sh复用attitude_control/build_overlay.sh，
从镜像复制控制器与Pixhawk平台源码，再按deployment/platform/patches的校验清单
应用补丁并补编译；本入口不读取本地`as2/aerostack2`子模块源码。
主仓库AS2锁定指针保持不变，本地额外检出旧提交不作为部署依赖或提交内容。
后续若切换为仓库AS2源码构建，应单独验证，不能沿用镜像overlay的通过结论。

任务结束自动回收所属场景、容器和中继；worker由其部署终端独立持有，完成后SIGINT
并等待退出。若降落恢复未确认，入口保留所属控制/时钟/场景，使用IPC控制的
`recover_ground.py --output <目录>`确认解除武装后回收，不停止其他实例。

## 停车与证据

正常任务首次入圈后取消ATTITUDE，等待执行结束和POSITION反馈，再交原生GoTo到原目标
（max_speed2.5m/s）。终点距离≤1.5m、速度<.3m/s持续2仿真秒并确认GoTo成功。
随后独立检查10墙钟秒内执行退出，两侧Action结束、POSITION反馈和连续.25墙钟秒
无新策略控制帧，最后原生Land并确认disarmed。相机断流仍保留原10墙钟秒物理恢复门槛。
不放宽原倾角、边界、碰撞或故障验收条件。

第40轮：250.542m路径，43.786仿真秒首次入圈，终点误差.0462m；最大倾角.5482rad；
停车7.736墙钟秒，低速驻留2.010仿真秒，执行退出.316墙钟秒；501次CUDA策略调用，
无新增碰撞，Land/disarmed与资源清理通过。摘要见[validation.json](validation.json)。
当前lookahead已修复方向与速度耦合；该轮10m前视/8m/s不受此前隐式限速影响，
短前视历史结果不能据此重判。整理后未追加飞行，后续运行仍需独立验收。

请求保留图像/状态采样时间，配对偏差≤50ms，新鲜度≤200ms；每任务仅一个真实未结束
推理请求。过期或旧会话结果不提交GRU隐状态，重复控制帧不刷新求解时刻。
姿态会话显式使用YAW_FROM_TOPIC，解码保留实际姿态观测及持续期望航向；求解有效期
200ms、控制流断连保护100ms。推力按起飞后PX4悬停设定值映射，非真机力标定。

本次整理移除一次性时延探针和配置对齐检查入口，原始证据保留在log/diffphys；
不以更换验收范围或整理文档重判任何历史失败。后续深度SGM、控制器辨识与提速
均从已知问题记录重新开展，不属于当前默认部署。
