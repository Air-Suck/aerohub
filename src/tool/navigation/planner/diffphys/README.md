# DiffPhys 接入边界

上游`upstream/`锁定提交`271936190b5c2a5e760230e718fa6b167718a5bf`。
算法源码按Planner归属保留；[Inference模型模块](../../../../basic_function/inference/models/diffphys/README.md)
提供官方swarm.pth与high-speed base.pth的独立CUDA环境、权重准备及无状态推理服务。
本阶段不训练，不执行官方会解锁/移动车辆的eval.py。

执行链为任务→ROS策略Adapter→Inference→ATTITUDE FollowReference Behavior→AS2→PX4。
网络输出不是轨迹，不伪装成PVA，也不绕过Behavior发布Motion Reference。
Adapter负责相机采样、状态配对、每任务GRU状态及世代、动作解码和执行生命周期。
超时、过期或旧会话结果不推进隐状态；高速解码保留实际姿态第三行与持续期望航向。

当前保留基线：单DepthPlanar相机，640×480/HFOV91°/Pitch20°，官方裁剪预处理；
high-speed权重、目标速度8m/s、4m原生起飞、10m lookahead、.5rad指令倾角锥和
.6rad实际倾角门槛。第40轮正常森林任务通过，实际速度中位6.359/峰值8.917m/s；
到达后取消姿态执行并交原生GoTo停车，再原生Land/disarmed。
这是单轮正常任务证据，不代表全程8m/s、重复可靠性或整体复现成功。

运行配置、验收证据和已知问题见[森林Demo](../../../../../deployment/airsim/demos/diffphys_forest/README.md)。
高速相机断流停车失败、10m/s碰撞、SGM深度分布、控制响应与估计状态差异仍保留，
后续从问题分析继续实现。官方最高速度声明不作为本平台验收结果。
当前任务/算力在同PC容器的独立进程运行，未验收RoboPi四方拓扑或真机。
