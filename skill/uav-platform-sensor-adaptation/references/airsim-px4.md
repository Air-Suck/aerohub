# AirSim + PX4 SITL 核查要点

适用于当前项目的平台接入。以下是核查入口与已知边界，不是已经全部跑通的部署流程。

## 数据来源分开核实

| 通路 | 核查重点 |
| --- | --- |
| AirSim → PX4 SITL → AS2 平台适配 | 飞控收到的仿真测量与输出的估计状态要分开；PX4 内部有某 uORB 话题不代表已通过 DDS/MAVLink 暴露，须核对目标版本配置 |
| AirSim RPC → 传感器桥 → ROS | 图像/CameraInfo、激光点云等可独立提供给算法，不要求绕经 PX4；IMU 若直接使用 RPC，需明确其与 PX4 所用 IMU 的实例、噪声和时间关系 |
| 算法输出 → AS2 状态插件/感知接口 | 位姿与速度不等于地图；插件发布正常不代表输入正确或 PX4 已融合外部定位 |

AS2 的 IMU topic 名不能证明数据未经处理；检查角速度、加速度、orientation 的实际来源。算法需要 IMU 测量时，不以 PX4 已估计姿态替代。PX4 外部里程计回灌是另一个有方向的接口，需要单独配置和验证；它不是传感器桥自动附带的功能。共享 IMU 时还需考虑估计相关性，不能将两路结果视为独立测量随意融合。

当前入口：

- `simulator/airsim/config/`：场景、传感器与资源配置。
- `simulator/airsim/sensors/`：RPC 与 ROS 传感器桥。
- `deployment/flight_board/`：PX4 SITL、消息及传感器/时钟桥环境。
- `src/perception/navigation_inputs/`：导航坐标注册；不是通用雷达逐点去畸变。
- `src/state_estimator/fast_livo2/`：算法、原生插件和现有输出桥；检查是否另有输入桥，不推定输出桥能反向使用。
- `src/state_estimator/pin_slam/`：复用现有标准 ROS 2 LiDAR 话题的算法 Adapter 与状态插件；不要再增加第二个 AirSim RPC 读取器。

先确认 scene 二进制、AirSim/PX4 源码版本与实际配置一致。源码中存在功能不证明下载的预编译场景包含它。检查仿真时间与 PX4 时间如何映射、暂停/reset 后是否清空旧数据，以及各车辆 TF/话题是否隔离。

## FAST-LIVO2 案例的已验证边界

项目使用统一 AirSim RPC ROS 2 传感器桥提供 IMU、RGB 图像和 PointCloud2，再由 FAST-LIVO2 Adapter 转为锁定算法接口。AirSim 1.8.1 没有逐点采样时间，适配明确使用 instantaneous scan，不按扫描角速度伪造时间；仿真推导属性不能描述成真实硬件回传值。

坐标约定已用平台数据验证：AirSim IMU 和 LiDAR 的局部 FRD 转换为 FLU，相机保留光学坐标；LiDAR 到前视相机的矩阵为 `x_cam=-y_lidar, y_cam=-z_lidar, z_cam=x_lidar`。车辆沿 AirSim 世界 `+Y` 平移 1 m 的点云对照中，原始 FRD 补偿的中位配准误差为 `0.285 m`，错误横轴翻转为 `0.551 m`，因此不得为了修正算法轨迹而再次翻转横轴。

实际有屏 shadow 飞行证明三种测量进入真实算法，配准点云由 ROS 2 Perception 消费，状态误差通过预设 `0.5 m` 门限；随后 FAST-LIVO2 作为 AS2 状态/TF 权威完成闭环飞行，并在传感器中断和算法退出后受控降落。PX4 的航向 preflight 偶尔无法稳定，必须在启动上层链前用实际状态消息拒绝该轮运行，不能仅依赖启动日志或关闭检查。

AirSim 射线循环没有可供算法恢复的逐点时间，不能将其直接等同真实旋转雷达。后续诊断应优先保存可重放的原始三传感器记录，分开评估扫描模型、场景可观性、内外参与估计器参数；不要用合成字段、真值状态或放宽门限替代缺失语义。

PIN-SLAM 的当前 AirSim 接入直接消费标准 FLU XYZ PointCloud2。因为该帧是瞬时射线结果且没有逐点时间，配置必须保持 `deskew: false`，也不生成 ring、强度或时间字段。适配器在进入 GPU 前拒绝错误 frame、非单调时间、空帧和非单值 FLOAT32 XYZ；这些接口检查通过仍不代表神经地图或定位精度通过。
