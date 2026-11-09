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

先确认 scene 二进制、AirSim/PX4 源码版本与实际配置一致。源码中存在功能不证明下载的预编译场景包含它。检查仿真时间与 PX4 时间如何映射、暂停/reset 后是否清空旧数据，以及各车辆 TF/话题是否隔离。

## FAST-LIVO2 案例的已验证边界

项目已新增算法专用 AirSim RPC 输入桥，提供 IMU、RGB 图像和 Velodyne 兼容 PointCloud2。点云包含 `x/y/z/intensity/time/ring`；`time` 全零并明确表示 AirSim 1.8.1 没有逐点采样时间，上游补丁通过 `preprocess/instantaneous_scan=true` 禁止按扫描角速度伪造偏移。ring 由配置的垂直视场和点的仰角推导；这是一种仿真适配属性，不应描述成真实硬件回传值。

坐标约定已用平台数据验证：AirSim IMU 和 LiDAR 的局部 FRD 转换为 FLU，相机保留光学坐标；LiDAR 到前视相机的矩阵为 `x_cam=-y_lidar, y_cam=-z_lidar, z_cam=x_lidar`。车辆沿 AirSim 世界 `+Y` 平移 1 m 的点云对照中，原始 FRD 补偿的中位配准误差为 `0.285 m`，错误横轴翻转为 `0.551 m`，因此不得为了修正算法轨迹而再次翻转横轴。

实际有屏 shadow 飞行证明三种测量进入真实算法，配准点云也由 ROS 2 Perception 消费；但 LIO/LIVO 状态误差超过预设 `0.5 m` 门限，并出现航向跳变。该结果是输入链通过、定位精度失败，不得写成 AirSim 状态估计或权威控制闭环通过。authority 必须由 shadow gate 阻止。

AirSim 射线循环没有可供算法恢复的逐点时间，不能将其直接等同真实旋转雷达。后续诊断应优先保存可重放的原始三传感器记录，分开评估扫描模型、场景可观性、内外参与估计器参数；不要用合成字段、真值状态或放宽门限替代缺失语义。
