# ProjectAirSim 传感器接入

用户入口为 `config/simulator/default.yaml`，其他场景配置放在同目录并用
`--config` 选择，仅 `*.local.yaml` 本机配置被 Git 忽略。可直接设置
常用相机、IMU、LiDAR 参数；启动时基于上游场景生成实际配置。旧
`basic_px4.json` 的 `adapter.sensor_contract=true` 同样可启用项目传感器接口。数据路径为
ProjectAirSim → 上游 C++ ROS bridge → 本项目 ROS 消息适配器。原生话题保留，
适配器不访问仿真 SDK、不调用运动控制 API，也不发布 `/clock` 或车辆动态位姿。

## 构建与版本

使用 `build_bridge.sh SOURCE WORKSPACE [CMake参数...]` 构建。脚本将上游 ROS 包
复制到外部 workspace 的 `src/`，应用仓库 `patches/` 中的补丁，然后在固定工具镜像
中执行 colcon build/test。C++ client 仍直接使用只读上游源码，不复制整个工程。
旧的、直接以上游路径构建的 workspace 应另选新目录，避免 CMake cache 路径冲突。

两个补丁针对源码 `92032a6c`，均不改上游工作区：

1. CameraInfo 使用 reliable、transient_local、depth=1，保留初始化标定供晚订阅者获取。
2. 在原生 Subscribe 之前安装处理回调，避免立即到达的缓存标定落入无回调窗口；
   订阅失败时撤销注册，后续可重新发现。

升级版本须重新核对补丁适用性和原生转换，不能强制忽略 git apply 失败。
本地资源 `projectairsim_ros2_setup` 指向此次构建的 `install/setup.bash`。

## 对外接口

命名沿用现有 AirSim sensor adapter，前缀为 `/<namespace>/sensor_measurements`。
namespace 和传感器 id 来自 profile 与展开后的原生场景，不硬编码 Drone1/drone0。

| 传感器 | 相对话题 | 类型 / 约定 |
| --- | --- | --- |
| RGB | `<camera>/rgb/image` | Image，BGR8 转 RGB8，保留像素、stride 和采样时间 |
| 视轴深度 | `<camera>/depth_planar/image` | 原生 image-type=1，保留原生浮点深度 |
| 射线深度 | `<camera>/depth_perspective/image` | 原生 image-type=2，保留原生浮点深度 |
| 分割图 | `<camera>/segmentation/image` | 原生 image-type=5，保留编码与数据 |
| 相机标定 | 各图像类型的 `camera_info` | CameraInfo，可靠持久单条缓存 |
| IMU | `<imu>/imu` | Imu，机体 FLU 加速度与角速度；orientation_covariance[0]=-1 |
| LiDAR | `<lidar>/points` | PointCloud2，传感器局部 FLU，保留上游 XYZ 布局 |

本次实际场景启用 RGB、image-type=2 深度、IMU 和 LiDAR；image-type=1 和分割
在接口中可配置，但没有在本轮实际场景中验收。不是所有消费者都接受射线深度，
不能只 remap 话题就当作视轴深度。

相机 frame 为 `<namespace>/<camera>_optical`，IMU/LiDAR frame 分别为
`<namespace>/<sensor>`。以 `<namespace>/base_link` 为父帧发布静态 TF，复用
AirSim adapter 的 FRD→FLU 和 optical 外参转换。LiDAR 已由上游变换为局部 FLU，
本适配器不再旋转点云；只为其赋予与真实外参对应的独立 sensor frame。

原生 IMU 数据源是车辆 kinematics；当前源码没有实现传感器杆臂。非零 IMU 外参
明确拒绝。上游 IMU orientation 的世界系也不能直接作为 AS2 ENU，因此标准接口
将姿态标记不可用，保留加速度和角速度；原始姿态仍可在原生话题查看。

原生 CameraInfo 的 K 来自实际配置，但 P 是单位矩阵占位。适配器只在画幅/FOV
一致、零畸变、R=I 的针孔模型下，将 P 规范为 `[K|0]`、模型标为 plumb_bob。
原生话题不修改；模型不满足约束时适配进程报错退出，不伪造标定。
标定的零时间戳保持为零，不改成当前时间冒充同步测量。连续测量的零时间戳
初始化消息不转发。

## 支持范围与故障行为

当前只支持 identity 根 link 上的固定相机和 LiDAR，以及无杆臂 IMU。关节链、
稳定云台、非 identity 根 link、未知 origin 表达和非有限外参明确拒绝，避免生成
错误静态 TF。相机仍须由场景作者确认无遮挡、杆臂合理和视场合适。

LiDAR 的空回波扫描合法，不能自动丢弃或填零点。上游可能为同一时间戳发布不同
空扫描/分片，逐消息比较须保留此事实；不能按时间戳去重测量。XYZ-only 云没有
ring、强度和逐点时间，不为 LIO 算法伪造这些字段。

Manager 保存 `sensor-contract.json`，记录 source/target/frame/外参和 capture
配置。adapter 脚本同时监管原生 bridge 与标准接口进程，任一退出都会使 adapter
退出，manager status 返回 unhealthy。时钟暂停或数据链失活但进程未退出，须由
消费者新鲜度检查发现；当前没有自动恢复 watchdog。

## 复核方法

在能够访问同一 ROS domain 的环境中加载 ROS，设置变量指向实际证据目录：

```bash
python3 deployment/simulator/basic_flight/sensor_probe.py \
  --topics deployment/simulator/basic_flight/projectairsim_contract_topics.json \
  --output "$RUN_DIR/sensors.json" --seconds 10
python3 deployment/simulator/basic_flight/verify_sensor_contract.py \
  --inventory "$RUN_DIR/sensor-contract.json" \
  --output "$RUN_DIR/paired-samples.json" --seconds 10
```

probe 检查发布、正时间戳、无倒退、相对 `/clock` 新鲜度、编码、frame、标定与
单一时钟发布者。LiDAR 明确允许单帧空回波，但窗口内须有实际点；图像要求非空。
配对检查要求每条流至少5对、配对比例≥95%，验证原生与标准接口时间戳/测量
保留、RGB 像素交换和配置静态 TF。它验证接口传输和配置一致性，不代替具有已知
几何/运动的标定实验或下游算法验收。

### PX4 航向与磁场模型（2026-11-13 课程场景诊断）

ProjectAirSim 当前锁定实现的默认 Magnetometer 使用 `EarthUtils` 简化偶极模型，
不是 PX4 的 WMM。Seattle home `(47.641468, -122.140165)` 的模型磁偏角为
`7.751675°`，本次 PX4 自动 WMM 值为 `15.0112°`。磁偏角不同会使位置与
航向的世界基准不一致：用 PX4 姿态注册局部 LiDAR 会整体旋转障碍物。
碰撞前静止悬停点云独立拟合得到 `7.36°` 偏差，表面距离中位数从
`1.117 m`（原始注册）降至 `0.030 m`（诊断旋转拟合）。该拟合不是运行补偿。

`simulator.projectairsim.magnetic_field.dipole_declination_deg(lat, lon)`
可计算已核实的默认偶极模型值，仅用于诊断。不能直接将该值作为已验证的 PX4
启动修复：本次尝试 DECL_TYPE=0 时 EKF 按零磁偏角对准；DECL_TYPE=2 时 EKF
启动后又将 MAG_DECL 回写为 WMM 的 15.0112°，地面航向检查仍失败。
仅写入参数不能证明修复成功，相应未通过的启动补丁已撤回。

后续需修正平台磁场与 PX4 的参考一致性及参数回写流程，先以冷启动的实际
NED 姿态对照原生 ground truth，再验证已知障碍的静态/空中几何和动态注册。
不得通过固定旋转 LiDAR 点云掩盖状态航向与位置参考的不一致。地面诊断不替代
避障与碰撞验收。原始失败数据和几何拟合仅保存在本轮日志中，不作为运行补偿。

### 已验证的仿真端修正：显式世界磁场

2026-11-13，按用户选择保留原版 flight_board/PX4，在 ProjectAirSim 92032a6 上应用
`patches/0003-configurable-world-magnetic-field.patch`，仅扩展原生 Magnetometer 的
可选 `magnetic-field-ned-gauss` 设置（以空格分隔的世界 NED 向量，Gauss）。
缺省时保留原偶极模型；设置后仍走真实机体/传感器姿态变换、原有噪声和 MAVLink。
配置在 Loader 拒绝零向量与非有限向量，不修改 ROS 点云轴向或状态权威。

课程配置参考原版 PX4 的 WMM decl=15.0112°、incl=68.88993°、strength=0.52760G，
世界向量为 `[0.1835364043, 0.0492168863, 0.4921928955]`G。该值是固定课程局部恒定
模型，不能描述为任意地域完整 WMM 支持。换地理位置/固件模型必须更新参考并复验。

同 UE Clang/libc++ 增量编译 SimLib core，再重链接插件/游戏、Cook、Stage、Archive。
新独立包配合原始 `flight_board:v0.1.1` image ID `0e9c1dabad5a…` 实测：默认
DECL_TYPE=3、MAG_DECL=15.0112、MAG_TYPE=0；20个静止样本最大航向误差0.258°，
磁场分量均值符合配置，全程未解锁。8项原生磁力计测试通过，已知地图24柱与HUD静态
检查通过。上述结果证明地面航向/磁场参考一致，空中地图和5m/s避障尚未验证。
复用 `verify_magnetic_reference.py` 采集实际 PX4 CLI 与原生真值，结果同时保留参数、
解锁状态、磁场样本及门限。不要仅检查字段存在或二进制字符串。
