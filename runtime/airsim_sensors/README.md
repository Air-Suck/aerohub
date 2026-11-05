# 平台传感器接入

`uav_as2_sensor_adapter` 是 AS2 平台封装内部的传感器后端。当前实现 AirSim
后端；上层只依赖标准 ROS 2 消息，不依赖 AirSim API。它是独立 ROS 节点，
不修改上游 AS2，不要求把图像、点云送入运动控制节点转发。

## 对上接口

| Topic（默认 namespace 为 drone0） | 消息 |
| --- | --- |
| `/drone0/sensor_measurements/<camera_name>/rgb/image` | `sensor_msgs/Image`，rgb8 |
| `/drone0/sensor_measurements/<camera_name>/depth_planar/image` | Image，32FC1，米，沿光轴深度 |
| `/drone0/sensor_measurements/<camera_name>/depth_perspective/image` | Image，32FC1，米，沿视线距离 |
| `/drone0/sensor_measurements/<camera_name>/segmentation/image` | Image，rgb8，分割颜色 |
| 每路图像同目录的 `camera_info` | CameraInfo |
| `/drone0/sensor_measurements/<lidar_name>/points` | PointCloud2，float32 XYZ，米 |
| `/tf_static` | base_link 到各传感器的静态安装变换 |

相机数量不限于示例的五个。节点从 settings.json 中指定车辆的 `Cameras`
和 `CaptureSettings` 发现数据流，用 `camera_name` 命名接口；LiDAR 同理从
`Sensors` 中发现。名称须为字母开头的字母、数字、下划线组合。
每个车辆启动一个节点，用 `vehicle_name` 映射 AirSim 车辆，
`vehicle_namespace` 指定统一 ROS 名称。读取频率可通过 `camera_rate_hz`
（默认 5）和 `lidar_rate_hz`（默认 10）调整，RPC 超时默认 3 秒。

## 坐标、时间与标定

- 图像使用 ROS optical 坐标（右、下、前）；点云使用传感器 FLU 坐标
  （前、左、上）。AirSim FRD 安装参数在平台内部转换。
- 支持 LiDAR SensorLocalFrame 和 VehicleInertialFrame，后者先按传感器
  pose 转回局部坐标。不会把世界坐标点错误标记为传感器坐标。
- 保留 AirSim 原始纳秒采样时间；同一图像及其 CameraInfo 使用相同时间戳，
  重复采样不重复发布。传感器节点本身不发布 `/clock`；
  [flight_board](../../deployment/flight_board/README.md) 的独立 clock 模式提供
  AirSim 全局时间桥，不能同时沿用 Gazebo 时钟桥。完整 AS2/PX4 时间映射仍需验收。
- CameraInfo 由配置 FOV、实际图像尺寸计算，假设无畸变、方形像素的针孔模型。
  当前不支持镜头畸变、运行时云台运动或运行时变焦；不要将此静态 TF/标定
  用于这些场景。DepthPerspective 是视线距离，不应直接当成光轴 Z 深度。
- 默认源 RGB 顺序适用于本次 Vulkan 场景；其他渲染后端若返回 BGR，设置
  `source_color_order:=bgr`，对上仍统一输出 rgb8。
- 图像、标定、点云使用 sensor-data QoS；TF 使用标准静态变换发布方式。
  RPC 失败会限频记录并重试，不生成伪数据。

## 运行

传感器和时钟已包含在固定的 Humble flight_board 镜像中，不需要独立镜像。
按[flight_board 说明](../../deployment/flight_board/README.md#使用固定镜像)
加载 tar，再启动 vehicle 与 clock 容器。RoboPi 场景由
`scripts/robopi.sh up` 统一启动，板端继续原生运行 AS2。

传感器与 AirSim 必须读取同一份 `config/airsim/settings/px4-sensors.json`。

示例融合 stage3 的五向相机和 racer 的 16 线 LiDAR：前/下 RGB +
DepthPerspective，左/右/后 DepthPerspective，图像均为 256×256；
LiDAR 为 16 线、25 米、10 转/秒、80000 点/秒，使用 SensorLocalFrame。

## 真实场景验证

```bash
bash scripts/robopi.sh verify
```

独立 ROS 订阅节点只读取平台 topic 和 TF，不调用 AirSim。已验证 7 路图像、
7 路 CameraInfo、1 路非空点云及 6 个传感器 TF；一次点云包含 5193 点。
证据见 `artifacts/airsim-verification/sensor-result.json`。
该验收是单机静止状态的数据通路验证，尚未验证 SLAM、避障、飞行闭环、
频率/延迟指标或多机负载。
后续已在 [AS2 完整交互验收](../../deployment/companion_as2/README.md) 中，直接使用
AS2 运行环境接收全部示例传感器，并通过 PX4 1.18 单机飞行控制闭环。
