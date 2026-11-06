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
每个物理相机和 LiDAR 使用独立采集线程；同一相机的多种图像仍合并一次 RPC。
ROS 定时器只投递采集请求，不执行阻塞 RPC，因此某一路读取超时不会阻塞其他
传感器的定时器或 `/clock` 接收。每路最多保留一个待采集请求；处理慢时合并
定时器触发，不积压历史采集任务。RPC 每次使用独立连接，不在线程间共享客户端。
采集触发仍采用 ROS 时钟；仿真暂停时不会持续产生新的周期请求。
节点退出时取消定时器、通知并等待线程结束，再销毁 ROS publisher。
未实现按订阅者休眠；默认读取 settings 中已启用、已配置的全部数据流。
可通过 `camera_enabled:=false` 停止全部相机采集，只保留 LiDAR；这不会关闭
AirSim 内部相机模型，传感器安装 TF 仍会发布。flight_board 对应开关为
`FLIGHT_BOARD_CAMERAS=0`，默认值为 `1`。多线程是唯一的正式采集方式。

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
后续已在 [AS2 完整交互验收](../../deployment/onboard_computer/README.md) 中，直接使用
AS2 运行环境接收全部示例传感器，并通过 PX4 1.18 单机飞行控制闭环。

## 多线程采集对照（2026-11-05）

NH、五路相机/七路图像和一路雷达全部开启；串行、多线程交替各两轮，
每轮预热 5 秒、统计 25 秒。原采集进程仅在对照期间暂停，结束后已恢复。
按仿真时间计算，串行雷达约 2.69 Hz、多线程约 8.07 Hz；全部相机图像
由约 2.7 Hz 提升到 5 Hz。仿真约以 0.95 倍墙钟速度运行，因此多线程
对应墙钟频率为雷达约 7.68 Hz、相机约 4.76 Hz。

多线程恢复了相机目标频率，但雷达尚未达到 10 Hz。雷达到帧间隔 P95
由约 419 ms 降至 219 ms；相机的采样到接收延迟 P95 增至约 75–96 ms，
不能把频率提升等同于延迟降低。Python GIL、图像转换及服务端竞争的
剩余影响尚未分别量化。没有在本轮提高点率或更换 MID-360 模型。
原始数据与复现脚本见 `artifacts/sensor-threading-20261105/`。

## 相机开关对照（2026-11-05）

两组均为正式多线程版本，五路相机的 AirSim 模型与雷达配置不变，只切换
ROS 侧 `camera_enabled`。各两轮，每轮预热 5 秒、统计 25 秒。
雷达仿真时间频率从相机开启时约 8.11 Hz 提升到关闭时约 9.59 Hz；
墙钟频率对应约 7.71 Hz 和 9.10 Hz。雷达到帧间隔 P95 从约 216 ms
下降到 125 ms。相机关闭组七路图像均为零输出。
结果只说明相机采集开销的影响，不代表禁用 AirSim 内部渲染后的性能。
证据位于 `artifacts/sensor-camera-toggle-20261105/`。
