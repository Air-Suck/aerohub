# AS2 导航输入

独立 ROS 2 C++ 节点，为外部规划器提供同一局部世界系下的定位和注册点云。
只依赖标准消息和 AS2 TF，不依赖 SUPER、AirSim 或 PX4 消息，也不发布 TF。

## 接口

在 `/drone0` namespace 启动时：

| 方向 | 默认 topic | 消息 / 约定 |
| --- | --- | --- |
| 输入 | `sensor_measurements/odom` | Odometry；位姿在 header.frame_id，速度在 child_frame_id |
| 输入 | `sensor_measurements/Lidar/points` | PointCloud2；传感器坐标系，保留原始采样时间 |
| 输入 | `/tf`、`/tf_static` | AS2 定位及传感器安装外参 |
| 输出 | `navigation/odom` | Odometry；位姿转到 target_frame，速度仍在 body_frame |
| 输出 | `navigation/points` | PointCloud2；XYZ 转到 target_frame |

首版默认 `target_frame=drone0/odom`、`body_frame=drone0/base_link`。
Odometry 源采用当前 `raw_odometry` 状态估计器的同源平台输入，保留协方差。
若将来启用融合状态估计，应把 `odom_topic` 切换到相应融合 Odometry 输出；
本节点不会将平台原始状态自动升级为融合定位，也不从 Pose/Twist 拼造协方差。

转换 Odometry 时，旋转位姿协方差，保留 child_frame_id 中的速度及速度协方差。
点云保留采样时间、字段、其他字段数据和布局；首版支持 packed little-endian
FLOAT32 XYZ，不支持行填充或 FLOAT64 XYZ，遇到不支持的布局明确丢弃。
不添加 intensity；SUPER 的 XYZ→XYZI 兼容由 SUPER 自己完成。

## 时间与失效

- 每条消息使用自己的采样时间查询 TF，不用最新 TF 代替历史 TF。
- 点云还要求该时刻的 target_frame→body_frame 可查询，确保规划器能取得建图位姿。
- 通过有界队列等待 TF；等待时间使用单调墙钟，仿真暂停也会超时。
- 默认 `transform_wait=0.25` 秒、`max_input_age=1.0` 秒、每路 `queue_size=20`。
- 缺失 frame、零时间、过期、重复及倒序输入丢弃并限频报告；未来数据等待 ROS 时间。
- ROS 时间后退清空待处理数据及顺序水位。规划器已有地图不能自动沿用，需重启 SUPER。
- 两路输出独立发布，不承诺每个点云时间都恰好有一条同时间 Odometry；
  消费者按点云时间查询 TF。SUPER 的 AS2 输入模式采用此方式。

注意：当前 Pixhawk 回调将 Odometry 标为节点接收时刻，而 AirSim 点云保留测量时刻。
本节点保留输入时间，不能消除上游时间误差。实际运动地图稳定性、端到端延迟和
PX4/AirSim 时钟关系仍需实测；单次整帧变换不提供逐点扫描去畸变。

## 构建与运行

在已安装 ROS 2 Humble 及本包依赖的环境，从仓库根目录执行：

```bash
colcon build --base-paths runtime/navigation_inputs --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run as2_navigation_inputs navigation_inputs_node --ros-args \
  -r __ns:=/drone0 \
  --params-file runtime/navigation_inputs/config/drone0.yaml
```

`Lidar` 来自当前 `px4-sensors.json`，区分大小写；其他传感器通过 `cloud_topic` 参数指定。
AS2、传感器、TF 与本节点须在同一 ROS 时间基准下。先启动已有 AS2/传感器/clock，
再运行本节点。节点没有控制输出，不会请求解锁、起飞或控制模式。
PC onboard_computer 镜像构建包含本包；该节点需显式启动，不改变已有单机运行行为。
RoboPi 已由 `scripts/build_lib.sh swarm-formation` 将本包加入隔离交叉编译工作区，
随 Swarm 独立 overlay 安装；三机真实点云与运动闭环证据见
[Swarm 应用](../../demos/swarm_open_field/README.md)。本机 TF 隔离时，导航节点也须将
`/tf`、`/tf_static` remap 到对应车辆话题。

## 验证

```bash
source install/setup.bash
colcon test --packages-select as2_navigation_inputs --event-handlers console_direct+
colcon test-result --verbose
```

真实 ROS 进程测试覆盖平移/旋转、协方差、速度保留、点云附加字段、时间戳、
延迟 TF、历史 TF 插值、未来样本、TF 超时、非法输入、乱序及仿真时钟回退。
测试使用 ROS_DOMAIN_ID=172 和 localhost DDS，应与业务运行隔离。
实际 AirSim 平移/旋转下的障碍稳定性及板端性能需另行验收。
