# AS2 输入接入

AS2 Runtime 的 `as2_navigation_inputs` 封装平台定位与点云，SUPER 消费标准 ROS 2
消息和 AS2 TF。默认规划参考系为 `drone0/odom`。

## 启动

先按主仓库运行说明启动 AS2、AirSim 传感器和唯一的 `/clock` 发布器。
编译并启动 `runtime/navigation_inputs` 包，参见该包 README。
再在已构建 SUPER 的环境中执行：

```bash
source install/setup.bash
ros2 run super_planner fsm_node --ros-args -p use_sim_time:=true \
  -p config_name:="$(ros2 pkg prefix super_planner)/share/super_planner/config/as2_inputs_ros2.yaml"
```

该配置关闭 FSM 的规划/命令定时器，仅接入状态、更新地图和发布可视化；
不接入 AS2 控制输出，不请求解锁或起飞。RViz Fixed Frame 设为 `drone0/odom`。
地图通过 `rog_map/occ`、`rog_map/inf_occ` 等现有接口发布。

## 输入约定

- `/drone0/navigation/odom`：Odometry，header.frame_id=`drone0/odom`，
  child_frame_id=`drone0/base_link`，有非零状态时间戳。
- `/drone0/navigation/points`：注册后的 PointCloud2，header.frame_id=`drone0/odom`，
  保留原始采样时间；支持 XYZ，缺少 intensity 时 SUPER 显式补零。
  使用无 intensity 的传感器时，禁用正阈值 intensity 过滤。
- `/tf`、`/tf_static`：须能查询点云时间的 `drone0/odom → drone0/base_link`。

`rog_map/ros_callback/cloud_pose_from_tf=true` 开启 AS2 输入模式：
地图更新使用点云采样时刻的 TF 位姿，不使用最近收到的 Odometry 替代。
最多缓存 8 帧，等待 `transform_wait` 秒（单调墙钟），超时丢弃；
超过 `odom_timeout` 的旧点云及倒序点云也会丢弃。
Odometry 用于持续更新机器人状态；点云的历史建图位姿独立从 TF 获取。
AS2 负责 TF 发布，SUPER 在此模式下不再广播自己的 `world → drone`。

规划、命令及可视化使用 `rog_map/visualization/frame_id` 选择的同一参考系。
目标须在该参考系表达；SUPER 不隐式变换其他参考系的目标。
默认旧 demo 配置保持 `cloud_pose_from_tf=false` 和 `world`，不要求 AS2 TF。

仿真时间回退后，AS2 输入节点清空队列；SUPER 停止接收/更新新输入并报告错误。
必须重启 SUPER，不能把旧地图与新仿真时刻混合。此模式没有自动地图重置功能。

## 验证范围

PC ROS 2 Humble 中已编译消息、ROG-Map 和 super_planner，并通过合成运动平台测试：
飞机平移和转向时，将不含 intensity 的传感器 XYZ 点云经过 AS2 封装送入真实
ROG-Map，检查固定障碍物位置稳定、地图参考系正确，以及非法输入不进入地图。

在同时 source 两个工作区的环境中可复现：

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 \
  thirdparty/SUPER-sim/super_planner/test/as2_inputs_smoke.py \
  --navigation-executable "$(ros2 pkg prefix as2_navigation_inputs)/lib/as2_navigation_inputs/navigation_inputs_node" \
  --fsm-executable "$(ros2 pkg prefix super_planner)/lib/super_planner/fsm_node"
```

这不是实际 AirSim 飞行或 OHOS 验收。上游 PX4/AirSim 时间映射误差、扫描畸变、
真实运动中的地图稳定性、处理延迟及 RoboPi 资源负载仍需单独测量。

## TODO：MID-360 感知对齐

当前 NH 使用 AirSim 内置 16 线旋转式 LiDAR 配置，不代表 Livox MID-360。
按用户要求，本轮保留当前配置完成输入与地图验证；MID-360 对齐留待下次处理。

- 官方规格参考：https://www.livoxtech.com/mid-360/specs 。
- 对齐目标：水平 360°、垂直 −7°～52°、200,000 点/秒、典型 10 Hz；
  量程需区分 40 米（10% 反射率）和 70 米（80% 反射率），近盲区 0.1 米。
- 单独确定实际安装位置和姿态；不能沿用机体原点零外参作为硬件标定。
- 区分参数近似与扫描模型复现：内置固定线束旋转式模型不能仅靠 settings
  复现 MID-360 随时间累积的非重复扫描覆盖。
- 之后评估逐点时间、扫描运动畸变及 IMU 输入需求，再选定实现与验收范围。
