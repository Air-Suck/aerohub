# FAST-LIVO2 接入 AS2

本组件保留官方 FAST-LIVO2 为独立 ROS 1 Noetic 进程，通过只读输出桥连接 ROS 2 Humble。自身状态由独立包中的原生 AS2 State Estimator 插件提供，点云走独立 Perception 接口；不创建 Planner 或运动 Behavior，不修改 AS2 上游源码。

插件构建会检测实际 AS2 State Estimator 头文件接口，同时支持原锁定版本和 AirSim 构建补丁中的宿主状态接口。后一种模式通过 `StateEstimatorInterface` 提交变换与速度，由宿主发布主状态和 TF；仅点云别名由插件发布。主变换在宿主完成权限注册后的首个有效测量中初始化。要求 `publish_hz=0`，以免宿主定时重复发布陈旧状态。两种宿主均已通过插件契约测试，此结论不等于 FAST-LIVO2 仿真飞行通过。

## 目录与版本

| 目录 | 用途 |
| --- | --- |
| `upstream/` | 官方 `hku-mars/FAST-LIVO2` Git 子模块，锁定 `0d2c0346107b75b59934975adec9a6eeeb913c64` |
| `plugin/` | `uav_fast_livo2` ROS 2 包，包含原生插件、输出桥、launch 与参数 |
| `ros1_proxy/` | ROS 1 输出桥；仅发送状态、配准点云和回放时钟 |
| `tests/` | 插件契约测试、合成三传感器输入与端到端观察器 |
| `../../perception/fast_livo2/` | 环境接口说明；源码仍只保留一份（仓库路径为 `src/perception/fast_livo2/`） |

构建入口位于 `deployment/fast_livo2/`，依赖来源见 `vendor/fast_livo2-dependencies.md`。官方源码、许可证均保留；FAST-LIVO2 的 GPLv2 与依赖许可不因接入而改变。

## 输入与输出约定

输入由 ROS 1 传感器驱动或 rosbag 提供：IMU、原始图像、所选激光类型的点云／Livox 消息。遵循上游配置中的传感器频率、逐点时间、内外参和时间偏移要求。当前桥只负责算法输出，不提供通用 ROS 2 → ROS 1 原始传感器桥；若传感器只在 ROS 2 中发布，需另接保留原始字段和时间语义的输入桥。

算法必须启用 `uav/imu_rate_odom=true`，启动文件已设置。普通 `/aft_mapped_to_init` 没有速度，不能用作本适配器的输入。构建补丁将 `/LIVO2/imu_propagate` 规范为：

- 位姿：IMU 原点在 `camera_init` 局部世界中的位置和姿态。
- 线速度：将原生世界系速度转到 IMU 坐标系。
- 角速度：当前 IMU 角速度减估计陀螺零偏，表达于 IMU 坐标系。
- `child_frame_id=fast_livo_imu`，时间戳保留 IMU 测量时间。
- 协方差未补充有效估计，不可将默认零值当成零不确定性。本链路不向 PX4 发送外部里程计。

ROS 2 输出：

| 接口 | 内容 |
| --- | --- |
| `/<namespace>/fast_livo2/odometry` | 补丁规范后的原始算法状态，供插件消费 |
| `/<namespace>/self_localization/pose` | 经安装外参及世界对齐变换后的机体位姿，表达于 AS2 earth |
| `/<namespace>/self_localization/twist` | 机体原点的线速度与角速度，表达于 AS2 base_link，包含杆臂项 |
| TF | `earth → <namespace>/map → <namespace>/odom → <namespace>/base_link`；另有 `odom → livo_world` 恒等别名供点云使用 |
| `/<namespace>/perception/fast_livo2/registered_points` | 保留 XYZ、颜色等原生字段的配准点云，不是完整占据地图或 ESDF |
| `/clock` | 仅在 `use_sim_time=true` 时转发 ROS 1 回放时钟，不应与其他时钟源并行启用 |

`imu_from_base` 表示 T_imu_base，`earth_from_livo_world` 表示 T_earth_livo；两者格式均为 `[x,y,z,qx,qy,qz,qw]`。示例恒等变换仅用于同原点测试，不是实机标定或多机世界对齐。插件保留测量时间，拒绝非有限值、错误帧名、非单位四元数、过期／未来／重复／乱序状态，不重复发布陈旧估计。消费者仍须自行实施状态超时保护。

点云只改为明确的 namespaced 局部世界别名，不改变三维表示。算法重置或重新开始数据集会改变局部地图语义，应停止消费者并重启整个估计部署；不支持飞行中的无缝定位源切换。只启动一个权威状态估计节点，不能与原有 PX4 定位插件同时写同一组 self_localization 和 TF。

## Docker 编译与测试

在远程仓库根目录执行，前提是开发者贡献指导所述 AS2 runtime 和 `uav/catora:noetic-upstream` 基础镜像已准备好：

```bash
git submodule update --init src/state_estimator/fast_livo2/upstream
docker compose -f deployment/fast_livo2/compose.yaml build
```

编译产物在镜像内 `/opt/fast_livo_ws/`（算法）和 `/opt/fast_livo2_as2_ws/`（插件）。补丁仅在 Docker 构建副本上应用；不修改锁定上游工作树。ROS 1 编译采用 `-j2`。上游启用了 native CPU 优化，换到不同 CPU／架构需在目标环境重新构建。

使用 AirSim 的 AS2 环境时，给 `Dockerfile.as2` 传入 `--build-arg BASE_IMAGE=<本次构建的机载镜像>`，在该宿主上重新编译插件。不得把原版本编译的插件动态库直接拷入新版宿主；接口兼容不意味着二进制 ABI 兼容。

插件测试（不启动算法）：

```bash
docker run --rm --network host -e ROS_DOMAIN_ID=182 \
  -v "$PWD/src/state_estimator/fast_livo2/tests:/tests:ro" \
  --entrypoint bash uav/fast-livo2-as2:humble -lc '
    source /opt/ros/humble/setup.bash &&
    source /root/aerostack2_ws/install/setup.bash &&
    source /opt/fast_livo2_as2_ws/install/setup.bash &&
    python3 /tests/test_estimator.py
  '
```

实际算法加合成三传感器输入的端到端冒烟：

```bash
bash deployment/fast_livo2/smoke.sh
```

该脚本使用独立 Compose 项目、ROS domain 182、ROS master 11331 和 loopback TCP 18761–18763；测试结束自动收集日志到 `log/fast_livo2/` 并移除测试容器。测试不发飞行指令、不启动 PX4、不更改原有多机部署。合成数据只验证算法与接口链路，不能证明真实定位精度、真实传感器同步或飞行安全。

## 数据集回放或传感器接入

先准备实际传感器对应的上游配置、相机标定及 AS2 安装外参，放入 `log/fast_livo2/`（或修改 Compose 挂载）；样例 Avia 标定只适用于其对应设备／数据集。原始 bag 放在 `log/fast_livo2/datasets/`。

```bash
export FAST_LIVO_CONFIG=/results/sensor.yaml
export FAST_LIVO_CAMERA=/results/camera.yaml
export FAST_LIVO_ESTIMATOR_CONFIG=/results/estimator.yaml
export FAST_LIVO_SIM_TIME=true
docker compose -f deployment/fast_livo2/compose.yaml up -d --no-build
docker compose -f deployment/fast_livo2/compose.yaml exec algorithm bash -lc \
  'source /opt/fast_livo_ws/devel/setup.bash && rosbag play --clock /datasets/recording.bag'
docker compose -f deployment/fast_livo2/compose.yaml down
```

话题须与 sensor.yaml 对应。若 bag 只有压缩图像，先在同一 ROS master 启动 `image_transport republish`，输出配置要求的原始 Image。真实传感器模式使用 `FAST_LIVO_SIM_TIME=false`，由 ROS 1 驱动代替 rosbag；Livox 驱动、硬件同步和标定由设备部署提供。

当前不自动向 PX4 EKF 回灌状态；这需另行处理坐标、协方差、延迟和飞控融合配置。真实数据集、仿真飞行及真机验收进度见 `skill/temp/fast_livo2/README.md`，不得把合成输入测试当作这些验收。
