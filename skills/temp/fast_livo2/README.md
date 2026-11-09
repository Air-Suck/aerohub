# FAST-LIVO2 复现与 AS2 适配记录

## 目标与环境（2026-11-07）

- 主工作区：RKOH_Remote `/home/noah/code/UAV/aerostack2-integration`。
- 遵循开发者贡献指导：算法独立进程，状态由 AS2 原生插件接入，环境表示独立输出。
- 不切换现有飞行部署的定位源，不把接口冒烟测试视作真实定位或飞行验收。
- 官方源码：<https://github.com/hku-mars/FAST-LIVO2>，main 当前提交 `0d2c0346107b75b59934975adec9a6eeeb913c64`，以子模块锁定在 `src/state_estimator/fast_livo2/upstream/`。

## 核查过程

- AS2 `raw_odometry` 直接复制输入 twist，并在帧名不匹配时仅改名，不执行刚体变换。因此必须先核实 FAST-LIVO2 的速度输出及坐标语义，不能直接 remap 后宣称兼容。
- 原生插件宿主将 `plugin_name` 拼接 `::Plugin`，使用 pluginlib 加载 `StateEstimatorBase` 派生类。可以在独立包注册，无需修改 AS2 核心。
- 远程已有 ROS 2 Humble AS2 镜像和 ROS 1 Noetic 算法基础镜像，剩余磁盘约 218 GB。

## 实现决策与问题处理

1. 官方仓库当前是 ROS 1 catkin 工程。保留独立 Noetic 进程，通过 loopback TCP 的三个独立流向 Humble 发送状态、点云、可选回放时钟。只保留最新样本，不缓存重放陈旧状态；不是通用 ROS1/ROS2 bridge。
2. 原生 `/aft_mapped_to_init` 不填写速度且使用发布时间。采用 `/LIVO2/imu_propagate`，在构建补丁中将世界系速度转为 IMU 系、补充去零偏角速度和子坐标系，保留 IMU 采样时间。地图头时间改为 LIO/VIO 更新时刻。
3. 使用独立原生 `fast_livo2::Plugin`：校验输入、变换 IMU 原点到 base 原点（含杆臂速度）、对齐局部世界到 earth，发布统一状态及 TF。无需修改 AS2。
4. 地图输出保留 `PointCloud2` 的 XYZ/RGB 与完整二进制布局，不强制转换为二维 Map Server 输出。实际是配准点云，不是完整持久地图。
5. GitHub 从远端 clone 两次失败（TLS 断开、连接超时）；从本地临时目录拉取官方 shallow clone 后同步到远端，并执行 submodule add/absorbgitdirs。上游工作树保持干净，完整提交锁定。
6. 上游 README 指向的 Sophus `a621ff` 无法解析，当前 Sophus 已模板化且不兼容。检查官方历史后选择非模板提交 `79830a101511fa5a52cf2c68be408b96926d7bcd`，增加构建安装配置和 complex 初始化兼容补丁。Vikit 锁定 `6c886c8e5d83997806e00294826d528cea3581dd`。来源和归档校验见 vendor 文档。
7. 首次插件编译发现 AS2 安装了基类头文件却未导出 include 路径；独立包从 as2_state_estimator 的 CMake prefix 查找，不改上游导出。
8. 首次端到端观察器失败：上游在启动阶段发布无字段的空点云。桥接层现跳过空点云并拒绝无 XYZ 的非空点云，随后测试通过。失败证据保存在 `log/fast_livo2/pipeline-observer-first-failure.log`。

## 已执行验证

| 验证 | 结果与证据 |
| --- | --- |
| 官方 FAST-LIVO2 + Vikit + 非模板 Sophus 镜像编译 | 通过；`log/fast_livo2/ros1-build.log` |
| 原生 AS2 插件独立包编译 | 通过；`log/fast_livo2/as2-build.log` |
| AS2 原生宿主加载与契约测试 | 通过；位姿/速度外参变换、杆臂、时间戳、动态 TF、七类拒绝输入、停止输出与恢复；`log/fast_livo2/estimator-test.log` |
| 真实算法 + 合成 IMU/激光/图像 + 输出桥 + AS2 | 通过；30 秒输入、40 秒观察，位姿 5064 条、速度 4991 条、XYZ/RGB 点云 296 帧；`log/fast_livo2/pipeline-observer.log` |
| 算法实际执行 LIO/VIO | 日志存在两者更新、特征与稀疏视觉地图维护；`log/fast_livo2/pipeline.log` |
| 独立测试部署收尾 | 测试 Compose 容器已停止并移除；未启动 PX4、未切换原有定位源 |

执行命令与可复用入口见 `src/state_estimator/fast_livo2/README.md` 和 `deployment/fast_livo2/smoke.sh`。合成场景仅用于算法执行与接口验证，纹理和几何不构成真实标定数据集；不作定位精度声明。

## 未覆盖与后续步骤

- 真实数据集回放、精度对比、传感器标定与硬件时钟同步。
- `/clock` 回放模式的端到端验收；本轮合成测试使用系统时间。
- ROS 2 原始传感器向官方 ROS 1 工程的输入桥。当前输入须为 ROS 1 驱动或 bag。
- PX4 外部里程计融合、飞行仿真闭环、真机、多机地图对齐及资源性能验收。
- 算法 reset/地图重定位不支持无缝衔接；需停止消费者并重新启动整个估计部署。
- 输出协方差尚未实现有效估计，不应将零字段用于宣称高置信度或自动 PX4 EKF 融合。

真实数据与标定尚未在任务中提供，不能以接口测试代替上述验收。正式 Skill 应从已验证步骤提炼，未验证事项保留为边界。
