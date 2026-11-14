# ProjectAirSim 传感器适配参考

适用于本文注明的源码版本；实测数字是历史证据，不是其他任务的默认参数或验收门槛。

## ProjectAirSim 原生 bridge 核查

在源码 `92032a6c` 的 `ros/projectairsim_ros2_cpp` 接入中，复用上游 typed sensor
bridge 与 `/clock`，关闭 `publish_tf`，AS2 状态保持来自 PX4。核对实际转换代码：
向量 `(x,-y,-z)` 不等于 NED→ENU；不能仅修改 frame_id 就作为 AS2 earth 真值。
相机 optical frame、LiDAR/IMU 外参及深度定义必须分别验证后再供算法消费。

CameraInfo 要执行晚订阅和重复冷启动测试。该版本相机初始化时发布内参，
TopicManager 缓存最后值供原生订阅者获取；ROS 侧须使用 transient_local/reliable，
同时在原生 Subscribe 之前安装 bridge handler。否则立即返回的缓存可能在回调
注册竞态中丢失。可重放补丁维护在 `simulator/projectairsim/patches/`，构建时应用
到外部源码副本；真实有屏场景已通过两路标定的晚订阅及重复冷启动检查。

原生 CameraInfo K 正确而 P 为单位占位；只有确认画幅/FOV、零畸变和 R=I 后，
项目接口才规范为 P=[K|0]。真实针孔场景已验证两路标定晚订阅。ProjectAirSim
IMU 取车辆 kinematics 并忽略传感器杆臂；非零 IMU 外参应拒绝，世界系姿态未转换
时用 orientation_covariance[0]=-1 明确不可用，不能直接当 AS2 ENU 姿态。

原生 LiDAR 可在同一 stamp 发布不同空扫描/部分点云。空回波保留且报告数量，
窗口内仍须有真实有效点；配对验证用 stamp+载荷身份，不能仅按 stamp 去重或把
空帧与有点帧误配。真实 ROS 检查已验证 XYZ 载荷保持与独立 sensor TF；无逐点
时间/ring，不能把这项通过提升为 LIO 扫描语义或运动去畸变验收。

ProjectAirSim一次性CameraInfo采用TRANSIENT_LOCAL时，晚启动消费者也必须申请匹配的
持久性，否则实际深度已到达而标定始终缺失。统一标定发布与订阅契约，避免额外
volatile订阅；TRANSIENT_LOCAL订阅也接收匹配发布端的后续更新，但不匹配volatile发布端。用“先发布一次标定、再启动消费者”的真实DDS回归验证，不以连续重发
标定的fixture替代。YOPO赛道机体原点相机出现机臂遮挡，前移0.5m仅消除了遮挡，
该轮5m/s导航仍失败，不能将相机修正升级为全程避障通过。

同场景切换LiDAR导航算法时，不沿用旧平台大小写不同的传感器话题默认值。
ProjectAirSim当前标准输入为sensor_measurements/lidar1/points；IPC既有契约使用earth
注册点云、earth odom与body FLU速度，应由NavigationInputs按实际stamp和TF变换，
而非将frame_id改为earth。地面贴近占据可导致影子路径被拒绝，ready仅证明输入新鲜；
保留未通过结果，起飞与导航求解证据分开。

640×480 float 深度图出现大间隔时，先分测原生采集、桥转换/发布及 ROS 接收。
ProjectAirSim 原生 Pair0 话题是单消费者连接：不得并行连接探针并将零帧解释为相机停采。
2026-11-13 赛道实测原生最大间隔 73 ms，桥转换约 1.3 ms，但默认 DDS 接收间隔超过
1 秒；对本轮所有 ROS 参与者加载 32 MiB SHM transport 配置后，真实原始/标准图
15 秒采样最大间隔均 72 ms。可复用配置见
`deployment/airsim/demos/projectairsim_course/diffphys-dds.xml`。
这是传输分层对照证据；不替代空中策略验收，也不推断其他环境必为同一根因。
