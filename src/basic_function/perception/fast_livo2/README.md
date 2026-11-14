# FAST-LIVO2 环境输出

算法源码仅保存在 `src/basic_function/state_estimator/fast_livo2/upstream/`。状态插件处理自身定位；此目录记录独立环境接口，不复制算法或重新实现地图服务器。

当前直接复用 FAST-LIVO2 的 `/cloud_registered`，通过同一输出桥发布 ROS 2 话题 `/<namespace>/perception/fast_livo2/registered_points`（`sensor_msgs/PointCloud2`）。保留全部字段、二进制数据、布局和测量时间；仅将 `camera_init` 这个局部世界坐标系标识映射为 `<namespace>/livo_world`，由状态插件发布与 AS2 odom 的恒等关系。

这是配准后的扫描／点云输出，不承诺是完整持久地图、占据图、TSDF 或 ESDF。当前不调用 `as2_map_server`，避免将三维表示投影为二维。后续消费者若需要其他表示，优先复用能满足语义的原生组件，必要时新增转换，不能仅修改消息类型。

该话题使用 SensorDataQoS，多个消费者可订阅；消费者负责检查时间新鲜度、覆盖范围及所需字段。当前输出桥使用单独 TCP 流、最新样本缓存，包大小上限 32 MiB，超限丢弃并报警；不是完整地图存储或无损离线归档通道。

上游点云时间戳使用发布时间，本项目构建补丁将其改为相应 LIO／VIO 更新时刻；点云内扫描的空间点仍有各自采样时间，消息头不意味着所有点同时采样。
