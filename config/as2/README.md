# AS2 配置

- `config.yaml`：仿真时间、状态估计和控制器等节点参数。
- `fleet.yaml`：ROS namespace、PX4 topic 前缀和目标 system ID。
- `pid_speed_controller.yaml`：速度控制器参数。

PC companion 镜像与 RoboPi 部署包读取同一份源码配置。容器和板端路径由
各自部署入口设置；修改配置后需要按目标重新构建镜像或部署配置。
车辆名称、namespace、system ID 应与 flight_board 和 AirSim settings 对应。
