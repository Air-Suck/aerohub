# DDS 配置

`fastdds_udp.xml` 是公共 UDPv4 transport 配置，由 PC 镜像和部署入口引用。
`tools/airsim/robopi.sh` 根据 `.env` 的 PC_IP 与 ROBOPI_IP 生成两端接口白名单，
写入 `log/airsim/robopi-runtime/`，不修改这里的公共模板。

两端 ROS_DOMAIN_ID 由运行配置选择；HDC 连接与 DDS 网络连通性分别检查。
