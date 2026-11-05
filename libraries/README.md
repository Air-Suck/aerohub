# 算法能力库

算法注册入口为根目录 [libraries.yaml](../libraries.yaml)。每个算法的专用
ROS 适配、代理、测试及部署文件放在同一个目录：

```text
libraries/
├── cat_ora/
│   ├── ros2_adapter/    # ROS 包 uav_cat_ora_adapter，含脚本与测试
│   ├── ros1_proxy/     # CAT-ORA 原生服务代理
│   └── deployment/     # ROS 1 集成镜像与独立 Compose
└── swarm_formation/
    ├── ros2_adapter/    # ROS 包 uav_swarm_formation_adapter，含测试
    ├── ros1_proxy/     # 状态、目标和 PolyTraj 代理
    └── deployment/     # ROS 1 集成镜像、launch 与独立 Compose
```

算法核心仍来自各自 Dockerfile 引用的基础镜像，本次没有迁入上游算法源码。
ROS 包名、节点名和镜像名保持现有约定。公共任务契约、Planner Registry 和
任务配置仍在 `planner_interfaces/`、`planner_framework/`；公共 AS2 能力
不归属某个算法库。

独立算法部署文件位于 `libraries/<name>/deployment/compose.yaml`，构建上下文
为仓库根目录。历史 Gazebo 部署保留在 `main`（原 `deployment/integration/compose.yaml`）；
当前分支的 AirSim Planner 接入尚未完成，以注册表的 `bindings` 为准。
各运行组合与验证状态以注册表为准。
