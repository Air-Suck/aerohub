# 算法能力库

算法注册入口为根目录 [deployment/airsim/libraries.yaml](libraries.yaml)。每个算法的专用
ROS 适配、代理、测试及部署文件放在同一个目录：

```text
src/tool/navigation/planner/
├── cat_ora/
│   ├── upstream/             # 锁定的上游源码子模块
│   ├── ros1_proxy/           # ROS 1 兼容代理
│   └── ros2_adapter/
│       ├── bridge/           # 公共 Planner 契约适配
│       └── native/           # 原生 Humble 移植
└── swarm_formation/
    ├── upstream/             # 锁定的上游源码子模块
    ├── ros1_proxy/           # ROS 1 兼容代理
    └── ros2_adapter/
        ├── bridge/           # 公共 Planner 契约适配
        └── native/           # 原生 Humble 移植及测试
```

ROS 1 原始源码位于两个算法的 `upstream/` 子模块，代理位于各自的 `ros1_proxy/`；
构建时显式选择所需包，避免 ROS 1 与 ROS 2 包混入同一 colcon 工作区。

CAT-ORA 已提取上游源码并迁移当前使用的轨迹服务，见
[迁移边界与验证](../../src/tool/navigation/planner/cat_ora/ros2_adapter/native/README.md)；Swarm-Formation 已迁移原生规划依赖闭包，
见[迁移边界与验证](../../src/tool/navigation/planner/swarm_formation/README.md)。公共任务契约、Planner Registry 和
任务配置仍在 `src/tool/navigation/interfaces/`、`src/tool/navigation/framework/`；公共 AS2 能力
不归属某个算法库。

独立算法部署文件位于 `deployment/<name>_humble/compose.yaml`，构建上下文
为仓库根目录。当前运行入口、拓扑和验证状态以 `libraries.yaml` 的 `bindings`
以及 [AirSim Demo 文档](demos/README.md)为准。
