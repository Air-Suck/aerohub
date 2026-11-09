# 算法能力库

算法注册入口为根目录 [deployment/airsim/libraries.yaml](libraries.yaml)。每个算法的专用
ROS 适配、代理、测试及部署文件放在同一个目录：

```text
src/planner/
├── cat_ora/
│   ├── ros2_adapter/    # ROS 包 uav_cat_ora_adapter，含脚本与测试
│   ├── humble/         # 原生 Humble 轨迹服务与 ROS 2 JSON 兼容入口
│   └── deployment/     # Humble 镜像与独立 Compose
└── swarm_formation/
    ├── ros2_adapter/    # ROS 包 uav_swarm_formation_adapter，含测试
    ├── humble/         # 原生 Humble 六包、七机 launch 与 ROS 2 JSON 代理
    ├── tests/          # 两版优化器数值/耗时与原生七机回放验证
    └── deployment/     # Humble 镜像与独立 Compose
```

ROS 1 原始源码、代理与旧部署已归档到 `src/planner/cat_ora/upstream/` 和
`src/planner/swarm_formation/upstream/`，均以 `COLCON_IGNORE` 排除；仍供数值基线验证使用。

CAT-ORA 已提取上游源码并迁移当前使用的轨迹服务，见
[迁移边界与验证](../../src/planner/cat_ora/ros2_adapter/native/README.md)；Swarm-Formation 已迁移原生规划依赖闭包，
见[迁移边界与验证](../../src/planner/swarm_formation/README.md)。公共任务契约、Planner Registry 和
任务配置仍在 `src/planner/interfaces/`、`src/planner/framework/`；公共 AS2 能力
不归属某个算法库。

独立算法部署文件位于 `deployment/<name>_humble/compose.yaml`，构建上下文
为仓库根目录。历史 Gazebo 部署保留在 `main`（原 `deployment/integration/compose.yaml`）；
CAT-ORA 的 PC AirSimNH 四机空旷路口应用已完成 Planner 飞行闭环；
Swarm-Formation 的实景飞行接入仍待完成，以注册表的 `bindings` 为准。
各运行组合与验证状态以注册表为准。
