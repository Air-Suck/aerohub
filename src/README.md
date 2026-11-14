# 源码目录与架构职责

`src/` 按 Tool、Basic Function、Behavior 三类组织；目录名不改变 ROS 包名、消息类型或运行接口。

```text
src/
├── tool/
│   └── navigation/
│       ├── planner/       # cat_ora、swarm_formation、super、gcopter、ipc、yopo、diffphys
│       ├── framework/     # 共用配置、轨迹校验、机群调用与执行协调
│       └── interfaces/    # uav_planner_interfaces 兼容协议包
├── basic_function/
│   ├── inference/         # 模型推理基础能力
│   ├── state_estimator/   # 自身状态估计
│   └── perception/        # 环境感知、建图与导航输入
└── behavior/
    ├── follow_path/
    ├── follow_reference/
    └── legacy_setpoint/
```

## 职责边界

- Navigation Tool 包含算法与任务执行协调，运动执行仍由 Behavior 承担。目录归类不表示通用 Tool 生命周期、资源互斥或 Mission/Agent 已实现。
- Inference 是基础能力。`models/` 保存各工作中模型计算部分的适配、前后处理、依赖与验证；公共 runtime、ROS Adapter 和设备后端提供在线推理服务。YOPO 轨迹重建等导航逻辑仍位于 `tool/navigation/planner/`。
- `navigation/interfaces/` 保留现有 `uav_planner_interfaces` 包。它历史上同时包含规划与 FollowTimedPath 执行协议；本次不拆包或更改类型，避免破坏现有消费者。
- AS2、PX4 和仿真器依赖保留原位置。Demo Mission 脚本和部署编排仍在 `deployment/`，卡片仍在 `cards/`。
- 尚未合入当前基线的目标检测分支不由此次整理自动合入。后续接入时，RGB-D Object Detection Tool 归 `tool/object_detection/`，YOLO 模型适配归 `basic_function/inference/models/yolo26n_seg/`。

## 路径迁移

| 原路径 | 新路径 |
| --- | --- |
| `src/planner/<算法>/` | `src/tool/navigation/planner/<算法>/` |
| `src/planner/framework/` | `src/tool/navigation/framework/` |
| `src/planner/interfaces/` | `src/tool/navigation/interfaces/` |
| `src/inference/` | `src/basic_function/inference/` |
| `src/state_estimator/` | `src/basic_function/state_estimator/` |
| `src/perception/` | `src/basic_function/perception/` |
| `src/behavior/` | 不变 |

子模块仅调整路径，锁定提交不变；`.gitmodules` 的历史 section 名保留为子模块身份，不是旧目录仍存在的声明。历史验证 JSON、源码来源记录和 `log/` 中的证据保留原运行路径，不能当作新目录上的验证结果。

已有外部构建工作区、模型虚拟环境、用户配置及运行容器可能保存旧的绝对路径、源码软链接或绑定挂载。迁移后使用已更新的部署脚本，在新的外部工作区补编译并显式加载 overlay；模型环境按各自 setup 重新准备。不要复用指向旧源码位置的 CMake 缓存，也不要仅重启旧容器就认为挂载已更新。保留现有工作区和证据，不自动清理或覆盖。

共享镜像不因本次目录调整而重建；各组件的历史验收状态不因此提升。开发约束见 [开发者贡献指导](../docs/开发者贡献指导.md)。

## 本次目录迁移检查（2026-11-14）

- 401 个 Python 文件语法检查、85 个 Shell 脚本语法检查、291 处既有文档链接迁移检查通过；40 个 ROS 包的 `package.xml`、消息/服务/Action 定义及 18 个子模块的锁定提交未改变。
- 在固定 `onboard_computer:v0.1.0` 中从新路径构建通过：`as2_navigation_inputs`、`uav_inference_interfaces`、`uav_inference`、`uav_planner_interfaces`、`uav_planner_framework`、`uav_follow_path_execution`。构建使用独立外部工作区，未重建共享镜像。
- 相关 Python 回归测试 83 项通过；其中 6 项 ROS 消息测试在接口生成并加载 overlay 后补跑通过。FollowPath 的 C++ timeline 测试 1 项通过。
- 证据位于 `log/src-layout-validation.itezrR/`，包含静态检查、测试、模型清单解析、核心包及 FollowPath 构建和测试日志。首次尝试使用宿主 UID 运行容器，因无法读取镜像 `/root/` 内的 AS2 环境而停止；后续调整容器运行权限并完成检查，首次尝试的现场目录 `log/src-layout-validation.v8MVEd/` 保留。
- 未重新运行全算法构建、GPU 模型推理、RoboPi 或 AirSim + PX4 飞行验收；本次结果只证明上述目录迁移检查通过。历史验证结果仍对应原版本和原环境。
