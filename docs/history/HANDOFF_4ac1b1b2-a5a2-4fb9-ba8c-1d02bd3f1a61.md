# Planner 复现与 Aerostack2 集成文档审计

## 当前任务

- 在继续算力板在环方案访谈前，修复四项基线缺口：全新隔离环境从零构建 AS2、厘清 Behavior 生命周期接口、更新过时文档、增加一键启动与验收脚本。

## 已完成（要点）

- 已核对 `aerostack2-integration` 的 adapter、统一 Planner framework、AS2 setpoint adapter、Compose、manifest、任务配置和保存的冷启动证据。
- 已在 RKOH 当前 `uav/aerostack2-planner-runtime:humble` 镜像内重跑 32 项测试：framework 22、CAT-ORA 3、Swarm-Formation 5、AS2 setpoint adapter 2，全部通过；Compose 配置校验通过。
- 已确认保存的冷启动结果：CAT-ORA 四机任务 `d7e1f160-…` 和 Swarm 七机任务 `7aaa394c-…` 均记录为 `SUCCEEDED/NONE`。
- 已逐项对照主复现说明、两个工程 README、Aerostack2 使用指南、最终验收报告与当前文件结构。
- 已建立讨论记录 `notes/2026-10-09-Aerostack2算力板在环方案-grill.md`，后续每个答案即时写入。
- 已在远端创建不含三棵上游源码的隔离目录 `/home/noah/code/UAV/aerostack2-from-zero-20261009`。按旧文档直接构建可稳定复现 `COPY aerostack2/`、`COPY px4_msgs/` 和 `COPY as2_platform_pixhawk/` 缺失错误。
- 新增 `deployment/prepare_sources.sh`，仅依赖 Bash 与 Git，按固定提交拉取 Aerostack2、Pixhawk platform 和 `px4_msgs`；新增 `deployment/build_from_zero.sh` 串联源码准备、平台镜像和集成镜像构建。
- Docker 构建上下文现排除上游仓库的 `.git`，以 `.source-commit` 校验锁定版本，并用标准 `patch` 应用 Pixhawk 兼容补丁。隔离环境中的无缓存构建已通过源码校验、补丁、rosdep 和 AS2 核心包编译，仍在完成后续镜像步骤。
- 新增 `deployment/integration/run_planner.sh {cat_ora|swarm_formation}`，自动重建六服务集成环境、等待服务、执行七机起飞、启动指定任务并轮询 `SUCCEEDED/NONE`。
- 已更新 CAT-ORA 顶层 README，将过时的四机方阵变直线流程改为当前 16 机方阵到圆环流程；Aerostack2 README、使用指南和设计文档已补齐从零构建、一键验收及生命周期边界。

## 有效结论

- 两个 Planner 已接入真实 Aerostack2 执行链：AS2 Pixhawk platform、状态估计、motion controller、motion behaviors 与 `motion_reference`。算法选择、生命周期和统一轨迹契约由项目自建 `/planner/control` framework 提供。
- 当前 Planner 并非 Aerostack2 官方 Behavior Server/Action 插件。文档中的“Planner Behavior”应解释为项目自建行为语义，避免误称为原生 AS2 Behavior。
- 主复现文档 `CAT-ORA与Swarm-Formation的PX4-Gazebo复现说明.md` 已覆盖固定版本、源码准备、镜像构建、上游冒烟、PX4/Gazebo 闭环、验收、demo 和排障，内容与当前 16 机 CAT-ORA、7 机 Swarm 资产基本一致。
- `catora-px4-gazebo/README.md` 已过时：仍写四机 square-to-line 和四路验收，当前 stage2 实际是 16 机方阵到圆环。Swarm 顶层 README 较简略，但主复现文档和 stage2 README 可补足。
- Aerostack2 使用指南覆盖构建、启动、起飞、两算法调用、生命周期、配置、日志和排障，但缺少三棵上游源码的从零准备步骤。平台 Dockerfile 要求本地存在 `aerostack2/`、`px4_msgs/`、`as2_platform_pixhawk/`；虽然有 `deployment/upstream.repos`，文档没有 `vcs import` 指令。本地 Windows 副本目前也缺前两棵源码。
- 集成工程没有端到端验收脚本，完整验收仍靠人工执行起飞、START、GET_STATUS。`FINAL_VERIFICATION.md` 有历史证据，但不替代可重复的一键验收。
- 集成工程现有一键验收脚本；脚本静态语法和 Compose 配置已通过检查，待隔离环境无缓存构建结束后分别执行两个 Planner 的动力学验收。
- 完整动力学冷启动使用行为代码相同的 `1337ee…` runtime 镜像；最终 `a6abc8…` 镜像只改 manifest 元数据和校验逻辑，已重跑 32 项测试与两任务 Registry 解析，尚未原样重跑两套动力学冷启动。
- 2026-10-09 检查远端当前状态时，Compose 定义有六个服务，但 `planner-swarm-formation` 容器已不存在；其余五个容器持续运行约四天。PX4 七机日志合计约 25 GB，根盘剩余约 113 GB。当前状态不适合作为六服务健康证明。

## 卡住的问题

- 无外部阻塞。隔离环境的无缓存 Docker 构建仍在运行。

## 下一步

1. 等待隔离环境构建结束，核对新镜像和 32 项测试。
2. 在没有端口冲突时依次运行 CAT-ORA 与 Swarm-Formation 一键动力学验收，并在结束后关闭测试栈。
3. 固化从零验证证据，再继续访谈目标板环境、部署粒度、网络边界、选择语义、故障策略和验收标准。

## 踩过的坑

- Windows 本机没有 ROS 2 `rclpy`，只能直接运行 framework 的 22 项纯 Python 测试；adapter 测试需在 runtime 容器内运行。本次已在远端容器完成全部 32 项。
- Compose `Up` 只能证明容器主进程存在，不能证明全部 AS2 子节点、Planner proxy 和任务链健康；验收应检查 ROS 服务、定位、飞控状态和任务结果。
- PX4 多机 ULog 会持续增长；本次发现七机四天约 25 GB。
- `.dockerignore` 排除 `.git` 后，Dockerfile 不能再依赖 `git rev-parse` 或 `git apply`。源码版本通过准备脚本生成的 `.source-commit` 校验，补丁改用 `patch`。

## 相关指令 / 链接

- 主复现说明：`docs/CAT-ORA与Swarm-Formation的PX4-Gazebo复现说明.md`
- Aerostack2 使用指南：`docs/Aerostack2统一Planner使用指南.md`
- 集成验收摘要：`docs/Aerostack2统一Planner集成与PX4-Gazebo多机验收.md`
- 运行证据：`aerostack2-integration/artifacts/resolved/FINAL_VERIFICATION.md`
- 集成 Compose：`aerostack2-integration/deployment/integration/compose.yaml`
- 上游锁版：`aerostack2-integration/deployment/upstream.repos`
