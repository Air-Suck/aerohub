---
name: deploy-lib
description: 使用现有 OHOS build Docker，将 AS2 和指定算法库部署到 RoboPi，进行板端加载、启动和必要接口的简单验证。
---

# Deploy Lib

范围是将 AS2 和指定算法程序部署到 RoboPi，确认能在板端运行并完成最小接口调用。完成简单验证即结束，不扩展为 PX4/AirSim 场景编排、飞行闭环、完整算法回归或性能评估；这些属于单独任务。

## 统一布局

- `/data/aerohub/as2/install/`：AS2 与公共运行依赖；环境入口为 `/data/aerohub/as2/board_env.sh`。
- `/data/aerohub/lib/<应用名>/install/`：算法及其必要应用包；CAT-ORA 使用 `catora`，Swarm-Formation 使用 `swarm-formation`；各自的 `env.sh` 先加载 AS2 再加载应用。
- 配置和 `runtime/` 放在各自目录下。默认安装路径固定，隔离候选 AS2 见下节；新的部署替换对应 `install/`，成功后清理用于回退的临时旧安装，不积累时间戳版本目录。

复用 `tools/robopi/deploy_lib.sh --component as2|catora|swarm-formation|all` 分别更新。部署默认不切换本机 `.env`；需要切换时显式使用 `--activate-config`，成功后更新 `ROBOPI_ROOT`。迁移期间旧路径仍在被使用时，可先在独立新路径落盘，保留旧配置与运行，验收和旧目录清理另做；不停止旧消费者来腾出资源。

## 准备与构建

- 从用户请求确定目标库，检查已有 AS2、算法包及必要依赖。复用匹配的 AS2 安装，只构建缺失或需要更新的包；不默认带上整套 Planner 应用。
- 从 `.env` 和现有入口定位设备、builder、隔离 SDK、工作区及安装前缀。构建使用 [oh-ros-build](../oh-ros-build/SKILL.md)，板端交互使用 [hdc-helper](../hdc-helper/SKILL.md)。
- 组件包含需要在 Houmo 工具链量化编译的 ONNX 模型时，先使用 [model-migration-houmo](../model-migration-houmo/SKILL.md) 生成并验证 HMM；`deploy-lib` 只消费其产物、版本和目标依赖，不复制模型编译流程。
- 同步源码前检查共享锁、builder 的真实挂载及板端占用。`log/airsim/robopi-runtime/cli.lock` 不能覆盖未遵守此锁的其他入口；不同容器名或 ROS 域也不能隔离同一可写 SDK、构建目录和 overlay。与其他任务冲突时报告并让出资源，不停止对方任务。
- 记录本次源码版本和本地修改，确认工作区副本已同步，复用现有 build Docker 按依赖增量构建。具体入口见 [构建文档](../../deployment/oh-ros-build/README.md)。

## 部署

检查目标架构、动态依赖、Python ABI 与入口解释器。传输后核对哈希，先在 staging 解包并显式重定位，再替换对应持久安装目录；旧安装只临时保留供失败回退，成功后删除；系统 ROS 保持原样。替换包前确认没有运行中的消费者，若被其他任务使用则推迟部署。

归档格式、分包前缀和 Python 路径等已知问题见 [部署经验](references/robopi.md)。只处理本次程序及其必要依赖，不为简单验证额外部署仿真、传感器中继或完整控制链。

构建入口为 `bash tools/robopi/build_lib.sh catora` 或 `swarm-formation`，`all` 构建两者及必要接口包。AS2 缺失时仍使用其独立构建入口。

## 简单验证与结束

1. 在同一次板端执行中加载环境，检查 AS2 和目标库所需的模块、接口及动态库能加载。
2. 用最小配置启动相关程序，确认没有立即退出或加载错误，必要节点、话题或服务能发现。需要输入时使用有限的静态样例，避免启动完整应用。
3. 对算法执行一个小型请求或一次消息输入，检查响应成功、数量/格式及基本数值合理；如有代理，仅在本次部署包含代理时验证。此步骤不发送解锁、起飞或运动指令。

优先复用 `bash tools/robopi/verify_lib.sh as2|catora|swarm-formation|all`；它在独立 ROS 域执行最小检查并回收日志。Swarm 的单节点合成输入检查只证明部署可运行，不等同于真实传感器或七机验收。

记录命令、返回码和简短日志。缺少必要输入或同一错误没有新证据时停止重复尝试，报告具体缺项，不靠放宽超时扩大验证范围。

停止本次验证进程，复查 wrapper 的子进程、端口与锁均已释放；HDC 返回或超时后另查板端。保留已部署程序和验证日志。用户要求保留运行时按其要求交接进程身份和停止方法。

交接只报告部署位置、版本、启动及最小接口结果、未通过项；不将这些结果描述为完整集成或飞行验收。用户暂停时清理本任务后等待恢复指令。

## 隔离候选 AS2

AS2 可显式使用 `--component as2 --as2-root /data/aerohub/validation/<任务>/as2`；默认前缀不变。非默认路径只接受 AS2，且不与 `--activate-config` 共用。先用 `--prepare-only <归档>` 检查完整包集（包括 uav_follow_path_execution），再核对 ELF、Python ABI 和内容哈希。候选 board_env.sh、逐包文本前缀、manifest 与安装器使用同一前缀；仅改变解包目录不够。替换前只检查该候选路径的消费者，保留其他安装和任务。候选模式的布局单元测试不等于板端加载或飞行验收。

在已安装候选中运行诊断时，诊断脚本应由自身位置定位 install 和日志，避免继续加载默认 AS2。候选 Python 类型支持和 FollowTimedPath 可发现性均须在实际目标解释器检查；PC 构造通过不能代替 OHOS 加载。系统 ROS、默认安装和默认配置均保持原样。

SUPER 算力侧候选（适用于 `four-part` 和历史 `robopi-all`）使用 `--component as2 --include-super --as2-root <候选路径>`，
将原生算法、Adapter、框架及导航包与 AS2 一起放入隔离候选，默认不带算法。
候选携带已记录来源、版权和 SHA 的 msgpack 纯 Python fallback，以及匹配 PCL 的局部
libpng；不把 PC 的 Python C 扩展复制到 OHOS。部署完成后再执行
`deployment/airsim/demos/super_forest/tests/verify_board_load.py` 的目标端加载检查，
避免将传输结束当成 staging 安装已完成。快速端到端用例由验证 Skill 负责。

候选包含 super_app 等非空目录时，更新必须把目录、文件和 manifest 都纳入事务：
先在 staging 保留各项旧内容，逐项切换，最后切 deployment.json；失败时逆序恢复
所有已切换项。不要直接 os.replace 覆盖非空目录；runtime 证据应保留。部署失败的
staging 先独立保留再重试，不把回滚执行包误认为元数据也已全部回滚。

通用生命周期按[四方部署职责](../../docs/项目简介.md#deployment-roles)区分能力服务与工作站任务；各项目按实际入口交付，不自动照搬 SUPER 的包集合和脚本。

在 SUPER 四方实例中，板端 `compute_runtime.py` 是导航/SUPER/Adapter 的能力启动器，
不执行 Mission、解锁或仿真 RPC。`manage_mission.py --kind compute` 使用独立
状态文件管理身份；旧 mission 模式仅为历史兼容。部署层启动 AS2 与 compute，
等待真实数据和就绪状态后交接工作站 ROS 调用。任务结束不自动等于服务应停止：
先记录服务仍存活，再由拥有身份的部署层显式回收。恢复未确认时保留服务与时钟。
停止后复查包装进程及其子进程，并把 compute 证据单独收集，避免覆盖工作站结果。
