# RoboPi 部署易错点

以下来自本仓库 2026-11-05 的操作经验，实际路径与版本以当前配置为准。

| 问题 | 处理 |
| --- | --- |
| builder 已被占用 | 检查命令及挂载，等待共享工作区释放后再同步源码。只改容器名会并发修改同一 SDK 或构建目录。 |
| tar 解包路径截断、目录冲突 | 本次板端 tar 不兼容 Python 默认 PAX 格式。复用 `tools/robopi/robopi.py` 中的 `tarfile.GNU_FORMAT`、`dereference=True`，先在 staging 解包检查。 |
| 脚本存在但 `ros2 run` 报找不到 | 检查 shebang。builder 曾生成 `/data/out/bin/python3`，设备实际 Python 为 `/sys_prod/robot/out/bin/python3`。按目标 `sys.executable` 修正本次入口，保留执行权限。 |
| 源码已改但板端仍运行旧 Python 实现 | builder 的后处理会更新 install 文件时间；setuptools 可能跳过比源码更新的旧目标。构建前比较源码与已安装模块的字节，发现差异时用同一 builder 清理该包的 build/install，再重建；构建后与打包前再次逐模块比对。仅检查 import 成功、源码目录或 build/lib 都不足以证明部署更新。`tools/robopi/build_lib.sh` 和部署打包器已包含此检查。 |
| Python 接口导入失败 | 核对目标 Python ABI、ELF 和依赖。本组合为 Python 3.12 / AArch64 OHOS；不能携带 PC C 扩展充当目标扩展。 |
| 拷贝 install 后加载失败 | 使用持久布局：AS2 为 `/data/aerohub/as2/install`，CAT-ORA 为 `/data/aerohub/lib/catora/install`。部署打包器逐包重定位文本前缀，清理 ELF 中构建机的搜索路径并检查架构与动态依赖不变，随后按对应环境入口加载；不能只移动文件。 |
| 重新启动时端口占用 | `ros2 run` wrapper 退出不一定代表子进程退出。按本次 PID 身份和运行标记清理后代，避免全局杀进程；HDC 超时后先查远端再重试。 |
| 服务可发现但输入未就绪 | 核对程序实际依赖。使用仿真时间的程序需首帧 `/clock`；简单离线服务验证应使用程序支持的时钟配置，不为此启动完整仿真。 |

构建与 AS2 安装约定分别见 [oh-ros-build](../../../deployment/oh-ros-build/README.md) 和 [robopi](../../../deployment/robopi/README.md)。

## CAT-ORA 的简单验证

原生包是 `mrs_formation_reshaping`，源码在 `src/planner/cat_ora/ros2_adapter/native/`。最小验证启动 `cat_ora_node`，调用 `/mrs_formation_reshaping/get_reshaping_trajectories`；如果部署包含 `ros2_proxy.py`，再检查其本地 JSON 接口。AS2 仅做所需模块加载、最小启动检查，不启动四机任务。

`src/planner/cat_ora/ros2_adapter/native/tests/verify_board.py` 可参考其环境与启动方式，但它执行多组数值回归；简单部署验证不必全部运行。已有板端服务证据见 `log/pr-history/src/planner/cat_ora/validation/robopi-service.json`，只代表记录中的版本与范围。

`deployment/airsim/demos/catora_open_field/run.py` 和 `run_robopi.py` 会编排场景并发送飞行指令，不是本 skill 的简单验证入口。算法不同则选择自己的最小服务或消息样例，不照搬 CAT-ORA 的包集合和接口。

## 新布局部署入口

`bash tools/robopi/deploy_lib.sh --component all` 部署 AS2、CAT-ORA 与 Swarm-Formation；`as2`、`catora`、`swarm-formation` 可单独更新。脚本从已准备工作区的 `ws/install` 取产物，核对包内容及传输哈希，staging 解包后替换持久目录。成功后移除临时旧安装，保留 `deployment.json` 记录包和文件哈希。目标目录被消费者使用时拒绝替换。

默认只部署，不修改旧任务正在使用的 `.env`。指定 `--activate-config` 才在成功后更新 `ROBOPI_ROOT=/data/aerohub/as2`。`tools/robopi/robopi.sh deploy` 面向 AS2，包含配置切换；迁移期间只准备新布局时使用前述不带激活选项的入口。

两套环境分开：AS2 加载 `/data/aerohub/as2/board_env.sh`；CAT-ORA 加载 `/data/aerohub/lib/catora/env.sh`。历史 tmp 目录和历史验证记录不再作为新部署默认位置；清理旧目录需等其消费者停止并完成约定的验收。

## 当前统一简单验证

`bash tools/robopi/build_lib.sh all` 使用官方 builder 构建两库；`bash tools/robopi/verify_lib.sh all` 依次检查 AS2、CAT-ORA、Swarm-Formation，不启动仿真或发送运动命令。Swarm 环境入口为 `/data/aerohub/lib/swarm-formation/env.sh`，六个原生规划包和所需应用包位于该目录下的 `install/`。

AS2 直接启动估计器时，需要同时加载安装包中的 `config/state_estimator_default.yaml` 和 `plugins/raw_odometry/config/plugin_default.yaml`；只有 `plugin_name` 不足以初始化插件必填参数。不要把缺少配置误判为部署或 CPU 故障。

Swarm 在 OHOS 构建时使用标准 Eigen 头路径；点云转换头文件包会导出未使用的 PCL IO 库，当前实现单独声明头文件/消息依赖，只链接实际使用的 PCL common。首次板端规划还发现原有 `desired_start_time` 未初始化：Clang 优化后读取空旧轨迹并崩溃。初始化首轮不使用的占位值后，单节点首条 PolyTraj 检查通过。

2026-11-05 的持久布局简单验收：AS2 五秒启动与消息序列化、CAT-ORA 一次服务/JSON 请求、Swarm 单节点静态输入到两段五阶轨迹均通过。记录见 `log/pr-history/deployment/robopi/validation/startup.json`、两个库的 `validation/robopi-smoke.json`。这些不代表完整回归、真实点云或飞行验收。
