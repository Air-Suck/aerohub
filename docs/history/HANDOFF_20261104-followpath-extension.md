# FollowPath 扩展实现与验收记录

目标：提取执行层基础能力；兼容扩展原生 FollowPath 支持两个 Planner。后续复验已通过两个 Planner 的指定仿真场景。下方“未完成”部分保留排查时的历史状态，以本节最新结果为准。

## 最新验收结果

- 修复 Swarm 镜像构建的静默配置错误：原 sed 依赖旧值 `-12.0`，实际基础镜像却是 `point0_x=24.0`，导致算法目标与请求目标不一致。改为 `configure_waypoints.py` 按 XML 名称设置目标与航点数，缺失/重复参数直接失败。运行时 `rosparam get /drone_0_ego_planner_node/fsm/waypoint0_x` 已确认 `-4.0`。新增两项回归测试通过。
- 七机提前量 0.2 秒两次触发 FLEET_ACK_TIMEOUT，现设为 0.5 秒；按指定未来时刻采样，不改变原生曲线时序，不放宽连续性阈值。
- Swarm 冷启动成功：status=3、reason=NONE、285 段；最大跟踪误差 drone0=0.314010、drone1=0.591006、drone2=0.479841、drone3=0.615867、drone4=0.315120、drone5=0.313284、drone6=0.541548 m。执行后端为扩展 FollowPath Actions，仿真为 PX4/Gazebo。
- CAT-ORA 上轮当前执行实现冷启动成功：status=3、reason=NONE、1 段；四机最大跟踪误差 0.643、0.625、0.636、0.654 m。本轮仅修改 Swarm 部署与 Swarm 提前量，不改变 CAT 配置或执行代码。
- 最新 runtime 镜像 `0c2d5e6d6569` 构建通过，CTest 1 项、framework 24 项、CAT 3 项、Swarm 5 项、legacy adapter 2 项均通过。Swarm 镜像 `7c5cd64bb30f`。原始成功输出仍在停止的 `followpath-test-runtime:/tmp/swarm.log` 中。
- 最新 runtime 再次运行 ROS smoke 通过：原生 FollowPath 完整执行、互斥、未来启动、过期拒绝、滚动修改、暂停后重新规划、取消、轨迹耗尽及导数参考输出。使用隔离 ROS_DOMAIN_ID=83，模拟控制响应器，不能替代上述 SITL 证据。
- 不连续规划仍按失败收口，不将原生紧急停止的跳变曲线当作可执行普通轨迹。未承诺真机、任意障碍场景、严格 C² 连续或全程 0.15 m 跟踪误差。

## 已实现与证据

- `follow_path_execution` 继承上游 FollowPathBehavior，原 Action 不变；新增 FollowTimedPath Action，互斥执行、未来生效、版本、轨迹更新队列、取消、暂停后重新规划、终点位置判定与故障收口。
- 独立 timeline.hpp：线性/三次/五次采样，时间及字段校验、有界连续性检查；CTest 通过。
- framework 默认调用机群 Actions，保留显式 legacy_setpoint 对照；坐标转换、拒绝停止机群/迟到接受取消测试通过。
- Humble runtime 镜像构建通过：24 framework + 3 CAT + 5 Swarm + 2 legacy adapter 单测，以及 CTest。ROS smoke 通过原生 FollowPath 完整执行、互斥、时间、更新、取消、导数发送等。
- CAT-ORA 四机 PX4/Gazebo 经扩展 Action 返回 SUCCEEDED/NONE，任务 `3e053426-0833-47d9-a906-c084d84ebead`，1 个轨迹段。该通过发生在后续新增部分校验及 Swarm 改动之前，最终仍需复验。
- 执行能力分类与 Mission 测试场景已写主文档。没有实现完整 Mission 系统。

## 历史排查：Swarm 尚未通过时的记录

Swarm 七机尚未通过。多次冷启动发现并修正：ROS1 墙钟与 ROS2 仿真时间不一致；代理重新贴生效时间；丢失原生多项式导数；原生 PID 轨迹模式继承起飞的零水平限速；上游 start_time < 1e9 哨兵与零起点仿真钟冲突。

目前 ROS1 以固定 1700000000 秒历元加 Gazebo 时间运行，代理做时间边界转换。Swarm 返回 p/v/a，终点静止且到目标后标记 final。PID trajectory kp 调到 1.5；时间化模式生效前保持实际位置设置速度限制。加速度衔接阈值独立配置，部署 1.4 m/s²（上游每轴 max_acc=0.4 的两端变化范围）。位置阈值保持 0.15 m，不应随意放宽换取通过。

最新测试仍失败：drone3 新旧轨迹在切换处 position=0.300635 m、velocity=0.332533 m/s、acceleration=0.088348 m/s²，日志 `FOLLOW_PATH_UPDATE_REJECTED:drone3`。此前有 position=0.844278 m 的跳变。应继续核对代理只保存最新 PolyTraj 是否丢失轨迹切换边界、滚动窗口预先执行旧曲线而新曲线改变窗口内轨迹、算法重规划边界与真实执行时序。不要把它记成已支持 Swarm 的完整动力学验收。

## 远程环境

### 后续复验补充

- 新增原生曲线交接日志 `native_handoff`，分别记录原生切换时刻的位置、速度、加速度差值。代理拒绝在原生 start_time 之前采样，避免把负时间静默截成零；拒绝乱序原生曲线。代理数学回归通过。
- Swarm 新一次冷启动：212 段后 `EXECUTION_FAILED`。原生日志在 ROS1 仿真时间 1700000100.684 发出 `Emergency stop!`；1700000100.696 的 drone2 新曲线在原生切换点 dp=0.468029 m、dv=0.586259 m/s、da=0.227746 m/s²。执行更新时差值为 0.643020 m、0.585422 m/s、0.181981 m/s²。这次不是仅由插值导致：原生紧急停止曲线本身已不连续。应检查上游障碍检测/重规划失败原因以及紧急停止如何映射执行层取消，而不能放宽阈值或把停止曲线当普通连续更新。
- 当前版本 CAT-ORA 冷启动复验通过：status=3、reason=NONE、1 段，四机最大跟踪误差分别约 0.643、0.625、0.636、0.654 m。通过的是现有实际终点判定，不代表全程误差低于 0.15 m（该阈值用于轨迹交接）。
- 本地完整 framework 单测因 macOS Python 缺少 PyYAML/ROS 环境未运行成功，不能计为通过；沿用此前 Humble 构建测试证据。`git diff --check` 和本地 proxy 数学测试通过。
- 所有本轮测试容器已执行停止；镜像 Swarm 更新为 7e42ed74029a，runtime 未变。未完成事项仍为 Swarm 正常任务验收与故障语义处理。

- 本机无 Docker；SSH 别名 RKOH 可访问，`-o BatchMode=yes`。无代理端口转发告警不影响访问。
- 独立测试目录 `/tmp/uav-followpath-3A16tQ`，源码 rsync 到这里，不覆盖远端已有项目。
- 原生源码 `/home/noah/code/UAV/aerostack2-integration/aerostack2`，锁定提交 19c397309c6a41b279e6b551818ec12605cfcecc。
- 测试镜像 `uav/aerostack2-followpath-test:20261104`（当前 12afacea...）与 `uav/swarm-followpath-test:20261104`（当前 182a6cdc...）。默认项目镜像 tag 未覆盖。
- 六个容器 `followpath-test-as2/px4/runtime/clock/cat-ora/swarm`。测试 Compose 在 follow_path_execution/tests/compose.sitl.yaml；与生产基线共用端口，不能并行启动。
- 环境变量：FOLLOW_PATH_TEST_IMAGE=uav/aerostack2-followpath-test:20261104；FOLLOW_PATH_SWARM_TEST_IMAGE=uav/swarm-followpath-test:20261104；PX4_SOURCE_DIR=/home/noah/code/UAV/swarm-formation-px4-gazebo/PX4-Autopilot；PX4_SIM_CACHE_DIR=/home/noah/code/UAV/swarm-formation-px4-gazebo/.px4-sim-cache。
- runtime 内 source /opt/ros/humble/setup.bash、/root/aerostack2_ws/install/setup.bash、/opt/uav_planner_ws/install/setup.bash；复制 run_sitl_case.py 到 /tmp 后运行 `python3 /tmp/run_sitl_case.py swarm_formation`。脚本负责七机顺序起飞及任务检查。
- 两次 SSH 在测试期间断开；已检查容器与任务状态，任务终止为失败，不应把断开当作待测任务仍运行。
- 运行日志和 resolved config 在远端 `/tmp/uav-followpath-3A16tQ/artifacts/resolved`。最新容器日志保留。

用户原有文档改动很多，勿覆盖或提交。当前所有新代码未提交。后续需修复 Swarm 衔接、复验两个 Planner、更新主文档进度与证据、核查暂停/恢复及部署边界，再完成目标。
