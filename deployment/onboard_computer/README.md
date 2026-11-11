# onboard_computer

AeroHub 正式维护的 PC 机载计算开发与仿真验证环境，承载 AS2、算法 Library
及应用任务，并作为 RoboPi 原生运行的功能参考基线。PC 验收与 RoboPi 板端验收分别记录。

基础镜像提供 ROS 2 Humble、PX4 消息接口、AS2 与开发工具；Library 和应用
既有快照通过派生镜像加入各自的算法及任务依赖，例如 `deployment/airsim/demos/catora_open_field/Dockerfile`。
通用 AS2 启动代码与配置由 `deployment/airsim/as2/`、`deployment/airsim/config/` 共享。该镜像随项目交付，用户可选择是否使用 PC 开发；
日常源码迭代按下节使用工作区补编译，基础镜像维护单独处理。Library 接入仍必须完成 RoboPi 验证。

镜像：`onboard_computer:v0.1.0`，与[固定 flight_board 镜像](../flight_board/README.md#使用固定镜像)配套。
GHCR 地址为 `ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.0`，
拉取、权限与实际摘要见[发布说明](../releases/README.md#从-ghcr-获取)。
基础环境使用 `onboard_computer_dependencies:humble-px4-1.18`，其中固定 PX4 消息已编译。
该依赖镜像基于 `osrf/ros:humble-desktop-full-jammy`，默认锁定其 digest：
`sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0`。

## 固定镜像与源码开发（用户确认的目标）

共享镜像作为稳定的 ROS/编译工具与依赖环境，各开发者保留自己的源码、build/install
缓存和日志。普通 AS2、算法或 Adapter 修改通过工作区补编译及显式加载 install overlay
生效，默认不重建镜像，不替换镜像内的系统库或预编译安装。镜像中已有 AS2 安装可作为
初始参考版本，实际运行选择的安装前缀与源码版本应可核对。

AS2 实现修改通常重编译受影响包即可；公共接口、C++ 基类或消息定义变化时，需连同
受影响的扩展、插件及客户端一起重编译。避免新 AS2 加载旧二进制插件，或启动入口
继续 source 镜像旧安装而遗漏开发 overlay。新增依赖先核查镜像现有内容或使用工作区
局部依赖；ROS、编译器或系统级依赖确需升级时，单独安排团队基础环境版本变更。

**当前实现差异：**现有 Dockerfile 在构建镜像时编译 AS2，entrypoint 固定 source
镜像安装，默认 Compose 只挂载配置；尚未接通上述外部源码/构建缓存及 overlay 选择。
后续应调整开发启动和补编译入口，而不是将每次源码变更都转为重建镜像。
本节记录开发约定，尚未修改这些运行入口；下文镜像构建命令用于基础环境维护或固定快照构建。

**已有实例：**[SUPER four-part](../airsim/demos/super_forest/README.md#four-part)直接复用
上述 ROS 桌面基线，外部编译接口 overlay 并显式加载，工作站只运行任务客户端；
算法与 AS2 在 RoboPi，使用官方 builder 补编译后部署候选安装。该实例没有重建共享
镜像，地面与短程已验证；它不等于通用 onboard Compose 已支持完整外部 AS2 开发工作区。

## 与原 AS2 镜像的区别

main 中的原 `deployment/platform/Dockerfile` 从第三方 Humble Desktop 镜像出发，
源码编译 AS2、Pixhawk 插件和 px4_msgs，再编译 DDS Agent。

本镜像改用指定的官方 OSRF 基础环境，保留原有源码编译方式，按所选 AS2
组件安装依赖。DDS Agent、PX4、传感器桥、时钟桥全部由 flight_board 提供，
不在 onboard_computer 中安装或启动。源码和编译工具保留在镜像中，便于开发。

AS2 和 Pixhawk 以 `.gitmodules` 声明的 AeroHub fork 为源码权威；主仓库 gitlink
锁定版本。onboard、CAT-ORA hybrid 和 OHOS 源码准备直接使用 fork，不再应用
等价的 AS2/Pixhawk 构建补丁。旧补丁和 provenance 仅保留给尚未迁移的历史
Gazebo/PX4 1.17 部署链，不作为当前 AirSim 构建输入。

本地 Docker 构建使用当前子仓库工作树；OHOS 准备继续导出主仓库锁定提交。
分享修改时先提交到对应 fork，再更新主仓库 gitlink；不自动跟随远端分支最新值。

| 源码 | 锁定 Commit |
| --- | --- |
| Aerostack2 | 1a8e52478474f25f13c0beee332fb3758c7777e9 |
| as2_platform_pixhawk | 4e5616408e0078abf38387d5b8c681f1387cfed6 |
| px4_msgs | 86d8239e962f6939e05c3737784f60c02fa884db |

PX4 接口基础提交仍固定在依赖镜像中，并覆盖三个存在布局差异的消息。
适配范围仅为本插件使用的接口，不能推广到整个 px4_msgs。
固件基线和原始定义见 [消息说明](px4_msgs/overrides/README.md)。

## AS2 交互路径

控制路径为 AS2 行为 → motion controller → Pixhawk platform → PX4 DDS →
AirSim 执行器。AS2 平台接收 PX4 的里程计、IMU、GPS、电池和控制状态，
转换成 `sensor_measurements/*`、`platform/info`；raw_odometry 再生成
`self_localization/pose`、`self_localization/twist` 和 TF。

AirSim 相机和 LiDAR 经独立传感器后端直接发布到同一架飞机的
`sensor_measurements/<sensor>/*`，供 AS2 上层模块订阅。无需在运动控制节点
里转发大图像或点云，AS2 不调用 AirSim RPC。

本次交互修复集中在：

- SensorGps 和 VehicleControlMode 使用当前固件的字段布局。
- 电池输入、姿态设定值输出使用 `_v1` topic。
- 接收 `vehicle_command_ack_v1`，记录 PX4 接受或拒绝命令；实际 armed/offboard
  仍以 VehicleControlMode 回传为准。AS2 上游 SetBool 服务的成功值不等同于 PX4 ACK。
- 电量百分比遵循 ROS BatteryState 的 0–1 范围。
- Offboard 切换前连续预发送 1.2 秒心跳，满足 PX4 的预先接收要求。
- PX4 消息的微秒时间戳使用系统时钟，匹配当前 XRCE-DDS 自动时间同步；
  AS2 的 ROS Header、TF 和行为定时器仍使用 AirSim `/clock`。直接把仿真时间戳
  交给系统时间同步链路，会让 PX4 把新发的心跳误判为过期。
- 修复 AS2 启动工具丢弃节点专属参数的问题，并显式传入 raw_odometry 插件配置，
  确保 GPS 和原点设置生效。

namespace、FMU 前缀、系统 ID 和 `/clock` 形成同一份交互约定，与 AS2 运行
在 PC 还是开发板无关。原生或交叉编译直接使用 `as2/aerostack2` 和
`as2/as2_platform_pixhawk` 的源码，并提供相同消息定义的目标架构 px4_msgs。
构建入口会在副本中应用对应补丁，不应手工修改官方子模块；PC 镜像里的 x86 库不能直接给 OpenHarmony 使用。
源码路径和两条构建链的使用边界见[项目简介](../../docs/项目简介.md#pr-目录适配)。子模块改动需先在子仓库提交并推送，再提交主仓库指针。

## 加载交付镜像

使用交付包提供的实际归档路径：

```bash
docker load -i /path/to/onboard_computer.tar
docker image inspect onboard_computer:v0.1.0
```

此处路径仅为示例；归档文件名及校验信息以实际交付包为准。
加载后可直接使用开发环境，无需先执行下节源码构建。Library / Demo 的派生镜像
按各自 README 构建；已有开发基础镜像不代表所有 Demo 依赖都已准备。

## 从源码构建和运行

原 `companion_as2` 已更名为 `onboard_computer`，目录、镜像标签和容器内入口
同步更新。已有旧镜像需按以下命令重新构建；依赖此基础镜像的 App 镜像也需
重新构建，以获得新的容器内路径。历史验收记录保留当时的镜像 ID 与证据路径。

```bash
bash deployment/onboard_computer/build.sh
docker compose -f deployment/onboard_computer/compose.yaml up -d
```

构建脚本先初始化缺失的 submodule，再构建固定 PX4 消息依赖镜像，最后编译两个
应用补丁后的 AS2 包，不依赖宿主机已经安装 ROS。安装目录保留原工程路径
`/root/aerostack2_ws/install`，供后续 App 镜像兼容使用。

同一容器按 fleet.yaml 为每架飞机启动：

- Pixhawk 平台节点。
- raw_odometry 状态估计。
- pid_speed_controller 运动控制。
- 起飞、降落、到点和路径跟随行为。

fleet.yaml 的 namespace、fmu_prefix、target_system_id 必须与 flight_board
对应。示例 drone0 对应 PX4 系统 ID 21，fmu_prefix 为 `/drone0`。
运行配置可挂载，程序与依赖固化在镜像；源码开发后需重新编译。
修改源码的本地开发构建可以直接使用 colcon，不必更换镜像名称。

AS2 使用全局 `/clock`，需要先启动 flight_board 的 clock 模式。
它不访问 AirSim RPC，不启动 Agent，也不自动解锁或起飞。

```bash
docker run --rm -it --network host --ipc host onboard_computer:v0.1.0 bash
# 容器内已加载 ROS 和 AS2 环境，可运行 ros2 或在工作区重新编译。
```

## v0.1.0 定版验收（2026-11-06）

当前镜像为 `onboard_computer:v0.1.0`，Linux amd64，镜像 ID：
`sha256:a99a9c1df2caabffa370091e48cc8190b6f0578513b913cf971a544e960471bc`。
从主仓库 `1fba658` 配方重新编译 9 个包，未把旧 companion 主镜像改标签冒充新构建。
PX4 消息依赖因源提交和三份覆盖消息字节完全一致，复用原已验证镜像；准确身份在发布清单中。

已通过默认七个节点及精简起降、逐机 TF 的两种离线启动检查，全部 use_sim_time 为 true。
CAT-ORA、Swarm 及 adapter 在该通用镜像中编译 12 包并加载通过，无需专用 compute 基础镜像。
与 `flight_board:v0.1.0` 完成新一次单机 NH 起飞、1 m 到点、保持、降落和解除武装：
到点误差 0.173 m，两秒保持漂移 0.436 m；七路图像、七路 CameraInfo、非空 LiDAR 和必需 TF 均收到。
完整多机算法验收仍以各 Demo 记录的实际镜像为准，本轮不扩展为新镜像的多机或长期验收。

[发布清单](../releases/v0.1.0.json)受版本管理；历史详细验收文件为本地
`log/pr-history/deployment/releases/v0.1.0-validation.json`，不随仓库分发，缺失时不能由该路径复核原始验收。
原始日志在 `log/airsim/release-v0.1.0/`，测试资源已清理。校验已加载的固定交付镜像：

```bash
python3 deployment/check_images.py
```

源码重建的镜像 ID 可能不同；修改源码时可设置 `ONBOARD_COMPUTER_IMAGE=onboard_computer:dev`，
将开发镜像与固定交付版本分开。以下历史记录保留原镜像 ID，不覆盖为新版本的证据。

## 构建与离线验证记录

下面的镜像 ID 和运行结果来自原 `companion_as2` 配方，作为历史证据保留；
它们不能代替新版本验收；更名后 `v0.1.0` 的新构建与启动、交互验收见上一节。

2026-11-04 已完成源码编译，9 个 ROS 包构建成功。
镜像 ID：`sha256:e6d8615e608de820556cca60886fcd78a3bc88f10b4cf07f1616ac5618fa94dc`。

```bash
bash deployment/onboard_computer/smoke.sh
```

已通过无网络的离线启动检查：platform、state_estimator、controller_manager
可发现，三者 use_sim_time 均为 true；四类飞行行为已启动，未发现启动异常；
容器不包含 MicroXRCEAgent。验证容器已清理。
证据保存在 `log/airsim/companion-as2-verification`。
以上为早期构建和启动记录；完整交互验收结果见下文。

## 实际交互验收

迁移到 thirdparty fork 后已重新编译并完成单机 AirSimNH/PX4 验收，退出码为 0。
证据位于 `log/airsim/submodule-migration`，其中 `source-state.json` 和两个 `.diff`
记录本次未提交源码的基础提交和准确变更，`live/as2-image.txt` 记录实际运行镜像。
本次定位、GPS 原点、7 路图像、7 路 CameraInfo、LiDAR 和静态 TF 全部通过；
解锁、Offboard、1.5 米起飞、1 米水平到点、2 秒悬停、降落、解除解锁全部通过。
到点误差约 0.160 米，悬停位移约 0.416 米；这些为单次验收值。
仿真场景和容器已清理。此验收不等于已重新完成 OpenHarmony 板端验证。

```bash
# 独立启动并清理 AirSim、PX4、clock 和 AS2；检查 AS2 的实际输出及传感器。
bash deployment/onboard_computer/verify.sh
# 另加模拟器中的解锁、Offboard、起飞、到点、降落与解除解锁检查。
VERIFY_AS2_FLIGHT=1 bash deployment/onboard_computer/verify.sh
```

只使用脚本创建的独立仿真资源；第二条命令会在该仿真中发送飞行指令。
飞行验收同时要求新的 PX4 ACK、固件状态回传、AS2 action 结果和实际高度变化。
证据默认在 `log/airsim/as2-interaction-verification`，可用 VERIFY_OUTPUT 更换目录。

迁移前的旧 AS2 基线（`19c3973`）在 2026-11-04 已通过真实 AirSimNH + PX4 1.18
单机完整验收，脚本退出码为 0。该历史镜像 ID：
`sha256:76f376978ea60ec9c0126b378820b35eec2db298957a5de16a82dcde178c4fa4`。
证据位于 `log/airsim/as2-acceptance-final`：

- `as2-interaction.json`：里程计、IMU、GPS、电池、定位、平台状态、全局时钟均收到；
  GPS 参数实际生效且原点已建立。静止时 NED→ENU 对照误差约 0.00023 米。
- `as2-check.log`、`as2.log`、`vehicle.log`：AS2 解锁与 Offboard 均获得 PX4 ACCEPTED
  回执和真实状态确认；1.5 米起飞、1 米水平到点、2 秒悬停、降落、解除解锁全部通过。
  本次到点误差约 0.164 米，悬停位移约 0.432 米；这些是单次测量，不是性能保证。
- `as2-sensors.json`：AS2 环境内接收到 7 路图像、7 路 CameraInfo、非空 LiDAR，
  传感器静态 TF 无缺失。
- `status.json`：结束前 PX4、Agent、传感器桥均健康；退出时已清理本次场景和容器。

行为切换需等待 `platform/info` 的 FLYING 状态传播，不能只凭 Takeoff action
刚返回成功就立即假设其他行为节点已更新状态；验收脚本包含这一等待。
仍可观察到 Humble/Jazzy/PX4 混合发现阶段的 `sequence size exceeds remaining buffer`
诊断，但该次历史单机验收的实际消息接收、回执和飞行均通过。
该次记录不覆盖多机和 Planner；后续算法场景验收见 [Demo 导航](../airsim/demos/README.md)，
也不能替代本次更名配方的构建验收。

其中 Offboard 心跳的要求参见 [PX4 Offboard 文档](https://docs.px4.io/main/en/flight_modes/offboard)。
DDS 的时间转换参见 [PX4 ROS 2 时间同步说明](https://docs.px4.io/main/en/ros2/user_guide#ros-gazebo-and-px4-time-synchronization)。
当前适配保留固件默认的 DDS 自动同步；跨主机运行时应同步 AS2 和 Agent 主机的
系统时钟。暂停、变速和重启恢复需要独立验收。

构建脚本会将调用环境中已设置的 HTTP_PROXY、HTTPS_PROXY、NO_PROXY 传给
两个镜像构建，不把代理地址或凭据写入 Dockerfile：

```bash
bash deployment/onboard_computer/build.sh
```
