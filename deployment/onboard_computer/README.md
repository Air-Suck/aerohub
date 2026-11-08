# onboard_computer

AeroHub 正式维护的 PC 机载计算开发与仿真验证环境，承载 AS2、算法 Library
及应用任务，并作为 RoboPi 原生运行的功能参考基线。PC 验收与 RoboPi 板端验收分别记录。

基础镜像提供 ROS 2 Humble、PX4 消息接口、AS2 与开发工具；Library 和应用
通过派生镜像加入各自的算法及任务依赖，例如 `demos/catora_open_field/Dockerfile`。
通用运行代码与配置继续由 `runtime/`、`config/` 共享。该镜像随项目交付，用户可选择是否使用 PC 开发；
修改源码后可按需本地重建。Library 接入仍必须完成 RoboPi 验证。

镜像：`onboard_computer:v0.1.0`，与[固定 flight_board 镜像](../flight_board/README.md#使用固定镜像)配套。
GHCR 地址为 `ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.0`，
拉取、权限与实际摘要见[发布说明](../releases/README.md#从-ghcr-获取)。
基础环境使用 `onboard_computer_dependencies:humble-px4-1.18`，其中固定 PX4 消息已编译。
该依赖镜像基于 `osrf/ros:humble-desktop-full-jammy`，默认锁定其 digest：
`sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0`。

## 与原 AS2 镜像的区别

main 中的原 `deployment/platform/Dockerfile` 从第三方 Humble Desktop 镜像出发，
源码编译 AS2、Pixhawk 插件和 px4_msgs，再编译 DDS Agent。

本镜像改用指定的官方 OSRF 基础环境，保留原有源码编译方式，按所选 AS2
组件安装依赖。DDS Agent、PX4、传感器桥、时钟桥全部由 flight_board 提供，
不在 onboard_computer 中安装或启动。源码和编译工具保留在镜像中，便于开发。

AS2 源码现在来自 `thirdparty/` 下的两个个人 fork submodule，直接编译本地源码，
不再在 Docker 构建时应用 AS2/Pixhawk 补丁。主仓库通过 submodule 指针锁定版本；
未提交的子仓库改动也会进入本地构建，分享版本时须先提交到对应 fork。
迁移时的 fork 基础版本如下（不包含随后迁入的本地适配）：

| 源码 | Commit |
| --- | --- |
| Aerostack2 | 1cb38ab9c37671d123b6a61f81bbc91e08988466 |
| as2_platform_pixhawk | fcba062219935acef69dfa009c8be376c707ee8a |
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
在 PC 还是开发板无关。原生或交叉编译直接使用 `thirdparty/aerostack2` 和
`thirdparty/as2_platform_pixhawk` 的源码，并提供相同消息定义的目标架构 px4_msgs。
无需再次打补丁；PC 镜像里的 x86 库不能直接给 OpenHarmony 使用。
源码归属及 submodule 提交流程见 [thirdparty 说明](../../thirdparty/README.md)。

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
AS2 fork，不依赖宿主机已经安装 ROS。安装目录保留原工程路径
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

[发布清单](../releases/v0.1.0.json)、[版本验收](../releases/v0.1.0-validation.json)。
原始日志在 `artifacts/release-v0.1.0/`，测试资源已清理。校验已加载的固定交付镜像：

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
证据保存在 `artifacts/companion-as2-verification`。
以上为早期构建和启动记录；完整交互验收结果见下文。

## 实际交互验收

迁移到 thirdparty fork 后已重新编译并完成单机 AirSimNH/PX4 验收，退出码为 0。
证据位于 `artifacts/submodule-migration`，其中 `source-state.json` 和两个 `.diff`
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
证据默认在 `artifacts/as2-interaction-verification`，可用 VERIFY_OUTPUT 更换目录。

迁移前的旧 AS2 基线（`19c3973`）在 2026-11-04 已通过真实 AirSimNH + PX4 1.18
单机完整验收，脚本退出码为 0。该历史镜像 ID：
`sha256:76f376978ea60ec9c0126b378820b35eec2db298957a5de16a82dcde178c4fa4`。
证据位于 `artifacts/as2-acceptance-final`：

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
该次记录不覆盖多机和 Planner；后续算法场景验收见 [Demo 导航](../../demos/README.md)，
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
