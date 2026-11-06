# OHOS 项目适配与编译

用户按官方指南准备开发环境后，本仓库直接使用官方
`voxelsky/ohos-ros-humble-builder` 编译 ROS 2 Humble 的 OpenHarmony AArch64 产物。
本目录仅提供调用脚本、工作区准备和 CMake 适配；不维护 Dockerfile 或派生镜像，
不交付构建工具 tar。SDK、目标依赖和源码通过挂载传入，产物用于板端原生运行。

## 前置条件

本仓库的 OHOS 构建与 RoboPi 操作均以已完成官方
[设备开发环境搭建和开发指南](https://gitcode.com/openharmony-robot/docs/blob/main/device-dev/%E8%AE%BE%E5%A4%87%E5%BC%80%E5%8F%91%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA%E5%92%8C%E5%BC%80%E5%8F%91%E6%8C%87%E5%8D%97.md) 为前提。
请先按官方指南完成基础开发环境准备，再执行本仓库的步骤；基础资源的
下载和安装以官方指南为入口，本仓库不重复分发这些资源。

本项目使用 AArch64 OpenHarmony、ROS 2 Humble 和目标 Python 3.12。
完成官方环境准备后，还需按下文组织隔离 SDK、目标依赖和源码工作区，
配置 `.env`，再运行 `run.sh check`。这些项目适配步骤与基础环境准备分开说明；
官方环境准备完成不等于本仓库工作区已经初始化。

首次使用顺序：

1. 完成上述官方指南，准备基础开发环境和依赖。
2. 执行下文的 `prepare.sh`，从官方环境初始化本项目的隔离工作区。
3. 确认官方 builder 已准备好，配置 `.env` 中的实际目录。
4. 执行 `bash deployment/oh-ros-build/run.sh check`，通过后再编译和部署。

## 使用官方 builder

本项目验证过的官方版本为 `voxelsky/ohos-ros-humble-builder:v0.1.5`，
默认通过固定摘要引用，确认本机已按官方指南准备该镜像：

```bash
docker image inspect voxelsky/ohos-ros-humble-builder@sha256:3312006a50a2a7f4a26e1c92afcfbb0fd2296989b7e6d22e71b1f77caad0d25f
```

仓库入口不会执行 `docker build` 或自动拉取镜像。缺少官方镜像时，先完成
官方指南中的获取步骤；无需创建任何本项目专用 builder 镜像。
其他官方版本需自行核对 SDK、目标 ROS 和 Python ABI 的兼容性。

`run.sh` 直接运行该官方镜像，并将本目录只读挂载到 `/opt/aerohub/ohos`，
调用仓库中的 `build_workspace.sh` 和 `cmake/Findyaml.cmake`。
构建环境变量由 `run.sh` 显式传入，镜像本身保持原样。

## 挂载输入

当前验证组合为 AArch64 OpenHarmony、ROS 2 Humble、目标 Python 3.12，
SOABI 为 `cpython-312-aarch64-linux-ohos`。其他组合需要独立验证。

| 宿主机资源 | 容器位置 | 权限 / 用途 |
| --- | --- | --- |
| `OHOS_WORK_ROOT` | `/work` | 可写的隔离工作区，布局见下文 |
| `OHOS_SYSDEPS` | `/target-out` | 只读目标依赖 `out`，不能换成 PC 的库 |

```text
OHOS_WORK_ROOT/
├── root/
│   ├── ohos-robot-toolchain/18/native/  # SDK 的隔离副本
│   ├── ros_ros2_base/                 # 匹配的 OHOS ROS 工具链 / 源码副本
│   └── install/                      # 已准备并完成路径重定位的目标 ROS 依赖
│       └── setup.bash
└── ws/
    ├── src/                          # 本次需要交叉编译的源码及其源码依赖
    ├── build/                        # 构建缓存
    ├── install/                      # 板端运行产物
    └── log/
```

原 `build-ros-humble` 会调用 `ohla patch` 修改 SDK，并处理产物路径，
因此 `/work` 必须是隔离副本；共享 SDK 原件不直接作为可写输入。
目标依赖中的绝对路径、Python/Numpy 头文件、OpenSSL、libyaml、Eigen、
pybind11 等必须与目标快照匹配。不能只创建同名空目录通过准备步骤。

## 首次初始化项目工作区

完成官方指南后，找到本机已有的四项资源。路径以自己的安装位置为准，
不要求使用本仓库开发者的目录：

| 准备入口参数 | 对应资源 |
| --- | --- |
| `--sdk-native` | OHOS SDK 的 `native/`，内含 `llvm/bin/clang` |
| `--ros-base` | 官方 `ros_ros2_base` 源码仓库，固定提交见下文 |
| `--ros-install` | 与板端系统匹配的 ROS Humble 目标 `install/` |
| `--sysdeps` | 同一目标环境的第三方依赖 `out/`，也可由 `.env` 的 `OHOS_SYSDEPS` 指定 |

官方源码仓库为 `https://gitcode.com/openharmony-robot/ros_ros2_base.git`，
本项目验证提交为 `8f588ad3c244781cc1cfc7472cfe1cbaff6702c3`。
请准备该提交的独立 checkout；入口不会切换或重置用户已有的官方源码。
若需要另取一份：

```bash
git clone https://gitcode.com/openharmony-robot/ros_ros2_base.git /path/to/ros_ros2_base
git -C /path/to/ros_ros2_base checkout --detach 8f588ad3c244781cc1cfc7472cfe1cbaff6702c3
```

在仓库根目录执行（替换示例路径）：

```bash
bash deployment/oh-ros-build/prepare.sh \
  --sdk-native /path/to/ohos-sdk/native \
  --ros-base /path/to/ros_ros2_base \
  --ros-install /path/to/target/deps/install \
  --sysdeps /path/to/target/deps/out \
  --work-root artifacts/ohos-as2
```

入口会先检查输入，再创建全新的隔离工作区，已有工作区会拒绝覆盖。
失败后保留现场；修正缺项后可选择另一个新的 `--work-root` 重试。
脚本只修改副本，完成以下项目适配：

- 复制 SDK、官方工具链源码、ROS 目标 install；重定位 install 中的路径。
- 为 SDK 副本补充目标 Python 3.12、NumPy 头文件及 `libsframe.a`，工具链增加 `/target-out` 查找路径。
- 从官方源码复制 OHOS RMW 修复包，初始化活动 AS2/Pixhawk fork。
- 下载固定提交的 px4_msgs、vision_msgs、mocap4r2_msgs 和 PID；覆盖 PX4 1.18 消息定义。
- 输出 `prepared-inputs.json`（输入目录与依赖版本）及 `current-sources.json`（源码哈希）。

首次准备需要 Git 网络访问；随后容器编译使用 `--network none`。
已有离线 Git 仓库时可加 `--source-cache /path/to/cache`，目录内应包含
`px4_msgs`、`vision_msgs`、`mocap4r2_msgs`、`pid_controller` 四个 Git 仓库，
且含锁定提交。具体提交记录在 `scripts/prepare_ohos_workspace.py` 的 `SOURCES`。

若提示缺少目标 GeographicLib、Python/NumPy、libyaml 或其他文件，应补齐
对应的 OHOS 目标依赖；不能用宿主机 Linux 库代替，也不能创建空文件绕过检查。
`--sysdeps` 指定的 out 在编译时只读挂载，必须保留在原位置或更新 `.env`。
项目历史原因及 RMW 修复说明见
[工具链准备记录](../robopi/README.md#首次工具链准备与历史经验)。

## 从仓库入口使用

复制根目录 `.env.example` 为 `.env`，填写自己的目录；已有 `.env` 不要覆盖：

```bash
OHOS_BUILDER_IMAGE=voxelsky/ohos-ros-humble-builder@sha256:3312006a50a2a7f4a26e1c92afcfbb0fd2296989b7e6d22e71b1f77caad0d25f
OHOS_WORK_ROOT=artifacts/ohos-as2
OHOS_SYSDEPS=/absolute/path/to/ros-target/deps/out
```

`OHOS_WORK_ROOT` 和 `OHOS_SYSDEPS` 的相对路径以仓库根目录为准。
`OHOS_WORK_ROOT` 必须与首次准备时的 `--work-root` 一致，`OHOS_SYSDEPS`
必须指向当时的 `--sysdeps`。已有工作区日常无需重复执行 `prepare.sh`。
先检查，再编译：

```bash
bash deployment/oh-ros-build/run.sh check
bash deployment/oh-ros-build/run.sh build as2_core
bash deployment/oh-ros-build/run.sh shell
```

`build` 后的包名传给 colcon 的 `--packages-up-to`，会构建相应源码依赖；
不提供包名则构建整个挂载工作区。入口使用 `--network none`，需要的源码和
依赖须提前准备。`shell` 用于查看挂载和诊断，不启动 ROS 应用。

本仓库 AS2 的组合入口保留为：

```bash
# 同步活动 fork 和 PX4 消息后，编译 platform、estimator、controller、Behavior
bash scripts/robopi.sh build

# 已同步源码时，直接使用 AS2 Runtime 包集合（包含 RMW 修复包）
bash deployment/oh-ros-build/build_as2.sh runtime
```

这两个入口均调用官方 builder，并复用本目录的项目适配脚本。

## 官方镜像中的项目适配

调用关系为 `run.sh` → 官方镜像中的 Bash → 只读挂载的 `build_workspace.sh`
→ 官方 `build-ros-humble --custom`。`build_workspace.sh` 提供输入检查和本项目
使用的 CMake 参数，不替代官方工具链。

`/work` 挂载隔离工作区，`/target-out` 只读挂载目标依赖，本目录只读挂载为
`/opt/aerohub/ohos`。默认 AS2 安装前缀为 `/data/aerohub/as2/install`；
构建目录为 `/work/ws/install`，部署布局见 [RoboPi 持久部署](../robopi/README.md#持久部署布局)。
应用最终位于 `/data/aerohub/lib/<应用名>/install`，打包器显式处理逐包前缀、
Python 入口与 ELF 搜索路径；不能只搬动目录就认定完成重定位。

## 输出与验证

编译后核对 ELF 为 AArch64，并使用匹配 SDK 的 `llvm-readelf` 检查动态依赖。
Python 扩展还需核对目标 SOABI；不要在 PC 上直接执行 OHOS 产物。
编译通过、架构检查通过、板端加载通过和完整闭环通过分别记录。

首次准备入口已使用本机 SDK、目标 ROS/第三方依赖和本地 Git 缓存验证：
生成新的隔离工作区，检查缺项拒绝与已有目录保护，再执行 Runtime 编译。
验证记录位于 `artifacts/ohos-prepare-verification/`，不包含板端重新部署验收。

2026-11-05：直接使用固定摘要的官方 builder，项目脚本与 CMake 适配只读挂载，
输入检查通过，AS2 Runtime 12 包增量交叉编译通过。日志位于本机忽略目录
`artifacts/ohos-official-builder-verification.log`。本轮未构建镜像、未部署或启动应用。
没有把已有工作区的成功扩大为“全新机器零准备即可运行”。实际构建与运行
记录见 [交付总入口](../README.md#验证记录)。
