# 交付与开发环境

正式镜像交付包含 **flight_board** 和 **onboard_computer**。前者包含 PX4、DDS Agent、
Humble 传感器和时钟；后者提供可选的 PC AS2 / Library 开发环境。用户可直接加载交付镜像。[Dockerfile](flight_board/Dockerfile)保留用于记录来源与新增内容。

OHOS 开发环境由用户先按官方[设备开发环境搭建和开发指南](https://gitcode.com/openharmony-robot/docs/blob/main/device-dev/%E8%AE%BE%E5%A4%87%E5%BC%80%E5%8F%91%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA%E5%92%8C%E5%BC%80%E5%8F%91%E6%8C%87%E5%8D%97.md)准备。
仓库直接调用官方 builder，只提供项目适配脚本与说明，不维护或交付派生 builder 镜像。

## 目录与交付边界

| 目录 | 用途 | 是否为镜像交付项 |
| --- | --- | --- |
| [flight_board](flight_board/README.md) | PX4 / Agent / Humble sensors / clock 的统一仿真底座 | 是，GHCR 或离线 tar 交付 |
| [oh-ros-build](oh-ros-build/README.md) | 官方 builder 调用、工作区准备与 CMake 适配；目录名为既有脚本路径 | 否，只有脚本和文档 |
| [robopi](robopi/README.md) | 板端原生 AS2 部署、启动和诊断 | 否，部署交叉编译产物 |
| [onboard_computer](onboard_computer/README.md) | 承载 AS2、算法 Library 与应用的正式 PC 开发及仿真验证环境 | 是，PC 开发可选，源码修改后可重建 |

AirSim 场景在 PC 宿主机独立运行，binary 通过资源 YAML 配置，不包含在 tar 中。
RoboPi 原生运行 AS2，不运行 Docker。SDK、目标 ROS 和第三方依赖由官方环境提供。

当前两份正式镜像统一为 **v0.1.0**，见[发布清单](releases/v0.1.0.json)与
[验收说明](releases/README.md)。镜像已发布至 `ghcr.io/popc0rn7/aerohub/`，由个人账号托管，
通过[获取与发布说明](releases/README.md#从-ghcr-获取)提供仓库文档链接；未正式关联 GitHub Packages。

## 首次使用顺序

1. 完成官方指南，准备匹配的 SDK、目标依赖、官方 builder 和板端系统环境。
2. 执行 `bash tools/robopi/prepare_sources.sh` 初始化仓库锁定的活动 submodule。
3. 按[项目适配说明](oh-ros-build/README.md#首次初始化项目工作区)准备隔离工作区，填写 `.env`。
4. 按[固定镜像说明](flight_board/README.md#使用固定镜像)加载 flight_board tar 和校验文件，或按[GHCR 说明](releases/README.md#从-ghcr-获取)拉取固定镜像并设置 `.env`。
5. 从 [AirSim 仓库 Releases](https://github.com/microsoft/AirSim/releases) 下载对应 Linux 场景，按[场景说明](../simulator/airsim/config/README.md)配置 AirSim binary 资源，准备 PC 与板端网络。
6. 需要 PC 开发或混合 Demo 时加载交付的 onboard_computer 镜像。
7. 按[RoboPi 文档](robopi/README.md)编译、部署、启动和验收；Library 接入必须完成 RoboPi 验证，PC 验证不能替代。

PC 运行组合为：

```text
PC 宿主机：AirSim 场景
PC Docker：同一 flight_board 镜像的 vehicle 与 clock 容器
RoboPi：OHOS AS2 原生进程
```

上述 RoboPi 运行组合无需独立 sensors 或 onboard_computer 镜像。AS2 源码编译由官方 builder 执行，
与用户是否需要构建 flight_board 镜像是两件事。

## 配置与源码

共享配置位于 `deployment/airsim/config/as2/` 和 `deployment/airsim/config/dds/`，AS2 启动器与交互检查位于
`deployment/airsim/as2/`，传感器节点和订阅检查位于 `simulator/airsim/sensors/`。
各部署入口复用这些文件，不维护另外一份实现。

`.env` 保存设备、网络和构建目录，`simulator/airsim/config/resources.local.yaml` 保存外部场景路径。
两者均由用户从模板创建，不提交 Git；已有本机文件不要覆盖。
SDK、依赖快照、构建缓存与本机验收日志放在忽略目录，不属于源码交付。

历史 Gazebo/PX4 1.17 部署见[固定历史参考](../docs/项目简介.md)。
历史命令和路径不作为当前使用入口。

## 仓库验收与运行验收

仓库交付检查包括：版本指针、文档与脚本一致、依赖来源明确、目录和链接有效。
运行检查是独立证据，不能代替上述检查。

正式 RoboPi 路径使用[单机验收入口](flight_board/README.md#robopi-单机闭环验收)。
PC 开发使用 [onboard_computer](onboard_computer/README.md)，依次验证环境启动、
AS2 单机飞行和相关 Library / App 集成；RoboPi 部署仍需独立板端验收。

已验证单机仿真飞行闭环：到点误差约 0.148 米、两秒悬停漂移约 0.408 米；
600 秒 AS2 内存观察约 651.2–651.7 MiB，预热后最大增长 0.4 MiB。
详细本机记录位于忽略目录 `log/airsim/flight-board-humble-flight-acceptance/`。
这些结果不覆盖多机、长期耐久或算法 Library 接入。
