# 第三方源码与归档

## ROS 1 算法归档

| 目录 | 原始版本 | 用途 |
| --- | --- | --- |
| `cat_ora_ros1/` | `ctu-mrs/catora` `4bc41f593eebde65dd74108a97e80430a145dfa0`，提取自 `uav/catora:noetic-upstream` | 完整原包快照、ROS 1 对照 |
| `swarm_formation_ros1/` | `ZJU-FAST-Lab/Swarm-Formation` `967a4bdfae949e994691f8ffc87dbb0147cbebb7` | 六个规划依赖包快照、ROS 1 对照 |

这两个目录是源码快照，不是 submodule；根目录的 `COLCON_IGNORE` 防止同名
catkin 包参与 Humble 构建。`aerohub/` 额外保存本项目原有 ROS 1 代理与旧部署，
旧 Dockerfile 的 COPY 路径、Compose 根构建上下文已随归档更新。归档不表示
恢复 ROS 1 为当前运行后端；活动实现位于 `libraries/<name>/humble/`。

CAT-ORA 包清单写 MIT、README 写 GPL-3.0，归档保留原文与作者；
Swarm-Formation 保留原 LICENSE 和 README。新的 Humble 部署不运行这些旧代理。

## 活动 AS2 / SUPER 源码依赖

`aerostack2`、`as2_platform_pixhawk` 和 `SUPER-sim` 是个人 fork 的 Git submodule，URL 由
根目录 `.gitmodules` 管理，版本由主仓库的 gitlink 提交锁定。

```bash
bash scripts/prepare_sources.sh
```

`prepare_sources.sh` 默认仅初始化缺失的 submodule，不重置已有源码，不产生
`.source-commit` 文件。构建直接使用 submodule 的当前工作区，因此本地修改无需
先推送即可编译。记录和分享版本时，应先在各子仓库中提交、推送，再在主仓库
提交新的 submodule 指针；仅提交主仓库不能保存子仓库中的未提交修改。

完成源码初始化后，OHOS 项目适配与编译见
[官方 builder 调用说明](../deployment/oh-ros-build/README.md)。
`companion_as2` 是可选 PC 开发路径，不属于正式镜像交付要求。

代码归属：

- `aerostack2/as2_core`：启动 YAML 保留节点专属参数；推力映射使用标准的
  `unsigned int`，兼容 OpenHarmony 工具链。
- `as2_platform_pixhawk`：PX4 1.18 话题、GPS/电池转换、命令回执、Offboard
  心跳和系统时间戳适配。直接修改 C++ 源码，构建时不再应用平台补丁。
- `px4_msgs` 不作为 submodule。OHOS 首次准备脚本获取固定源码并覆盖三个
  PX4 1.18 消息定义；日常同步保留基础源码。覆盖定义现位于
  `deployment/companion_as2/px4_msgs/overrides/`，两条开发路径共用同一份接口。
  可选 PC companion 镜像单独构建消息依赖，正式 RoboPi 运行无需该镜像。

当前 fork 的 raw_odometry 使用 `raw_odometry.use_gps` 和
`raw_odometry.gps_origin.set_origin`，配置已在 `config/as2/config.yaml`
同步更新，不能套用旧版平铺的 GPS 参数。

旧 Gazebo/PX4 1.17 源码与部署入口保留在 `main`，当前分支只初始化这里的活动
fork。历史复现可从 `main` 查看，不再通过当前源码准备脚本恢复旧部署。

## AS2 单点轨迹执行

SUPER / 公共 Planner 的轨迹接入规则与本次 PC、OHOS、NH 闭环验证见
[轨迹验证记录](trajectory_validation/README.md)。公共桥现在发布 `motion_reference/trajectory`，
由任务显式建立轨迹模式，不再由输出桥自动申请控制权。
