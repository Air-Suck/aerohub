# AS2 源码依赖

`aerostack2` 和 `as2_platform_pixhawk` 是个人 fork 的 Git submodule，URL 由
根目录 `.gitmodules` 管理，版本由主仓库的 gitlink 提交锁定。

```bash
bash scripts/prepare_sources.sh
```

`prepare_sources.sh` 默认仅初始化缺失的 submodule，不重置已有源码，不产生
`.source-commit` 文件。构建直接使用 submodule 的当前工作区，因此本地修改无需
先推送即可编译。记录和分享版本时，应先在两个子仓库中提交、推送，再在主仓库
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
