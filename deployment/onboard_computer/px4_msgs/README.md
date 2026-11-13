# 固定 PX4 消息依赖

`onboard_computer_dependencies:humble-px4-1.18` 保存 AS2 插件所需的 PX4 消息：
生成的头文件、消息定义、类型支持库、Python 接口和 CMake 配置均安装到
`/opt/px4_ws/install`。构建后移除源码，不依赖根目录被忽略的 px4_msgs checkout。

上游基础提交固定为 `86d8239e962f6939e05c3737784f60c02fa884db`，五个消息覆盖
定义及其固件来源见 [消息兼容性说明](overrides/README.md)。
不直接跟随上游 main；升级固件时重新核对接口并重建此镜像。

```bash
docker build --network host -f deployment/onboard_computer/px4_msgs/Dockerfile \
  -t onboard_computer_dependencies:humble-px4-1.18 .
```

日常使用 `bash deployment/onboard_computer/build.sh` 构建全部依赖和 AS2。
AS2 源码改动不会改变该镜像的构建缓存。镜像的本机平台版本不能替代
OpenHarmony 交叉编译需要的目标架构消息库，但应使用相同的接口定义。

## 固定发布镜像的外部诊断 overlay

当前固定发布镜像的核心控制消息可以与 flight-board PX4 通信，但镜像内
`VehicleStatus` 和 `FailsafeFlags` 早于固件
`61907962c9220437ec01a98fa26bfbf5cbb254b5` 的定义。订阅这两个诊断 topic
会出现 DDS payload 长度不一致，不能据此判断飞行链本身失效。

不要重建或重新标记共享镜像。使用锁定的 `as2/px4_msgs` 子模块和本目录中
经过 SHA256 校验的五份定义，在固定镜像内生成外部 overlay：

```bash
PX4_MSGS_OVERLAY_WORKSPACE="$PWD/log/workspaces/px4_msgs-61907962" \
  bash deployment/onboard_computer/px4_msgs/prepare_overlay.sh
```

输出目录必须为空；脚本全程使用 `--network none`，不下载源码，不创建镜像。
运行诊断订阅或 PIN/DPV 验证器时，在基础 PX4 安装之后加载：

```bash
source /opt/px4_ws/install/setup.bash
source /px4-overlay/install/setup.bash
```

容器需将上述工作区只读挂载为 `/px4-overlay`。普通算法节点若不使用新增诊断
消息，无需加载该 overlay。升级固定 PX4 或增加消息订阅时必须重新核对 schema，
不能把这五份覆盖扩展成“整个 px4_msgs 与固件一致”的结论。
