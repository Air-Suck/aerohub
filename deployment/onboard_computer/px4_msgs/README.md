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
