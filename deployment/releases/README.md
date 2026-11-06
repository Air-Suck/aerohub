# 运行镜像版本

`v0.1.0` 包含两份 Linux amd64 PC 镜像：

| 镜像 | 职责 | 固定本地镜像 ID 前缀 |
| --- | --- | --- |
| `flight_board:v0.1.0` | PX4 1.18 开发快照、Agent、传感器和时钟 | `8c33881fde69` |
| `onboard_computer:v0.1.0` | 当前 AS2、导航输入及算法编译开发环境 | `a99a9c1df2ca` |

完整 ID、来源和依赖见 [v0.1.0.json](v0.1.0.json)，结果见
[v0.1.0-validation.json](v0.1.0-validation.json)。两份镜像已在本地确认，尚未上传远端。
这些是 PC 镜像；RoboPi 仍运行独立交叉编译的 AArch64 原生程序。

`flight_board` 沿用已验收的相同镜像内容。`onboard_computer` 从当前源码重新编译，
完成默认完整节点及精简起降/逐机 TF 启动、两库兼容构建和一次新单机飞行。
单机到点误差 0.173 m，降落并解除武装；必需传感器与 TF 均收到。
不将本次单机结果扩展为多机、无线、长期运行或所有算法场景保证。

加载交付镜像后校验：

```bash
python3 deployment/check_images.py
```

校验器按清单核对本地镜像 ID 和平台。上传后再记录 registry 返回的 digest；
本地 image ID 不代替尚不存在的远端上传结果。源码开发重建可能产生不同 ID，
应使用开发标签，固定交付镜像以这里的清单为准。

离线启动复验：

```bash
bash deployment/onboard_computer/smoke.sh
AS2_BEHAVIORS=takeoff,land AS2_TF_SCOPE=per-vehicle \
  VERIFY_OUTPUT=artifacts/onboard-scoped-check bash deployment/onboard_computer/smoke.sh
```

独立单机飞行复验（会发送仿真飞行指令）：

```bash
VERIFY_ROS_DOMAIN_ID=181 VERIFY_PX4_INSTANCE=46 VERIFY_XRCE_AGENT_PORT=8896 \
  VERIFY_AS2_FLIGHT=1 VERIFY_OUTPUT=artifacts/runtime-v010-flight-new \
  bash deployment/onboard_computer/verify.sh
```

仍需预先配置 `.env`、AirSim binary 与匹配场景；确认 RPC 41471、TCP 4580、
Agent 8896、控制 UDP 14640/14680 未被其他任务占用。成功检查后的资源由入口回收。
内部 PX4 消息依赖镜像不作为第三份正式运行镜像交付。
