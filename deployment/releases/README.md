# 运行镜像版本

## v0.1.1：算力侧消息与源码来源更新

2026-11-11 已发布 `ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.1`，
配套 `flight-board:v0.1.0` 保持原内容。新算力镜像编译 PX4 固定消息定义
（VehicleStatus v4）及 AS2/Pixhawk 消费者，源码直接来自 AeroHub fork。
详见 [v0.1.1.json](v0.1.1.json)。各功能已有运行默认值不随本次构建迁移自动改变；
显式设置 `ONBOARD_COMPUTER_IMAGE` 选择新镜像。

```bash
docker pull ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:ba618faf22ece6a76ffb8a0fb625b741894ef9f8f1c086a552737156a769086d
# 为使用本地镜像标签的脚本准备同一内容
docker tag ghcr.io/popc0rn7/aerohub/onboard-computer@sha256:ba618faf22ece6a76ffb8a0fb625b741894ef9f8f1c086a552737156a769086d onboard_computer:v0.1.1
ONBOARD_COMPUTER_IMAGE=onboard_computer:v0.1.1 bash deployment/onboard_computer/smoke.sh
```

远端 digest 是单平台运行 manifest 的摘要，不是原本地 OCI index ID。
此次构建的两个 fork 均干净，但镜像标签中的主仓库 revision 为 `5450d3b`、
`io.aerohub.source.dirty=true`；不能只凭这个 revision 声称精确复建已发布镜像。
本轮分支整理提供后续构建规则，不改写历史发布来源。七节点离线启动已通过；
不将其等同于所有算法、真机或全场景验收通过。

### v0.1.1 的 Skopeo 发布方式

本机 Skopeo 1.4.1 复制构建证明时出现
`unsupported MIME type for compression: application/vnd.in-toto+json`。
归档同时含 linux/amd64 运行镜像和 unknown/unknown attestation；本次明确选择
amd64，仅发布运行镜像，不使用 `--all`。这会省略 attestation，不修改运行层。
原始 tar 保留完整本地导出内容；不要为上传重建镜像。

```bash
docker save -o /tmp/aerohub-onboard-v0.1.1.tar onboard_computer:v0.1.1
skopeo --command-timeout 2h --override-os linux --override-arch amd64 copy \
  --preserve-digests --retry-times 10 \
  --dest-authfile "$HOME/.docker/config.json" \
  --digestfile /tmp/aerohub-onboard-v0.1.1-remote.digest \
  oci-archive:/tmp/aerohub-onboard-v0.1.1.tar \
  docker://ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.1
cat /tmp/aerohub-onboard-v0.1.1-remote.digest
skopeo inspect --raw --authfile "$HOME/.docker/config.json" \
  docker://ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.1 | sha256sum
```

发布已完成，上述命令仅记录过程；不要重复覆盖固定 tag。后续内容变化使用新 tag。

## v0.1.0：历史双镜像发布

`v0.1.0` 包含两份 Linux amd64 PC 镜像：

| 镜像 | 职责 | 固定本地镜像 ID 前缀 |
| --- | --- | --- |
| `flight_board:v0.1.0` | PX4 1.18 开发快照、Agent、传感器和时钟 | `8c33881fde69` |
| `onboard_computer:v0.1.0` | 当前 AS2、导航输入及算法编译开发环境 | `a99a9c1df2ca` |

完整 ID、来源和依赖见 [v0.1.0.json](v0.1.0.json)，结果见
[v0.1.0-validation.json](../../log/pr-history/deployment/releases/v0.1.0-validation.json)。两份镜像已上传到个人账号 `Popc0rn7` 的 GHCR。
这些是 PC 镜像；RoboPi 仍运行独立交叉编译的 AArch64 原生程序。

`flight_board` 沿用已验收的相同镜像内容。`onboard_computer` 从当前源码重新编译，
完成默认完整节点及精简起降/逐机 TF 启动、两库兼容构建和一次新单机飞行。
单机到点误差 0.173 m，降落并解除武装；必需传感器与 TF 均收到。
不将本次单机结果扩展为多机、无线、长期运行或所有算法场景保证。

加载交付镜像后校验：

```bash
python3 deployment/check_images.py
```

该校验器用于原始本地镜像或离线交付包，按清单核对本地镜像 ID 和平台。
GHCR 镜像按下方远端 digest 核对；本地 image ID 不等同于 registry 索引摘要。
源码开发重建可能产生不同 ID，
应使用开发标签，固定交付镜像以这里的清单为准。

离线启动复验：

```bash
bash deployment/onboard_computer/smoke.sh
AS2_BEHAVIORS=takeoff,land AS2_TF_SCOPE=per-vehicle \
  VERIFY_OUTPUT=log/airsim/onboard-scoped-check bash deployment/onboard_computer/smoke.sh
```

独立单机飞行复验（会发送仿真飞行指令）：

```bash
VERIFY_ROS_DOMAIN_ID=181 VERIFY_PX4_INSTANCE=46 VERIFY_XRCE_AGENT_PORT=8896 \
  VERIFY_AS2_FLIGHT=1 VERIFY_OUTPUT=log/airsim/runtime-v010-flight-new \
  bash deployment/onboard_computer/verify.sh
```

仍需预先配置 `.env`、AirSim binary 与匹配场景；确认 RPC 41471、TCP 4580、
Agent 8896、控制 UDP 14640/14680 未被其他任务占用。成功检查后的资源由入口回收。
内部 PX4 消息依赖镜像不作为第三份正式运行镜像交付。


## 从 GHCR 获取

镜像托管于个人账号，通过本文档与 `Air-Suck/aerohub` 关联说明；没有建立 GitHub Packages
与源仓库的正式关联，不要求它们出现在仓库侧栏的 Packages 中。
[个人 Packages 页面](https://github.com/Popc0rn7?tab=packages)。
私有包需要所有者授予读取权限，并先使用具有 `read:packages` 权限的凭据登录 GHCR；
本文不承诺匿名拉取，也不自动更改包可见性。

```bash
docker pull ghcr.io/popc0rn7/aerohub/flight-board:v0.1.0
docker pull ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.0
```

在本机 `.env` 中设置（保留其余配置）：

```dotenv
FLIGHT_BOARD_IMAGE=ghcr.io/popc0rn7/aerohub/flight-board:v0.1.0
ONBOARD_COMPUTER_IMAGE=ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.0
```

本次实际远端索引摘要：

| 镜像 | Registry digest |
| --- | --- |
| `flight_board` | `sha256:1eff3190ca7870d0267ddb1531365dd7c68d85c6e52ade3d25b17b84b4758843` |
| `onboard_computer` | `sha256:964048afd46a9d394aa771d801d48b57de8a2b2665979b1ba6f2b5e273f9adb9` |

需要严格固定远端内容时，将引用中的 `:v0.1.0` 替换为 `@sha256:...`，使用表中对应摘要。
2026-11-08 已读取远端索引并与 Skopeo 输出摘要比对。远端运行 manifest 和 attestation
manifest 的描述符与原始 OCI 包一致；顶层索引省略了原有 `mediaType` 字段，因此索引 digest
与原始本地 ID 不同。不要用本地 ID 作为远端拉取摘要，也不要要求 GHCR 拉取后通过原始 ID 校验器。

## 使用 Skopeo 发布

本次 Docker 29.9.2 / containerd image store 的 push 多次在 blob 提交时出现
`timeout awaiting response headers`，最终使用 Skopeo 1.4.1 上传成功。
这记录了本次可行绕法，不将其他注册表案例当作本次超时根因已经证实。

以 onboard_computer 为例，在已配置代理的终端执行；Skopeo 使用终端代理环境，
Docker daemon 的代理配置不会自动传给它。

```bash
docker save -o /tmp/aerohub-onboard-v0.1.0.tar onboard_computer:v0.1.0
skopeo inspect --raw oci-archive:/tmp/aerohub-onboard-v0.1.0.tar
skopeo --command-timeout 2h copy \
  --all --preserve-digests --retry-times 10 \
  --dest-authfile "$HOME/.docker/config.json" \
  --digestfile /tmp/aerohub-onboard-remote.digest \
  oci-archive:/tmp/aerohub-onboard-v0.1.0.tar \
  docker://ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.0
cat /tmp/aerohub-onboard-remote.digest
skopeo inspect --raw --authfile "$HOME/.docker/config.json" \
  docker://ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.0 | sha256sum
```

`inspect` 应读取到完整 OCI index，包含运行镜像和 attestation。`2h` 是整个复制操作的预算。
最后两条命令的摘要应一致（digest 文件带 `sha256:` 前缀）；另与发布清单核对。
尽管使用了 `--preserve-digests`，本次仍观察到顶层索引摘要变化，因此必须实测远端结果。
flight_board 将本地镜像名、tar、digest 文件名和目标路径分别改为对应名称，逐份处理。
只有上传、远端摘要和 manifest 比对完成后才删除临时 tar。不要为上传重建冻结镜像，
也不要覆盖已发布版本；源码改动应使用新版本或开发标签。
