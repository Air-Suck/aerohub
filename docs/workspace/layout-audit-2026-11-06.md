# 剩余部署与 Demo 整理验收（2026-11-06）

后续进展：`v0.1.0` 已完成新镜像构建、离线及单机交互验收，见
[发布清单](../../deployment/releases/README.md)。下文保留目录审查时点的结果和当时待验证项。

检查基线为 `02ed54d`。结论：目录与源码整理应纳入仓库；本次只通过源码、入口、
配置及历史证据完整性检查，不将目录重命名记为新的 Docker 构建或端到端飞行验收。

| 内容 | 判断与处理 |
| --- | --- |
| `deployment/companion_as2` → `deployment/onboard_computer` | 保留新布局。目录、标签、入口、消息路径及运行默认值应成套提交。补齐 Dockerfile 漏掉的两个 launch 辅助文件。 |
| `apps/catora_open_field` → `demos/catora_open_field` | 保留迁移，26 个原跟踪文件均存在；算法与任务行为未因迁移改变。 |
| `apps/swarm_open_field` → `demos/swarm_open_field` | 保留迁移，18 个原跟踪文件均存在；跨 Demo 引用已更新。 |
| SUPER forest8 证据 → `demos/super_forest` | 保留迁移，14 个原跟踪文件均存在。三份原始 `.log` 增加精确 gitignore 例外，避免入仓漏失。 |
| `tools/planner-demo` | ROS 1 / Gazebo 录制与演示工具已退出活动运行链，不复制到新 Demo。由 `02ed54d:tools/planner-demo/` 保留源码，历史复现文档指明查阅方式。 |
| 旧多仓工作区索引 | 保留为历史记录，明确仓库外路径不保证在当前 checkout 存在；不作为当前运行指南。 |
| skill、交接文档与导航 | 活动入口改为 `demos/`、`onboard_computer`；交接版本表标为历史基线，实际版本以 submodule 指针为准。 |

## 已通过的检查

- 三个 Demo 所有历史 JSON 与日志和迁移前逐字节一致；保留其中当时的路径、哈希和镜像信息。
- 两套 PC 消息覆盖文件与原目录逐字节一致，三份消息 SHA256 校验通过。
- Demo / runtime Python 语法、部署 Shell 语法、Compose 配置检查通过。
- 本次涉及的本地 Docker `COPY` 源文件均存在；`run.py`、
  `selected_behaviors.launch.py`、`scoped_tf.launch.py` 一并进入新运行目录。
- CAT-ORA PC 入口、CAT-ORA 板端入口、Swarm 板端入口的 `--help` 均可从新位置运行。
- 搬迁后的 TCP 中继回归 2 项、部署布局/替换回归 2 项通过。
- 新位置的 Swarm analyzer 重放 `swarm-hybrid-ground-04` 和 `swarm-hybrid-flight-04`
  原始证据，仍返回 `passed=true`；重放结果写入临时目录，未改写历史验收 JSON。
- 当前活动文档的本地链接目标可解析。旧多仓索引中的 31 个仓库外目标属于已有历史路径，
  已明确标注，不将其视作当前部署依赖。

## 新镜像尚待验证

本机可见旧标签 `companion_as2:airsim-px4` 与
`companion_as2_dependencies:humble-px4-1.18`，未发现以下新标签：

- `onboard_computer:airsim-px4`
- `onboard_computer_dependencies:humble-px4-1.18`

因此新配方的完整构建、离线 smoke、派生 Demo 镜像重建和新位置的完整飞行尚未执行。
既有 Library 与 AS2 运行证据继续有效，但只代表其记录中的实际版本和拓扑。
`deployment/onboard_computer/README.md` 已将原 companion 镜像 ID 和验收记录标为历史，
避免它们被误认为新名称的构建证据。

完成新镜像后，可按 [onboard_computer 文档](../../deployment/onboard_computer/README.md)
执行构建和 smoke，再按 [Demo 导航](../../demos/README.md)选择需要的运行组合。
本次未启动仿真或访问 RoboPi，也未停止现有任务。
