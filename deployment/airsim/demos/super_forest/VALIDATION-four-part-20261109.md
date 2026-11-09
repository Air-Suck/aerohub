# SUPER 四组件 Validation（2026-11-09）

结论：四方职责链路已接通，地面检查与短程飞行通过；原 forest8 第一轮失败，完整原任务尚未验收通过。没有通过重试筛选结果。机器可读指标、工件哈希与限制见 [Validation JSON](four-part-validation-20261109.json)。本报告不切换默认运行模式或通用开发入口。

## 实际职责

| 部分 | 本轮实际运行 |
| --- | --- |
| 仿真器 | PC 宿主 AirSim；碰撞真值由工作站本地 RPC 查询 |
| 飞控板 | PC `flight_board:v0.1.0` 容器，PX4 SITL、传感器与 `/clock` |
| 算力板 | RoboPi 候选 AS2、FollowTimedPath、导航输入、SUPER 与 Adapter |
| 工作站 | `osrf/ros:humble-desktop-full-jammy`，外部接口 overlay 与 `mission.py` |

工作站镜像 ID：`sha256:592f97da6b871a438fdb97f86b422ab9f80657e80efa7b98d258894738c8a0c0`，未修改或重建。仅在外部工作区补编译接口和类型支持。板端候选为 `/data/aerohub/validation/super-follow-path/as2`，保留默认安装。domain 94、drone0、PX4 system 61；DDS 显式网卡/peer，仿真 RPC 保持回环监听。

部署编排先启动板端能力并等待真实地图与 ROS 就绪，随后工作站通过 ROS 参数、enable 服务、目标话题和 Behavior Actions 执行任务。工作站没有算法/AS2 子进程（每轮 `mission_owned_children=[]`）。任务结束时板端 compute 仍存活（`capability-after-task.json`），恢复确认后才由测试部署层另行停止。本轮未把部署管理 HDC 调用伪装成 ROS 任务接口。

## 结果

| 运行 | 结果 | 关键证据 |
| --- | --- | --- |
| four-ground-01 | 通过；未请求运动 | 权威发布者、地图、时钟、Action、目标参数服务与碰撞查询前置检查 |
| four-short-01 | 通过 | 4 m 短程，误差 0.11543 m，路由 4.8301 仿真秒，Behavior 完成、停止确认、降落解除解锁 |
| four-forest8-01 | **失败** | 约 245.42 m 观测航程后 revision 222 更新被拒绝；停止、原生降落及解除解锁已确认 |

forest8 参数保持 8 m/s、8 m/s²，误差 ≤1.5 m、速度 <0.3 m/s 持续 2 仿真秒，沿用 observe 边界策略。本轮未满足原任务完成判据。最大观测速度 8.19249 m/s，因此也不宣称严格 8 m/s 限速通过。观测越界样本为零；备用标志计数不代表受控切换验收。

拒绝链：Adapter `EXECUTION_FAILED:FOLLOW_PATH_UPDATE_REJECTED:drone0`；板端 FollowPath 日志为 `invalid metadata or late trajectory`。现有日志将多个校验条件合并，缺少该次提交/接收时间和具体失败谓词，尚不能认定是时钟漂移或连续性问题。保留全部门槛，无自动重试。两侧运行期墙钟偏差采样约 -0.334～-0.263 s，未触发既有 0.5 s 门槛；这不证明不存在短时跳变或长期漂移。本轮未进一步排查系统对时。

恢复使用原生 LandBehavior，记录 Goal succeeded 和最终 disarmed。日志同时保留 LANDING 时 disarm 转换被拒、随后已解除解锁导致服务返回失败的告警，不能仅截取成功行。最终状态确认通过后才回收组件。

PVA 离线对照：forest8 记录 249 条原生计划、2546 条参考；参考均能找到源曲线候选，最大最佳匹配 P/V/A 差约 0.0000314 m、0.00180 m/s、0.0756 m/s²。匹配不是唯一 revision 映射，也不能替代更新拒绝原因定位。

## 前置失败与验证

- 固定镜像没有 `rclpy.parameter_client`：改用标准 `SetParametersAtomically`，没有替换镜像依赖，接口构造检查随后通过。
- DOUBLE_ARRAY 经 ROS wire 解码为 `array.array`，旧 list/tuple 校验误拒绝目标：板端回归先 RED，改为 ROS 参数类型检查后 GREEN；长度、有限性及空闲状态保护保留。解锁前发现并修复。
- 板端 handoff 回归通过；工作站消息类型支持、禁止启动算法和禁止直接发 PX4 应急命令检查通过；本地部署测试 8 项、Demo 测试 4 项通过。

取消、输入失流、外部控制接管、两轮 forest8 成功、长时可靠性及真机飞行均未覆盖。four-part 暂无板端失流注入 API；不得借旧拓扑结果补足。

## 可重复执行入口与证据

已有本地设备配置、候选 AS2/SUPER 和依赖准备完成后：

```bash
python3 deployment/airsim/demos/super_forest/run.py build --execution-location four-part
python3 deployment/airsim/demos/super_forest/run.py run --execution-location four-part --case ground --output log/airsim/super-four-part/NEW-ground
python3 deployment/airsim/demos/super_forest/run.py run --execution-location four-part --case short --output log/airsim/super-four-part/NEW-short
python3 deployment/airsim/demos/super_forest/run.py run --execution-location four-part --case forest8 --output log/airsim/super-four-part/NEW-forest8
python3 deployment/airsim/demos/super_forest/run.py stop --output log/airsim/super-four-part/NEW-forest8
```

`build` 在该模式仅生成外部工作区，不执行 Docker image build。必须使用新输出目录并顺序执行；stop 请求现有任务恢复，恢复未确认时保留执行端与仿真，不能强杀代替降落。

原始证据位于 `log/airsim/super-four-part/`（本地保留，未纳入 Git），各轮含 `result.json`、`source-manifest.json`、`board-deployment.json`、`images.json`、`flight.jsonl`、两侧进程身份、时差、资源及 `compute/` 日志。forest8 另存 `board-as2.log`。源码 manifest 记录受测 dirty 工作树逐文件哈希，安装 manifest 记录板端二进制哈希；不能仅用当时 HEAD 代表未提交代码。

三轮均确认 compute/AS2/relay 已停止；检查本轮容器无残留、独立 TCP/UDP 端口可绑定、两把锁可取得。候选安装、外部 overlay 和证据保留，其他部署未停止。下一步是补齐拒绝诊断、最小回归后重跑受影响阶段；通用项目开发入口另议。
