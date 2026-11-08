# CAT-ORA 四机 AirSimNH 空旷场地应用

使用实际 NH 场景道路交叉口中央的开阔区域，执行四机起飞、队形展开、
到点保持、降落和解除解锁。控制链路为原生 Humble CAT-ORA → 公共 Planner →
AS2 单采样轨迹参考（PVA）→ PX4 SITL → AirSim；不通过 AirSim 运动 API 直接控制飞机。

CAT-ORA 不负责避让场景障碍。首次运行前应查看本应用的俯视图，确认区域布局
与当前 NH 资源一致；场景变化后需要重新选址。

## 运行

按 `config/airsim/README.md` 配置 `airsim_nh` 资源，准备本地固定
`flight_board` 和 `onboard_computer:v0.1.0` 镜像。仓库根目录：

```bash
python3 demos/catora_open_field/run.py build
python3 demos/catora_open_field/run.py run
```

`run` 会启动独立仿真并实际发送仿真飞行指令，结束后清理本应用的场景和容器。
证据默认在 `artifacts/catora-open-field/`，可用 `--output /absolute/path` 指定。
已有成功或失败记录的目录会拒绝覆盖，再次飞行时指定新的 `--output`。
`status` 查看当前应用，`stop` 清理中断后遗留的本应用资源。

### RoboPi 运行完整应用

AS2 与 CAT-ORA 使用 `deploy-lib` 的持久布局；构建使用现有 OHOS builder。
PC 保留 AirSim、四个 PX4/Agent Docker 和 `/clock`，CAT-ORA 原生服务、代理、
Planner、轨迹桥和四机 AS2 均运行在 RoboPi。先配置 `.env` 的 HDC、网络和工作区。

```bash
bash scripts/build_lib.sh catora
python3 demos/catora_open_field/run_robopi.py --preflight-only --output artifacts/catora-ground-new
python3 demos/catora_open_field/run_robopi.py --output artifacts/catora-flight-new
```

`--preflight-only` 只做地面定位、时钟和原生规划到轨迹桥的输入检查，不解锁。
完整运行会发送四机起降和轨迹接管指令；只有 `result.json` 的完整验收条件
通过才算飞行成功。启动成功或 `pipeline-probe.json` 不代表飞行通过。
运行入口会替换 CAT-ORA 安装；AS2 需提前构建并用
`bash scripts/deploy_lib.sh --component as2` 部署，已有消费者时拒绝替换。

板端应用使用 `/data/aerohub/lib/catora/validation/run-<编号>` 保存本次辅助文件，
程序仍加载 `/data/aerohub/as2/install` 和 `/data/aerohub/lib/catora/install`。
每次运行的 `topology.json` 记录准确路径、ROS 域和环境，`results/processes.json`
记录轨迹接管前的 AArch64 进程。控制状态以 PX4 原始局部里程计的 `<机名>/odom`
为准，转换成 Fleet NED 后规划，再由桥转回该机局部 ENU；AS2 的 earth 定位
不混入轨迹坐标。接管前等待原生算法输出和四机桥输入准备完毕，显式请求
TRAJECTORY 模式；桥仅在四机均确认模式和航向后发送单采样 PVA，保留采样时间。
AS2 原有的过期轨迹保护保持启用。此应用只启动 Takeoff、Land 两类 motion behavior。

结束时按本次 `CATORA_APP_ROOT` 标记与 PID 身份清理板端后代，回收日志及
`board-cleanup.json`，关闭本应用 PC 场景、容器和只读真值 RPC 中继（42473）。
保留每次成功或失败的证据目录，便于比较，不覆盖历史结果。

不占用默认单机场景的 RPC/TCP 端口：本应用 RPC `42471`，PX4 TCP
`4680..4683`，控制 UDP `14840..14843/14980..14983`，PX4 实例
`40..43`、系统 ID `41..44`、Agent `8901..8904`，ROS domain `91`。
CAT-ORA JSON 接口使用本机 `28500`，运行前应保持可用。
容器名称统一以 `aerohub-catora-field-` 开头；脚本只管理这些资源和自己的场景目录。

## 三个 PC 节点与一个 RoboPi

使用 `run_robopi.py --mixed` 可将 drone1/2/3 的 AS2 和本机 adapter 放入三个 PC
容器，RoboPi 只运行 drone0 的 AS2，同时保留全局 CAT-ORA/Planner。地面通信与
完整任务均已验证，复现、负载口径和边界见 [混合部署](hybrid/README.md)。

## 仿真启动就绪

RoboPi 入口按 `.env` 的 `FLIGHT_BOARD_IMAGE` 选择 PC 镜像，`images.json` 记录实际 ID。
AirSim 保留 TCP 4680..4683；本应用通过回环 TCP 4780..4783 转发 MAVLink，
仅在连接建立后缓冲首包 100 ms；两端均启用 TCP_NODELAY，之后双向字节原样转发，不添加逐包等待。
`px4-settings.json` 只将 PX4 连接端口指向转发端口，场景 settings 保持原值。
板端任务开始前须四架 PX4 均报告 Ready for takeoff，结果在 `px4-startup.json`；
25 秒仍缺项就失败清理，不启动板端飞行任务，不自动重试掩盖缺失。

此措施针对 NH 所带 AirSim 的启动竞态：TCP 读取先于心跳回调注册，偶发漏掉
首心跳，随后 AirSim 不发送 HIL 传感器，PX4 持续 poll timeout，最终没有定位。
在固定旧镜像、同四机配置下，仅 PC 的最小对照三次无等待有一次失败，
增加 100 ms 首包缓冲后三次均通过。证据分别为
`artifacts/catora-handshake-before/` 和 `artifacts/catora-handshake-after/`。
这是应用侧启动缓解和就绪检查，没有修改 AirSim 或 PX4 二进制；三次通过不是长期可靠性保证。
转发器最初未禁用 Nagle，引入最大约 41.9 ms 小包排队；启用双向 TCP_NODELAY 后
同条件最大约 3.3 ms。真实小包负向对照和回归测试保留在 `artifacts/catora-relay-*`，
避免只验证 TCP 能通却影响 HIL 传感器节拍。

```bash
source scripts/load_env.sh
python3 demos/catora_open_field/verify_sim_startup.py --output artifacts/catora-startup-new
```

该验证只使用 PC AirSim/PX4，不部署、不访问 RoboPi，也不解锁起飞。
`tests/test_sim_startup.py` 验证首包等待及后续载荷透明转发。

## 板端多机 TF 对照

四机应用默认 `--tf-scope per-vehicle`，只调整 ROS 话题 remapping，保持坐标与控制参数。
共享 TF 时 4 个发布者、20 个订阅者构成 80 路分发；隔离后每机 1 发布、5 订阅，
合计 20 路。CAT-ORA 输入直接消费局部里程计，不依赖全机群 TF。
同条件地面对照中，任务侧轨迹接收约 18.7 → 30.6 Hz，观测到的最大时钟超前
约 57 → 21 ms；CPU 两组仍接近满载，不将其描述为已获得充分 CPU 余量。
独立观测与实际订阅图在 `artifacts/catora-tf-shared-02/`、`artifacts/catora-tf-scoped/`。
AS2 源年龄、未来偏差及失流保护没有放宽。后续增加节点或负载需要重新验证时序。

## CAT-ORA 适配与停止验证

局部定位转换归属 CAT-ORA 自有 `cat_ora_state_adapter_node`，应用不再发布定位中继。
输入先校验局部 frame、源时间与有限值，再供规划和到点判断使用。
`cat_ora_as2_adapter_node` 复用 PVA 转换，取消／暂停／失败／成功时将停止意图
转为真实 AS2 HOVER 服务请求，要求响应成功和模式反馈；停止确认完成后保留结果，
不因随后的降落模式撤销确认。Swarm 继续使用原公共输出入口。
接口与测试说明见 [CAT-ORA adapter](../../src/planner/cat_ora/ros2_adapter/README.md)。

应用异常时先请求 AS2 悬停，尝试取消规划，再通过 AS2 降落；失败结果写入
`results/failure-stop.json`，不会把清理进程当作停止成功。AS2 本身的失流保护仍启用。

```bash
python3 demos/catora_open_field/run_robopi.py --scenario cancel --output artifacts/catora-cancel-new
python3 demos/catora_open_field/run_robopi.py --scenario stream-loss --output artifacts/catora-loss-new
```

这两个命令均为真实四机飞行验证。`cancel` 在实际轨迹输出后取消规划，等待 AS2
悬停确认，再降落。`stream-loss` 暂停实际输出 adapter 进程，等待 AS2 自身失流
悬停，恢复进程后通过与异常相同的停止、取消及降落路径收尾。
后者保存 `fault-event.json` 和 `failure-stop.json`；只有故障已实际观测、停止和
降落均确认成功才生成通过结果。两者不等同于正常任务到点成功。
每次还记录板端安装清单、构建输入哈希、PX4/Agent 和 clock 容器日志。

## 任务与测量

出生点在共享 NED 中为 `(0,-3)`、`(0,3)`、`(6,-3)`、`(6,3)`，各自起飞至
3 m。规划目标前移 3 m、左右各展开 1 m，保持高度；最大速度 0.8 m/s、
最大加速度 0.5 m/s²。目标归属固定，原 adapter 负责拒绝 CAT-ORA 换配结果。

每台起飞需 AS2 action、PX4 ACCEPTED ACK、实际状态及高度反馈；Planner
必须报告 SUCCEEDED，四机误差不超过 0.6 m，保持两秒，观测机间距离不小于
2 m，规划飞行阶段无新的 AirSim 碰撞；随后降落并确认四机解除解锁。
应用在地面静止定位稳定后，根据 AirSim 实际位置与 AS2 局部定位的同次观测
计算固定坐标原点，写入运行目录 `deployment.yaml`，不修改共享部署配置。
目标仍采用场景共享 NED；另要求 AirSim 实际终点误差不超过 0.75 m，避免
AS2/GPS 高度原点偏差让自身反馈通过而实际目标未到达。

`result.json` 保存终点误差、规划阶段观测的最小机间距离、最大采样跟踪误差、
实际墙钟耗时和降落结果。`flight.jsonl` 记录 10 Hz 仿真时间与共享 NED 位置。
每秒另存 AirSim 实际位置，报告独立终点误差、机间距离及与定位的最大差异。
碰撞按 AirSim timestamp 区分起飞前地面接触与规划飞行期间的新碰撞。
`ground.png`、`airborne.png`、`goals.png`、`landed.png` 为固定俯视相机截图。
`images.json`、各进程日志与 `scene/` 保留实际镜像和场景启动记录。

此应用测量完整规划/控制链路；单次通过不代表复杂障碍、交叉换位、通信延迟
或长时间飞行已验收。Swarm-Formation 不在本应用的验证范围内。

## 当前适配验收（2026-11-06）

最终验收索引为 [stability-validation.json](stability-validation.json)，三次使用相同
板端安装清单、当前应用脚本、`.env` 指定的 PC 镜像和逐机 TF 配置：

| 路径 | 结果 | 本机证据 |
| --- | --- | --- |
| 正常任务 | SUCCEEDED，四机到点、悬停、降落并解除解锁 | `artifacts/catora-stability-normal-08/` |
| 主动取消 | AS2 悬停确认约 0.103 s，无断流，四机降落解锁 | `artifacts/catora-stability-final-cancel/` |
| 实际输出进程暂停 | 四机日志均记录断流悬停，异常收尾取消规划并降落解锁 | `artifacts/catora-stability-final-stream/` |

正常任务的 AirSim 真值终点误差：drone0 0.117 m、drone1 0.103 m、
drone2 0.109 m、drone3 0.102 m；最小实际机间距 5.870 m，无新碰撞、轨迹拒收
或断流。局部定位终点误差最大 0.092 m，最大采样跟踪误差 1.085 m，定位与真值
最大采样差异 0.432 m；这些指标不能混为终点误差。

五组原生服务／JSON 轨迹继续满足既有 ROS 1 数值基线的 1e-9 m 容差，
包含此前明确记录的参数顺序修正；七类非法请求及恢复通过。板端真实 ROS 进程
测试另覆盖非法定位、暂停恢复、停止拒绝、停止后降落状态保持、迟到 RPC 和
wrapper 遗留子进程回收。错误、取消和正常完成均调用 AS2，不靠终止进程冒充停止。

失流测试观测延迟约 0.667 s，包含消息与反馈调度；配置仍为最后有效样本后
500 ms 断流，不将观测延迟解释为修改后的超时。正常与主动取消日志没有断流。

最终 `artifacts/catora-stability-cleanup.json` 确认本任务板端进程、28500/28509、
PC 容器、场景及转发端口都已释放，共享锁空闲；持久安装保留。
该结果仅覆盖固定 NH 四机场景。CPU 余量仍有限，不代表长稳或增加相机、点云、
飞机数量后的容量验收。PX4 飞前检查或实际 ACK 拒绝仍会失败并安全收尾，未关闭检查。

## 首轮 RoboPi 完整飞行验收（2026-11-06）

四机原生板端链路通过，见 `robopi-validation.json`。完整证据为
`artifacts/catora-robopi-full-06/`，其中 `result.json` 为结果，
`results/processes.json` 为板端进程，`results/` 含完整日志与四阶段截图。

| 飞机 | 局部里程计终点误差 | AirSim 真值终点误差 |
| --- | --- | --- |
| drone0 | 0.033 m | 0.152 m |
| drone1 | 0.041 m | 0.187 m |
| drone2 | 0.036 m | 0.197 m |
| drone3 | 0.040 m | 0.146 m |

Planner 报告 SUCCEEDED，四机均降落并解除解锁；最小实际机间距 5.882 m，
规划阶段无新碰撞。桥向每机实际发布 191 条轨迹参考，任务观察节点收到
105–124 条；观察计数受 best-effort 丢样影响，不作为实际发布率。
AS2 日志未出现轨迹拒收或流丢失。AS2 300 ms 样本年龄、50 ms 未来时间及
500 ms 断流保护使用默认值。

CAT-ORA 原生日志报告本次生成计算 0.35 ms、轨迹时长 5.80 s。
从 START 请求至终点核验墙钟 23.36 s，包含 adapter 启动、规划、执行和保持，
不能把它当作算法计算耗时。最大采样跟踪误差 1.092 m，定位与真值最大采样
差异 0.401 m；这些指标与终点误差不同。一次通过不代表压力或长期稳定性验收。

排查时修正了 Python install 新时间戳掩盖旧代码的问题，并为桥加入只读准备
状态，接管前核对实际输出节点。地面观察发现调度压力，AS2 `spinLoop` 复用
executor，应用只启动所需起降行为，状态消费者保留最新样本。未放宽控制保护，
也未修改 CAT-ORA 算法数值参数。executor 修改位于 `as2/aerostack2`
子仓库的工作区；提交时需单独保存子仓库改动，主仓库提交不能包含其未提交源码。
这些组合修正后完成本次飞行，不把改善单独归因于某一项。

`cleanup-verified.json` 确认板端本次进程、28500 端口、PC 本应用容器及仿真
端口均释放，场景停止、共享锁空闲。持久安装保留供后续使用。

## PC 容器实景验收记录（2026-11-05）

四机在 NH 实际道路交叉口完成任务，Planner 报告 SUCCEEDED，四机均降落并
解除解锁。版本化摘要见 `validation.json`，完整本机证据：
`artifacts/catora-open-field-final/result.json`，同目录
包含原点校准、10 Hz 位置记录、每秒 AirSim 实际位置和四阶段俯视截图。

| 飞机 | AS2 终点误差 | AirSim 实际终点误差 |
| --- | --- | --- |
| drone0 | 0.043 m | 0.077 m |
| drone1 | 0.146 m | 0.117 m |
| drone2 | 0.172 m | 0.090 m |
| drone3 | 0.160 m | 0.043 m |

规划阶段墙钟耗时 15.20 s，观测的最小实际机间距离 5.63 m，无新的规划阶段
碰撞。AS2 与实际位置最大采样差异 0.306 m；相对当前发布 setpoint 的最大
采样跟踪误差 1.025 m，和最终到点误差是不同指标。

早期一次运行因应用误用未带版本号的 ACK topic 而在解锁回执等待处退出，
现按 `VehicleCommandAck.MESSAGE_VERSION` 订阅实际 topic。
另一次仅用 AS2 自反馈的验证暴露地面局部高度原点偏差，最终版本加入固定
原点对齐及独立 AirSim 终点检查后通过。算法及 AS2 控制参数未为本案例修改。
