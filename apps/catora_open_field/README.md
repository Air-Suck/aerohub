# CAT-ORA 四机 AirSimNH 空旷场地应用

使用实际 NH 场景道路交叉口中央的开阔区域，执行四机起飞、队形展开、
到点保持、降落和解除解锁。控制链路为原生 Humble CAT-ORA → 公共 Planner →
AS2 位置参考 → PX4 SITL → AirSim；不通过 AirSim 运动 API 直接控制飞机。

CAT-ORA 不负责避让场景障碍。首次运行前应查看本应用的俯视图，确认区域布局
与当前 NH 资源一致；场景变化后需要重新选址。

## 运行

按 `config/airsim/README.md` 配置 `airsim_nh` 资源，准备本地固定
`flight_board` 和 `companion_as2:airsim-px4` 镜像。仓库根目录：

```bash
python3 apps/catora_open_field/run.py build
python3 apps/catora_open_field/run.py run
```

`run` 会启动独立仿真并实际发送仿真飞行指令，结束后清理本应用的场景和容器。
证据默认在 `artifacts/catora-open-field/`，可用 `--output /absolute/path` 指定。
已有成功或失败记录的目录会拒绝覆盖，再次飞行时指定新的 `--output`。
`status` 查看当前应用，`stop` 清理中断后遗留的本应用资源。

不占用默认单机场景的 RPC/TCP 端口：本应用 RPC `42471`，PX4 TCP
`4680..4683`，控制 UDP `14840..14843/14980..14983`，PX4 实例
`40..43`、系统 ID `41..44`、Agent `8901..8904`，ROS domain `91`。
CAT-ORA JSON 接口使用本机 `28500`，运行前应保持可用。
容器名称统一以 `aerohub-catora-field-` 开头；脚本只管理这些资源和自己的场景目录。

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

## 实景验收记录（2026-11-05）

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
