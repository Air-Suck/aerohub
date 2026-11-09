# 三个 PC 计算节点与一个 RoboPi

PC 的三个 Docker 容器分别运行 drone1/2/3 的一套 AS2、局部定位 adapter 和本机输出 adapter。
RoboPi 只运行 drone0 的一套 AS2 和本机 adapter，同时承担单实例全局 CAT-ORA、Planner 与验收程序。
AirSim、四个 PX4/Agent 与全局仿真时钟仍在 PC。

本轮复用现有消息链：FleetSetpoint 在 RoboPi 的 Planner 中统一采样，跨机经 DDS UDP
到各自 adapter，再由本机 AS2 执行。没有实现分布式轨迹采样或多算法实例独立求解。
三个 PC 节点共享宿主机网络和 x86 硬件，不能当作三个独立 ARM 板或三条无线链路。

## 复现

先确认现有本地依赖镜像，再离线构建当前 AS2 和 adapter：

```bash
docker image inspect uav/apps-catora-open-field:humble
docker build --network none --pull=false -f deployment/airsim/demos/catora_open_field/hybrid/Dockerfile \
  -t uav/catora-hybrid-compute:local .
bash tools/robopi/build_lib.sh catora
python3 deployment/airsim/demos/catora_open_field/run_robopi.py --mixed --preflight-only --output log/airsim/hybrid-ground-new
python3 deployment/airsim/demos/catora_open_field/run_robopi.py --mixed --output log/airsim/hybrid-flight-new
```

AS2 和 CAT-ORA 仍使用约定板端持久安装。运行入口自动部署当前 CAT-ORA overlay。
每个计算节点只发布本机轨迹与停止请求，四个节点的状态由任务聚合。
全局目标、速度、加速度、终点和碰撞阈值与原四机应用一致；本实验 `valid_from_lead=15 s`
为三个输出节点准备配置留出时间，属于共同轨迹起点的等待，不是算法计算耗时。

任务 resolved YAML 只在启动时通过 HDC 复制一次并校验 SHA256，结果见 `resolved-transfer.json`。
实时定位、参考、状态和服务使用 DDS。容器只读挂载该启动配置，不通过 HDC 传输控制周期。
使用前确认没有本应用已有容器；入口按名称和运行标记回收本次资源。

## 实测（2026-11-06）

索引：[hybrid-validation.json](../../../../../log/pr-history/deployment/airsim/demos/catora_open_field/hybrid-validation.json)。原始数据：
`log/airsim/catora-hybrid-ground-01/`、`log/airsim/catora-hybrid-flight-01/`。

- 四机正常任务 SUCCEEDED，全部降落并解除解锁；最大真值终点误差 0.107 m，最小实际机间距 5.886 m，无新碰撞。
- 四套 AS2 各 5 个原生节点进程；RoboPi 为 AArch64，三个 PC 容器为 x86_64。四套控制器日志均无轨迹拒收或断流。
- 执行链各节点约 48.4 Hz；PC 节点最大消息间隔约 49 ms，最大源时间年龄约 39 ms。
- 起飞前各做 20 次 ROS 服务往返：RoboPi 本地中位约 7.88 ms；三个 PC 节点约 8.05–9.11 ms，P95 约 11.44–11.87 ms。
- 实际轨迹执行窗口约 21.5 s，RoboPi 总 CPU 中位 73.5%、最高 77.9%，进程 RSS 合计约 1.48 GiB。
- 同窗口三个 PC 计算容器各约占 0.34 个 x86 逻辑核，RSS 合计约 601 MiB；不能外推为 ARM 性能。

RoboPi CPU 分组中位数以“一个核心 = 100%”计：AS2 原生进程约 112%、启动器 15%、
原生算法及代理 33%、各类 adapter 合计 224%、全局 Planner 74%、验收程序 107%。
因此本结果包含协调和观测开销，不是纯单机 AS2 负载，也不能按四板简单除四。

RTT 包含 ROS 调度和序列化；源时间年龄包含仿真时钟分发，不是纯单向网络延迟。
RSS 是进程求和，包含重复计算的共享页。PC 的 host_cpu_percent 包含 AirSim/PX4 和其他
宿主机负载，逐容器比较应使用 process_cpu_core_percent。全部原始指标在各节点的
`metrics.jsonl` 中，摘要按实际 TRAJECTORY 执行窗口统计。

本轮验证正常局域网链路，未进行断网、注入丢包、带宽限制、独立时钟漂移或长期运行测试。
