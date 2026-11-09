# AS2 / SUPER 轨迹执行验证

接口与使用说明见 [公共桥 README](../../planner_framework/bridges/as2_setpoint/README.md)。

SUPER 长程端到端任务、配置及验收记录已集中到 [SUPER forest8 Demo](../../demos/super_forest/README.md)。
本目录保留公共轨迹接口和失流保护的专项验证。

2026-11-05 的真实验证结果：

- PC Humble：AS2 core、控制器、Pixhawk 平台、运动行为构建通过。
- OHOS AArch64 / Python 3.12：上述运行包交叉编译通过，部署到现有 RoboPi 私有 overlay。
- 数值/时间契约：单点、有限位置与 yaw、可选三维 NaN、ENU→NED、重复/倒序/过期/
  未来时间、模式进入前参考、20/50/100 Hz 最新值执行测试通过。
- 实际控制器：20/50/100 Hz 各输入 1.2 秒，均输出 60 次；源时间戳保留，重复参考不续期，
  失流调用悬停，随后新参考不能自动恢复。见 `2026-11-05/stream-tests.log`。
- 真实 ROS 消息下的 adapter / 任务启动回归共 7 项通过，含 SUPER 主/备用轨迹、
  固定/跟随航向、仅位置 NaN 和局部里程计。见 `2026-11-05/adapter-tests.log`。
- NH 完整链：PC SUPER → RoboPi AS2 → PX4，AS2 起飞、显式轨迹接管、SUPER 到点、
  悬停、AS2 降落及 firmware armed=false 均通过。速度上限 1 m/s。
  见 `2026-11-05/mission.log` 与 `flight-summary.json`。
- 捕获的 152 条完整 PX4 PVA 均可匹配源参考的 ENU→NED 结果；捕获的 146 条 actuator
  参考全部保留源时间戳；固定 yaw 的变化为 0。DDS 采用 best effort，捕获计数受发现延迟
  和丢包影响，不用它代替执行频率计量；观测发送间隔中位数约 20.7 ms。

验证中修正了位置起飞插件的非零原点完成条件：目标是绝对高度，完成条件应比较目标绝对
高度与绝对反馈，不能直接拿相对起飞高度比较。第一次超时后使用专用 SITL 原生降落恢复；
该恢复不计入链路成功。最终完整成功记录来自修正后的独立飞行。

传感器多线程实现和相机开启配置保持原样。板端保留私有包的回滚副本：
`/data/local/tmp/aerohub-as2/trajectory-backup`，中间版本另外保存，未修改系统 ROS 安装。
原始 DDS 捕获在 `/tmp/as2-trajectory-evidence/flight.jsonl`。

故障注入结果见 `2026-11-05/fault-results.json`。在一次真实 NH / RoboPi / PX4 飞行中，
使用受控 PositionCommand 发生器通过实际 SUPER adapter 发送有效静止 PVA：参考源退出、
实际 adapter 进程暂停、过期参考、显式取消均通过；恢复源和 adapter 后不自动接管；最后
由 AS2 降落并确认 PX4 Disarmed。测试时暂停真实 SUPER，确保同机仅一个输入源。
实际 SUPER 的停止有效输出也已在备用优化失败场景触发 AS2 悬停，但这不等价于主动终止
一个正在输出有效轨迹的 SUPER 进程；该项仍应在后续算法验收补测。

`trajectory_fault_ros_test.py` 是上述受控接口故障驱动，需要 `/evidence` 可写目录、真实
AS2 和 SUPER adapter。看到 `STOP_ADAPTER` 时暂停 adapter 进程；看到 `RESUME_ADAPTER`
时恢复它并创建 `/evidence/continue-ADAPTER`，驱动会检查不能自动恢复。驱动的 finally
在断言失败时尝试 AS2 悬停及降落；它只用于专用仿真环境。

边界：真实 SUPER 的部分短距离目标出现 `OptimizationBakTrajInPolytopes` / `OPT_FAILED`，
本次没有禁用备用轨迹来掩盖它。主/备用 flag 的接口兼容已通过；尚未主动触发并验收真实
避障备用轨迹切换。完整成功飞行记录采集时 max_acc=1 m/s²；最终轨迹配置保留原 SUPER
max_acc=5 m/s²，仅限制 max_vel=1 m/s。实际避障、提速和算法可行性调优不在本次已通过
的执行链验收结论内。该无 RC 仿真降落后应回到 PX4 Hold 再正常申请解锁；不强制解锁，
不关闭健康检查。最终现场已恢复 Hold、Disarmed，SUPER / 导航 / 传感器保持运行。

源码提交顺序见 [source-commits.json](source-commits.json)。历史 source-manifest 保留试验时的
未提交输入摘要；整理提交时仅清理两份 SUPER YAML 的末尾空行，没有改变参数。提交的
文本日志已去除 ANSI 颜色和 CR 换行，原始日志仍在上述 /tmp 证据目录。
