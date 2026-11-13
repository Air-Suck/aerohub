# Offboard 切换期间定位停更诊断

当前结论：**Offboard 阻塞已修复，PC/RoboPi 地面回归及四机起飞验证通过**。
以下诊断与红色回归为修复前证据，均保留；完整四阶段Demo尚未通过。

原诊断结论：问题不只出现在 RoboPi，也能在 PC 的对应节点复现。
无需解锁或起飞，给目标节点发送一次 Offboard=true 就能检出。
机器可读摘要和证据断言见同目录 `state-pause-diagnosis.json`。

## 三轮真实地面对照

| 场景 | 触发对象 | 结果 | 目标 CAT-ORA 定位最大接收间隔 |
| --- | --- | --- | --- |
| 静态基线 | 无控制服务 | 四机新鲜度通过 | drone0 约 70 ms |
| RoboPi Offboard | drone0 | 仅 drone0 触发新鲜度保护 | 约 1.205 s |
| PC Offboard | drone1 | 仅 drone1 触发新鲜度保护 | 约 1.200 s |

RoboPi/PC 服务耗时分别约 1.203/1.203 秒。两个触发窗口中，目标 PX4 原始里程计的
最大接收间隔分别约14.3/14.0毫秒；全局clock分别约23.6/23.3毫秒。
其余三机CAT-ORA定位接收间隔约60–74毫秒。全部四机armed反馈始终为false。
这排除了“只有RoboPi定位算法停算”以及“整个仿真时钟/飞控数据暂停”的解释。

RoboPi同机原始trace也记录到AS2 odom约1.204秒停更，而原始PX4输入继续，
所以不是仅跨机传输或工作站接收不及时。板端探针早于工作站启动，比较时使用
wall_ns对齐事件；直接给两侧各套相同elapsed预热窗口会漏掉板端较早发生的关键事件。
前两轮板端probe因监管结束未落最终report，原始trace完整覆盖事件，分析时明确这一缺口。

## 对应实现

- `as2/as2_platform_pixhawk/src/pixhawk_platform.cpp` 的 `ownSetOffboardControl()`：
  在服务回调内以100Hz执行120次预发送，并逐次sleep，总计约1.2秒。
- `as2/as2_platform_pixhawk/src/pixhawk_platform_node.cpp` 调用 `as2::spinLoop()`。
- `as2/aerostack2/as2_core/src/core_functions.cpp` 的spinLoop使用SingleThreadedExecutor。

服务回调未返回时，同节点的PX4里程计订阅与平台状态发布无法被调度；下游状态估计和
CAT-ORA输入因而断流。PX4原始消息仍在发布，恢复后新消息时间戳也正常，不能用
“收到消息时的age很小”替代“整个期间没有停更”。探针定时评估最后状态的age捕获了该问题。

0.5秒新鲜度保护是正确触发；不能改成大于1.2秒、补发旧状态、修改stamp或使用真值绕过。

## 最小复验入口

加载本地环境、图形会话及依赖后，复用一板三PC地面启动：

```bash
python3 deployment/airsim/demos/catora_town05/run_ground.py --diagnostic \
  --probe-offboard --probe-vehicle drone0 --output log/airsim/robopi-offboard-new

python3 deployment/airsim/demos/catora_town05/run_ground.py --diagnostic \
  --probe-offboard --probe-vehicle drone1 --output log/airsim/pc-offboard-new
```

当前实现应返回非零，并保留diagnostics/report.json中的目标机age违例，这是预期的红色回归信号。
探针从不调用arming/Takeoff。Offboard=false在当前平台实现中被明确拒绝，不能把该请求
当成已恢复手动模式；四机始终disarmed，测试结束按拥有身份回收本轮所有节点和仿真。

## 修复边界（原诊断建议）

应消除预发送握手对状态回调的阻塞，同时保留PX4要求的超过1秒稳定预发送，以及
服务结果、取消、重复请求、失败处理和回调生命周期。简单换成多线程执行器但仍使用
同一个互斥callback group不解决问题；随意增加并发也可能给共享控制状态引入数据竞争。

原诊断轮没有改AS2平台运行实现。后续已在独立overlay完成下述修复，没有重建或替换共享镜像/默认板端安装。原验证要求为：
先让上述真实地面红色回归变绿，再做解锁/Offboard及起飞恢复验证。消息版本载荷不匹配
是另一个已知缺口，不据此冒充已证实为本次1.2秒停更的根因。

## 证据位置

- `log/airsim/catora-town05-state-baseline-20261112-01/`
- `log/airsim/catora-town05-state-offboard-20261112-01/`
- `log/airsim/catora-town05-state-pc-offboard-20261113-01/`

各轮包含原始trace、源码快照、镜像ID、clock门控、节点发布者、清理结果及工件校验和。
总体Demo依旧未完成，不能从诊断成功推导为飞行成功。


## 修复与复验

- AS2基类新增protected服务回调注册入口，仅内联方法，无新增基类成员或虚方法。
- Pixhawk使用ROS deferred response，10ms wall timer完成至少120帧且至少1200ms预发送；
  期间仍调度原里程计、状态和应急回调。3秒总超时或500ms发送中断结束挂起请求。
- 重复/false请求保留拒绝语义；显式disarm（包括已经disarmed）取消挂起握手。
  stop/kill取消并锁存禁止新握手，须新平台实例恢复；shutdown不留延迟切换命令。
- PC与板端Pixhawk节点及静态库均重编。板端采用私有SDK工作区，默认安装保持不动。
  运行时显式选择PC workspace和候选AS2 prefix；Planner仅复用旧候选的纯Python前缀。

| 验证 | 结果 |
| --- | --- |
| PC实际地面Offboard | 握手约1.214s；odom最大间隔16.85ms，CAT-ORA约71.64ms，违例0 |
| RoboPi实际地面Offboard | 握手约1.206s；odom最大间隔17.07ms，CAT-ORA约73.64ms，违例0 |
| 实际平台+合成DDS生命周期 | 120帧/1.207s；duplicate/manual拒绝、disarm取消、kill锁存通过 |
| 合成DDS退出/超时 | 退出与超时后模式切换命令均为0 |
| AirSim/PX4四机起飞 | 四机原生Takeoff及横排4秒保持通过；失败后四机原生Land/disarmed确认 |

源码补丁：`deployment/platform/patches/{as2-core,pixhawk}-offboard-nonblocking.patch`。
`prepare_ohos_sources.py`在导出的锁定源码上重放补丁并记录SHA，不依赖未提交的submodule
checkout才能重现；重放字节一致和幂等测试见`tools/robopi/tests/test_offboard_source_patches.py`。

Lifecycle第1轮失败（已经disarmed的disarm未取消）保留在callback-test-01；修订后的
callback-test-02及shutdown/timeout通过，不将合成DDS结果提升为飞行故障验收。

四阶段任务仍未通过：flight-20261113-02中原生CAT-ORA交换了中间两机的菱形目标，
适配层以固定逐机目标契约拒绝（日志标签NO_SOLUTION，detail为reassigned a fixed goal）。
这是新的目标归属问题，不是定位停更复发。没有丢弃中间轨迹或目标绑定来获得通过。
