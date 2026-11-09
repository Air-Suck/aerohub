# CAT-ORA 自有适配

AS2 负责运动执行；CAT-ORA 保留原生算法与 Planner 的采样调度。
本包处理算法所需的输入、轨迹转换和停止意图到 AS2 接口的适配，
不依赖 `as2_navigation_inputs`，不新增 AS2 控制模式或放宽执行保护。

| 入口 | 职责 |
| --- | --- |
| `cat_ora_state_adapter_node` | 逐机 AS2 `sensor_measurements/odom` → `catora/local_pose`；保留源时间与局部 ENU 坐标，仅将规划状态降至最高约 20 Hz。 |
| `cat_ora_adapter_node` | 合格局部定位和固定 Fleet 原点 → 原生 CAT-ORA 服务 → `TimedTrajectorySegment`；保留固定目标归属检查。 |
| `cat_ora_as2_adapter_node` | Planner 采样的 `FleetSetpoint` → AS2 单点轨迹；复用已有 PVA 转换实现。暂停、取消、失败或完成时请求 AS2 HOVER，验证响应及状态反馈。 |

输入入口参数为 `deployment_config`；其他两入口参数为 `resolved_config`。
部署配置的 `topics.pose` 应指向 `/<namespace>/catora/local_pose`。
输入必须为该机 `<namespace>/odom`，位置与四元数有限且四元数归一化；
源时间非零、严格递增，年龄不超过 1 秒，未来偏差不超过 50 ms。
算法请求发起前及 RPC 返回后都核对定位新鲜度；慢响应不能把过期起点变成新规划。Planner 也拒绝用过期定位判定到点。
时间回退导致输入失效时应停止并重新开始任务，不沿用旧任务状态。

任务直接调用 AS2 起飞、轨迹接管和降落；adapter 不自动申请 TRAJECTORY。
Planner 的 CANCEL 代表取消规划获准，不能当作飞机已经停止。
任务还需等待 `planner/as2_bridge_state` 中四机 `stop_confirmed`，
其中每机都要求 AS2 HOVER 服务成功及 HOVER/POSITION 输入模式反馈。
8 秒内无确认或 AS2 拒绝会出现在 `stop_errors`，不能报告成功。
停止确认完成后保留结果，后续进入降落模式不撤销已经完成的停止。
恢复暂停任务需重新规划，并由任务显式请求 AS2 TRAJECTORY。

`planner/catora_input_state` 提供各机接收、发布计数和最后拒收原因。
最终 PVA 的时间检查、50 Hz 执行及失流保护仍由 AS2 负责。
Swarm 暂时继续使用原公共输出入口，本次不修改其执行流程。

完整启动、真实取消及故障回归见 [四机应用](../../../../../deployment/airsim/demos/catora_open_field/README.md)。
板端隔离接口测试为 `tests/verify_ros_adapters.py`，使用 `adapterprobe*` 命名空间
及假 AS2 服务，不发送真实解锁或起飞命令；覆盖非法定位、暂停恢复、取消、
降落后停止结果保持，以及 AS2 拒绝停止的错误反馈。

Planner 停止运行进程时回收整个独立进程组，不能仅等待 `ros2 run` wrapper。
`tests/verify_slow_proxy.py` 验证迟到响应被拒绝、下一轮新鲜输入可重新生成轨迹。
