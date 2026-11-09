# AS2 共用运行代码

`run.py` 读取 `deployment/airsim/config/as2/` 中的节点、机群与 PID 配置，启动和管理
AS2 platform、状态估计、控制器及运动 Behavior。PC onboard_computer 镜像复制这份
启动器，RoboPi 部署包也复制同一份，不再由某个部署目录独占。

`tools/airsim/verify_interaction.py` 验证状态、时钟和可选的单机仿真飞行闭环。
它由对应部署入口执行，不是自动启动的业务任务。

- PC AS2：[部署配方](../../deployment/onboard_computer/README.md)
- RoboPi：[运行入口](../../deployment/robopi/README.md)
- Planner 输出桥仍位于 `src/behavior/legacy_setpoint/`。

## 多机 TF 话题

`AS2_TF_SCOPE=per-vehicle` 通过 `scoped_tf.launch.py` 为上游 launch 加入
`/tf → /<namespace>/tf`、`/tf_static → /<namespace>/tf_static` remapping。
不改 frame_id、变换数值、控制频率或 AS2 源码。未设置时保留原共享 TF 行为。
CAT-ORA 四机应用默认逐机隔离，`--tf-scope shared` 仅用于地面对照。
需要 TF 的外部消费者应订阅对应飞机的话题；本配置不合并全机群 TF 树。
