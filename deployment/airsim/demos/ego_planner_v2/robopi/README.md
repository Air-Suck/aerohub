# RoboPi 单机入口

操作、构建、候选安装与回退见 [Demo说明](../README.md#robopi-单机)，历史验收见
[VALIDATION.md](../VALIDATION.md)。此目录由 STEP23 已验收快照整理，只有路径与
外部工作区配置适配；混合双机 runtime 的后续修改未纳入。

状态未知或停止未确认时保留仿真/时钟/控制服务，检查本轮 run ID、PID及启动时间，
通过 `run.py status/stop/collect --output <session>` 管理。回退须先安全停止当前会话，
选择保留候选的匹配配置，再启动新会话；不替换运行中库、不转移旧规划任务。
