# 集成工程交接

历史审计与未完成事项；运行状态以记录当时为准。

- [2026-11-03 文档整理](HANDOFF_20261103-docs-organization.md)

[返回文档总览](../README.md)

- [Planner 复现与 Aerostack2 集成文档审计 — HANDOFF_4ac1b1b2-a5a2-4fb9-ba8c-1d02bd3f1a61.md](HANDOFF_4ac1b1b2-a5a2-4fb9-ba8c-1d02bd3f1a61.md)

## 旧部署参考版本

当前分支已移除 Gazebo/PX4 1.17 历史部署、早期 AirSim 传输探针和初始 OHOS
补丁目录。旧 Gazebo 部署由删除前 main 提交保存：

```text
c420169460e59c396f586320e0e06fc96e387f6e
```

例如只读查看原 Compose：

```bash
git show c420169460e59c396f586320e0e06fc96e387f6e:deployment/integration/compose.yaml
```

后续 main 合并清理后，仍可用这个提交定位历史源码。初始 OHOS 的 uint 修复
已在当前 AS2 fork 中；当前验证、配置和消息定义使用现有入口，不恢复旧部署。
