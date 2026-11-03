# Codex 反复重连排查

## 当前任务

解释 Cursor/Codex 显示 `Reconnecting 1/5`、`5/5` 和 `stream disconnected before completion` 的原因。

## 已完成

- 本机到 `127.0.0.1:18080` 的 Sub2API 健康检查返回 `{"status":"ok"}`。
- SSH 隧道、Sub2API、PostgreSQL、Redis 均在运行；当前连接未显示本地隧道断开。
- RKOH 的 Sub2API 日志在 2026-10-23 17:33-17:42 多次记录上游错误：`Our servers are currently overloaded. Please try again later.`
- 失败请求使用 `gpt-6-astra`，账号主要为 23；单次请求体约 3.1 MB，耗时约 8-36 秒后上游流失败。部分请求切换到账号22并完成。

## 有效结论

- 当前反复重连的直接原因是上游 OpenAI 响应流失败/过载，Codex 客户端自动重试；不是本机 Sub2API 健康检查失败，也没有证据表明 SSH 隧道反复断线。
- 超长上下文和多个同时运行的 Codex 进程会把请求体推到数 MB，放大失败概率和等待时间。
- Sub2API 将部分上游失败记录为 HTTP 200 后关闭 SSE，客户端因未收到完整事件而显示 `stream disconnected`。
- 2026-10-23 17:57-18:07复核：账号23仍反复返回同一过载错误；账号22的同一模型请求在17:??和18:07完成，说明至少部分上游账号/线路可用。日志未见`context canceled`、`EOF`或隧道超时，当前证据不支持把这次主要归因于本机到Sub2API的网络断线。

## 下一步

- 暂停或新建短对话，避免继续使用数 MB 历史上下文。
- 暂时使用 `gpt-5.6-sol` 或降低推理强度；不要通过反复点击重连同时制造更多请求。
- 如仍频繁失败，再单独检查账号23、上游代理链路和 Sub2API 账号故障转移设置。
