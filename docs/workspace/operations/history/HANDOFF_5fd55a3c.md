# Codex 插件网络链路

## 当前任务
- 排查 Codex 插件 `Reconnecting... waiting for network` / `Connection failed: error sending request`。

## 已完成（要点）
- 2026-11-03 02:25–02:30：这条文案来自 Codex 对 `http://127.0.0.1:18080/v1` 的 responses 流重试（5s→60s）。当时 `ssh -N RKOH_Remote-tunnel` 因远端 `7890` 已被占用、`ExitOnForwardFailure yes` 整段退出，`18080` 一起消失。02:29:52 还有一次 `Connection to 192.168.1.224 closed by remote host`，流中断报 `error decoding response body`。
- 02:33 起当前会话 `ssh -N RKOH_Remote-tunnel`（pid 7276）保持。`/health` 200。Codex pid 5486 已连上 `127.0.0.1:18080`。RKOH 上 `curl -x http://127.0.0.1:7890` `generate_204` 返回 204。02:30:14 之后没有新的 `responses_retry`。

## 有效结论
- 模型链路：Codex → 本机 `127.0.0.1:18080` → `3203-jump`（`vyatta@202.120.40.82`）→ `noah@192.168.1.224` 的 sub2api `:18080` → sub2api 走 RKOH `127.0.0.1:7890` → 这条 SSH 的 RemoteForward → 本机 `127.0.0.1:7892`（ziyoumao）→ 外网。
- 自启脚本 `~/.ssh/rkoh-mac-tunnel.sh` 循环的是 `RKOH_Remote-tunnel`。`18080` 和 `7890` 绑在同一次 SSH 上，`7890` 失败会拆掉 Codex 的入口。同一时刻只能有一个客户端占 `7890`。
- 插件 webview 仍在直接请求 `chatgpt.com` 的 `/wham/usage`、`/wham/tasks/list`、`/ces/v1/rgstr`，本机直连超时。系统代理是 `127.0.0.1:7892`，这条 webview 请求没有走它。这不是上面那句重连文案。

## 卡住的问题
- Windows 侧 `RKOH-tunnel` 若仍抢 `7890`，本机这条隧道会再被拆掉，Codex 会再次报同样的重连。

## 下一步
- 只保留一台机器占 `RKOH 127.0.0.1:7890`。本机要当代理出口时，停掉 Windows 计划任务 `RKOH-tunnel`。

## 踩过的坑
- `Stop-ScheduledTask` / 停 launchd 父进程都不会自动清掉已建立的 `ssh`，远端 `7890` 会继续被占。
- `~/.ssh/config` 里某一行 `LocalForward` 写进全角空格时，所有 ssh 都会报 `Bad forwarding specification` 并退出。

## 相关指令/链接
- 隧道日志：`~/grok-bot/logs/rkoh-mac-tunnel.log`
- 插件日志：`~/Library/Application Support/Cursor/logs/20261103T014647/window1/exthost/openai.chatgpt/Codex.log`
- 前序：`docs/HANDOFF_20261103-rkoh-tunnel.md`
