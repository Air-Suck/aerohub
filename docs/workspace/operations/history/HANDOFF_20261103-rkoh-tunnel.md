# RKOH 开机 SSH 隧道

## 当前任务
- 解释 `ssh RKOH_Remote-tunnel` 报 `remote port forwarding failed for listen port 7890`，并重启登录自启任务 `RKOH-tunnel`。

## 已完成（要点）
- 2026-11-03 15:56 交互式 `ssh RKOH_Remote-tunnel` 失败：开机任务已占着远端 `127.0.0.1:7890`，`ExitOnForwardFailure yes` 让第二条会话直接退出。
- 已停掉计划任务并结束残留 `ssh -N RKOH_Remote-tunnel`（停任务不会杀掉 ssh 子进程），再 `Start-ScheduledTask RKOH-tunnel`。
- 新会话 PID 19816。本机 `127.0.0.1:18080`、`18081` 在听；`/health` 返回 200；远端经 `127.0.0.1:7890` 访问 `generate_204` 返回 204。

## 有效结论
- `Welcome to VyOS` 是跳板 `vyatta@202.120.40.82` 的横幅，不是失败原因。
- 自启脚本是 `C:\Users\Lenovo\.ssh\rkoh-tunnel.ps1`，Host 为 `RKOH_Remote-tunnel`：`LocalForward 18080/18081`，`RemoteForward 127.0.0.1:7890 → 本机 7892`。同一时刻只能有一个会话占 `7890`。
- 手动重启：`Stop-ScheduledTask RKOH-tunnel` 后若 `ssh -N RKOH_Remote-tunnel` 仍在，先结束它，再 `Start-ScheduledTask RKOH-tunnel`。

## 卡住的问题
- 无。

## 下一步
- 不要再开交互式 `ssh RKOH_Remote-tunnel` 去测隧道；用 `curl.exe --noproxy "*" http://127.0.0.1:18080/health`。

## 踩过的坑
- `Stop-ScheduledTask` 只停 powershell 父进程，`ssh.exe` 子进程会继续占着本机 `18080` 和远端 `7890`。

## 相关指令/链接
- 前序：`docs/HANDOFF_c41e9d27-sub2api-remote.md`
- 状态：`Get-ScheduledTask RKOH-tunnel`
