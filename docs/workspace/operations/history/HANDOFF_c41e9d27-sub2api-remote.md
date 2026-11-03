# sub2api 非局域网访问（RKOH_Remote 隧道）

## 当前任务
- 笔记本不在 RKOH 局域网时，Codex 仍可通过 `RKOH_Remote`（经 `3203-jump`）使用 RKOH `~/code/sub2api`。

## 已完成（要点）
- `~/.ssh/config` 新增 `Host RKOH_Remote-tunnel`：经跳板，`LocalForward 127.0.0.1:18080 → RKOH 127.0.0.1:18080`（sub2api 本体），`RemoteForward 127.0.0.1:7890 → 本机 127.0.0.1:7892`（上游代理），`ExitOnForwardFailure yes`。`RKOH-tunnel`（局域网）同步加了同样的 LocalForward。
- `C:\Users\Lenovo\.codex\config.toml`：默认 `model_provider = "sub2api"`（`[model_providers.sub2api]`，base_url 为隧道地址，`supports_websockets = false`，2026-10-10 16:07 由其他会话加入），同时保留 `openai_base_url = "http://127.0.0.1:18080/v1"`。旧会话记录的是内置 `openai` provider，少了这一行就会直连 api.openai.com，报 401 invalid_api_key。旧会话第一次请求会先试 WebSocket 并重试，所以较慢。
- 用户级环境变量 `NO_PROXY=127.0.0.1,localhost`。
- `C:\Users\Lenovo\.ssh\rkoh-tunnel.ps1`：循环 `ssh -N RKOH_Remote-tunnel`；断开后若 RKOH `:7890` 不通，`pkill -u noah -f '^sshd: noah\s*$'` 清理残留的纯转发会话，15s 后重连。
- 计划任务 `RKOH-tunnel`（登录时启动，隐藏窗口）运行上述脚本。
- 2026-10-08 实测（局域网直连 `192.168.1.224:8080` 不通的情况下）：`/health` 200；`/v1/responses` 200 `completed`；`codex exec` 返回 `REMOTE_OK`；杀掉隧道 ssh 后约 15–30s 自动恢复；残留会话清理路径实测生效。

## 有效结论
- 端口 8080 是 `force_proxy.py`（缓冲式 Python 反代，只为 admin 接口注入 `proxy_id`），不支持 WebSocket、会缓冲流式响应，所以隧道直连 sub2api 的 18080。
- Codex 的 HTTP 客户端会走 Windows 系统代理 `127.0.0.1:7892`，且不认 `ProxyOverride` 里的 `127.*`，结果请求 `127.0.0.1` 被自由猫返回 502。`NO_PROXY` 解决了这个问题。
- RKOH sshd 没有配置 `ClientAliveInterval`，noah 也没有免密 sudo。笔记本休眠或断网后，死会话会一直占着 `:7890`，所以脚本里要清理。
- 纯转发会话在 RKOH 上的进程名是 `sshd: noah`（后面补空格）；交互会话是 `@pts/N`，执行命令是 `@notty`，清理时不会误杀。
- 通过隧道访问管理页 `http://127.0.0.1:18080` 时不经过 force_proxy，新增 OpenAI 账号要手动选代理 `local-7890`。
- Codex 模型选择器不读 sub2api 的 `/v1/models`，而是用自带目录（`~/.codex/models_cache.json`，10/8 从官方后台拉取，只有 gpt-5.6 系列）。已用 `model_catalog_json = 'C:\Users\Lenovo\.codex\model_catalog.json'` 指向自定义目录：内容等于官方缓存，再加一条按 `gpt-5.6-sol` 复制的 `gpt-6-astra`（用真实型号名，不用别名 `gpt-6`）。2026-10-10 实测请求的模型、发往上游的模型和上游返回的模型都是 `gpt-6-astra`，没有映射。
- 模型映射：账号 `model_mapping` 都是一对一，`openai` 分组未启用路由、默认映射和推理档位映射。但 sub2api 内置了静态别名：`gpt-6 → gpt-6-astra`、`gpt-5.6 → gpt-5.6-sol`，所以 Codex 目录里只放真实型号名。
- 官方 Codex 目录（经 sub2api 分组设置「固定账号获取模型列表」从上游实际拉取）只有 gpt-5.6-sol/terra/luna 和隐藏的 gpt-5.5，没有 GPT-6。该设置拉取后已关回 `{"enabled": false}`。
- Codex 先尝试 WebSocket，sub2api 报 `no available OpenAI accounts supporting model ... excluded=2`（账号不支持 WS 模式），Codex 会自动回退到 HTTPS，不影响使用。

## 卡住的问题
- 无。

## 下一步
1. 用户使用的是 Cursor 中的 Codex 插件（`openai.chatgpt-26.721.30844`，自带 `codex.exe app-server`，共用 `~/.codex/config.toml`）。需要完整退出 Cursor 再打开，插件才能继承 `NO_PROXY`。已用该插件自带的二进制、在设置了 `HTTP(S)_PROXY=127.0.0.1:7892`（Cursor 的 `http.proxy` 设置）时实测，返回 `EXT_OK`。
2. 可选：请有 sudo 的人在 RKOH 的 sshd 中设置 `ClientAliveInterval 30` / `ClientAliveCountMax 3`，设置后脚本里的清理步骤可以去掉。
3. 可选：在 sub2api 账号中开启 OpenAI WS 模式，省掉 WebSocket 回退。

## 踩过的坑
- 如果交互式 `ssh RKOH`（带 RemoteForward 7890）在隧道断开期间抢先占住了 `:7890`，隧道会重连失败，直到该会话退出。
- 在 PowerShell 里给 ssh 传远端命令时，`$(...)` 会先在本地展开。

## 相关指令/链接
- 前序：`docs/HANDOFF_8f3c3b0a-sub2api.md`
- 状态：`Get-ScheduledTask RKOH-tunnel`；`curl.exe --noproxy "*" http://127.0.0.1:18080/health`
- 手动重启：`Stop-ScheduledTask RKOH-tunnel; Start-ScheduledTask RKOH-tunnel`
