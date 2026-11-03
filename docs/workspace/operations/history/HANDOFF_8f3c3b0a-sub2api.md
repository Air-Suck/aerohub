# RKOH sub2api 服务启动与使用核验

## 当前任务
- 在 RKOH 启动并核验 `/home/noah/code/sub2api`。
- 将该服务作为自定义模型提供方接入 Windows Codex。
- 修复 Cursor Codex 偶发上下文丢失和 Sub2API 流式断连。

## 已完成（要点）
- 已读取 RKOH 运维总结；目标机可通过 `ssh RKOH` 或 `ssh RKOH_Remote` 访问，登录用户为 `noah`。
- 已按用户要求从项目 `AGENTS.md` 删除 Codex with ChatGPT 的默认启用规则。
- 已在 `/home/noah/code/sub2api` 执行 `docker compose up -d`；`sub2api`、PostgreSQL、Redis、`sub2api-force-proxy` 均已运行，前三者健康检查通过。
- RKOH 本机和 Windows 端访问 `http://192.168.1.224:8080/health` 均返回 `200 {"status":"ok"}`。
- 用现有 API Key 调用 `/v1/models` 成功；2026-10-08 当前返回 24 个模型，包含 GPT 5.2 至 GPT 6、Codex、音频/实时及图像模型。
- 用户已将四个 OpenAI OAuth 账号恢复为 active、schedulable，并加入 `openai` 分组。
- 2026-10-08 已用现有 API Key 对 `gpt-5.6` 调用 `/v1/responses`，返回 HTTP 200、`completed`、文本 `OK`；实际调度到 gpt4，端到端链路可用。
- 已核对本机 Codex CLI 为 `0.146.0-alpha.3`；用户级配置使用 `model = "gpt-5.6-sol"` 和 `openai_base_url = "http://127.0.0.1:18080/v1"`，通过本机 SSH 隧道访问 Sub2API。
- 2026-10-09 核对本机演示文稿能力：`Presentations` 是 PPT/Google Slides 创建、编辑、渲染和质检的核心 Skill；`template-creator` 用于把参考 PPT 固化为可复用个人模板；`imagegen` 可补充定制视觉素材。
- 已核验 `Presentations`、`template-creator`、`Spreadsheets`、`imagegen` 和 `humanizer` 均已安装；前三个对应插件已在用户级 Codex 配置中启用，无需重复安装。
- 2026-10-09 从 `ningzimu/codex-ppt-skill` 安装社区 Skill `codex-ppt` 到 `C:\Users\Lenovo\.codex\skills\codex-ppt`。该 Skill 采用整页图像生成，提供 12 套视觉风格、参考风格复刻、样稿确认和演讲者备注，适合作为可编辑型 `Presentations` 的视觉增强补充。已补齐 Python `openai` 依赖、升级 `httpcore` 至兼容版本，`pip check` 无冲突，组装脚本可正常启动。
- PPT 使用结论：默认专业交付优先 `Presentations + imagegen + humanizer`，数据较多时加入 `Spreadsheets`；仅在视觉冲击和风格统一优先、可接受整页图片不可编辑时，以 `codex-ppt + imagegen + humanizer` 为主。不要让 `Presentations` 与 `codex-ppt` 同时生成同一版成品。
- Cursor Codex 上下文故障已做两轮修复。第一轮备份位于 `C:\Users\Lenovo\.codex\repair-backups\20261009-143515`；第二轮全量备份位于 `C:\Users\Lenovo\.codex\repair-backups\all-threads-20261009-145818`，两个 SQLite 备份完整性均为 `ok`。全量扫描 30 个 rollout 后，又规范化 22 个文件中的 5,485 条旧版 `token_count.rate_limits` 元数据；所有文件 JSON 错误为 0、遗留格式为 0，对话正文未改动。
- 第一轮遗漏了投影游标：旧记录被拒后，数据库字节偏移已移到下一条，但期望序号仍停在被拒记录。已精确回退 4 条错位线程的字节偏移，未清空已有 `thread_items`/`thread_turns`。两条活跃线程均已自动补齐：`01a0f230-...` 从 ordinal 165 推进到 11715，消息索引从 12 条恢复到 3450 条；`01a0f18a-...` 从 ordinal 212 推进到 2917，消息索引从 46 条恢复到 885 条。数据库完整性为 `ok`；其余两条归档线程会在首次打开时补建索引。
- Cursor 市场当前仍提供 `openai.chatgpt@26.721.30844`，强制更新后版本不变，已排除漏更新。Codex 支持同项目多个独立对话；项目事实继续通过 `AGENTS.md` 和本 handoff 共享。
- Sub2API 流式链路实测 8/8 次成功，均收到 `response.completed`，平均 2.98 秒、最大 4.27 秒。
- 已定位 13:17 至 13:21 流式断连根因：RKOH 两次发生全局 OOM，内核杀死 `ego_planner_node`，同时 Docker、Redis、Postgres 和 SSH 代理停顿；并非 Codex 上下文窗口耗尽。当前该容器已不存在，Sub2API 四个容器健康。
- 已增强 `RKOH-tunnel`：SSH 保活改为 10 秒、最多 2 次，加入 TCP keepalive 和 15 秒连接超时；重连等待由 15 秒缩短至 5 秒；计划任务设置为失败后每分钟重启、最多 999 次并启用 `StartWhenAvailable`。现有隧道下次重连后采用新 SSH 参数。
- Codex 支持并建议同一项目使用多个对话，每个对话对应独立目标并保留独立 transcript；同项目对话共享工作区文件、`AGENTS.md` 和项目配置，但不会自动共享其他对话的消息内容。此前问题是部分旧线程历史投影损坏及可能选错线程，不是“项目不能有多个对话”。
- `openai_base_url` 已改为 `http://127.0.0.1:18080/v1`（SSH 隧道），见 `docs/HANDOFF_c41e9d27-sub2api-remote.md`。
- 2026-10-10 再次核查上下文问题：当天没有新增历史投影错误，当前 Sub2API 对话和课程/PPT 对话的原始消息均完整。课程/PPT 对话已有 312 轮、1,014 条消息，当前单次输入约 14.8 万 token；出现的“忘记早期内容”属于超长线程中的检索质量下降，不是消息被删除。对应 `docs/HANDOFF_2b86c7e8-36b9-4ad3-86d5-302402a85e87.md` 内容完整且已更新到当前讨论位置。
- 同次核查发现当前线程有 5 次流式重试。Sub2API 日志显示 WebSocket 入口因候选账号被排除而选号失败，随后 HTTP `/v1/responses` 请求成功；这会导致回复重试或中断，但不会删除本地 transcript。
- 已备份原配置到 `C:\Users\Lenovo\.codex\repair-backups\context-loss-20261010\config.toml.before`，并将 Codex 改为显式 `sub2api` provider：`base_url = "http://127.0.0.1:18080/v1"`、`wire_api = "responses"`、`requires_openai_auth = true`、`supports_websockets = false`。`codex exec --strict-config` 实测返回 `OK`，新进程日志中 WebSocket/重试计数为 0。
- 已定位 PPT 样稿无法生成的原因：`imagegen` Skill 已完整安装，但当前 API Key + 自定义 `sub2api` provider 会话没有注册内置 `image_gen` 工具，只有 `view_image`；重装 Skill 或重新打开同类会话都不会补出该工具。
- 已执行 `codex-ppt` 自带的备用后端修复：在 `C:\Users\Lenovo\.codex-ppt-skill\.venv` 建立隔离运行环境，并将 `C:\Users\Lenovo\.codex-ppt-skill\.env` 配置为 Sub2API `http://127.0.0.1:18080/v1` 和 `gpt-image-2.5-flare`。`doctor --check-api` 返回 HTTP 200，`pip check` 无依赖冲突；未生成测试图片，避免在后端确认前产生调用费用。
- 后续真实生成暴露了分组权限问题：`openai` 分组虽然列出图片模型，但 `allow_image_generation=false`，因此 `/v1/images/generations` 在Sub2API本机以0毫秒返回403和 `Image generation is not enabled for this group`。已将分组2的普通图片生成开关设为true，批量图片生成保持false。
- 修复后用 `gpt-image-2.5-flare` 完成两次真实验证：1024×1024 low测试返回HTTP 200和PNG数据；第12页样稿以2560×1440 medium请求返回HTTP 200并写入 `F:\UAV\ppt\无人机系统架构与技术路线\origin_image\slide_12.png`。服务实际调度账号4，生成耗时约117秒；上游实际返回1672×941的16:9 PNG。
- 另一个PPT会话随后已选择原生可编辑PPTX方案，见 `docs/HANDOFF_373289b0.md`。上述整页图片只作为权限验证和视觉参考保留，不代表最终交付切回 `codex-ppt` 图片型方案。

## 有效结论
- RKOH 的 `127.0.0.1:7890` 通常来自 SSH RemoteForward，依赖至少一个对应 SSH 会话持续存在。
- 2026-10-26 鉴权 403 核查：本机 `127.0.0.1:7892` 代理可用，经代理访问 `auth.openai.com/oauth/token` 能到达 OpenAI（返回 405，非连接失败）；当前出口公网 IP 为 `209.50.229.42`，归属美国 Leaseweb 数据中心。`unsupported_country_region_territory` 因此不是代理未配置，而是当前数据中心出口被鉴权策略拒绝。此前记录的出口 `140.235.143.20` 已不再是当前节点。
- 修复方向：在本机代理客户端切换到支持 OpenAI 鉴权的干净、非数据中心出口，并保持 RKOH 的 `7890 -> 本机 7892` RemoteForward；切换后用 `curl --proxy http://127.0.0.1:7892 https://ipinfo.io/json` 复核出口，再重新进行 OAuth token exchange。不要在 Sub2API 代码中重试或更换画质，这类 403 不会靠重试消失。
- 2026-10-26 进一步核查：RKOH `127.0.0.1:7890` 实时出口为 `140.235.143.20`；Sub2API、Postgres、Redis、force-proxy 均健康。普通 `/v1/responses` 请求持续返回 200。OAuth `/api/v1/admin/openai/exchange-code` 多次返回 403，但同一时段 `sub2api-force-proxy` 无请求/注入日志，说明添加账号时很可能使用了直连 `18080` 管理入口且请求未携带 `proxy_id=1`。通过 force-proxy 的 `8080` 入口，或在 `18080` 管理页面显式选择 `local-7890`，才能保证 token exchange 使用该代理。
- 2026-10-26 已新增 SSH 隧道转发：本机 `127.0.0.1:18081 -> RKOH 127.0.0.1:8080`，用于访问 force-proxy 管理入口；原有本机 `18080 -> RKOH 18080` 保留给 Codex API。隧道重启后两个本地 `/health` 均返回 `{"status":"ok"}`。
- 2026-10-27 gpt2（account_id=2）被标记为 gpt-6-astra 限流的真实原因：上游返回 HTTP 400，Sub2API 日志明确写为 `upstream_400_codex_plan_gated_model`，不是普通 HTTP 429，也不是代理失败。Sub2API 随后写入 `model_rate_limits`，对 `gpt-6-astra` 设置约 30 分钟冷却（本次记录 reset_at 为 2026-10-27 17:12:46 +08:00）；同一账号的 `gpt-5.6-sol` 也出现同样的 `codex_plan_gated_model`。gpt2 当前数据库状态仍为 `active/schedulable=true`，所以账号本身未被停用。
- 2026-10-27 进一步核对 gpt2：`credentials.plan_type=plus`，但 `credentials.subscription_expires_at=2026-10-20T13:08:50Z`，订阅已过期；OAuth token 仍显示到 2026-10-28。`model_mapping` 只是 Sub2API 的全局映射配置，不代表该账号真实模型权限。当前已实证 gpt-6-astra 和 gpt-5.6-sol 均被上游以 `codex_plan_gated_model` 拒绝。
- 远程登录 shell 是 zsh，复杂命令应显式使用 `bash -lc`。
- 对外入口：`http://192.168.1.224:8080`；应用内部端口为 `18080`。
- OpenAI 账户 1、2、3 都配置 `proxy_id=1`。代理记录 1 为 `local-7890`，协议 HTTP，地址 `127.0.0.1:7890`，无认证。
- 实际链路：Sub2API → RKOH `127.0.0.1:7890` → SSH RemoteForward → Windows `127.0.0.1:7892` → `F:\ziyoumao\ziyoumaoCore.exe`。实测可访问 OpenAI，当前出口公网 IP 为 `140.235.143.20`。
- SSH 配置为 `RemoteForward 127.0.0.1:7890 127.0.0.1:7892`。若所有提供该转发的 SSH 会话断开，OpenAI 上游代理也会断开。
- 2026-10-09 的长时间流式中断由 RKOH 全局内存耗尽触发。再次运行 `ego_planner_node` 前应给对应容器设置内存上限并保留系统余量，否则任何隧道参数都无法完全避免主机级停顿。
- 管理员账号为 `admin@sub2api.local`；密码由用户自己保管。`.env` 中的 `ADMIN_PASSWORD` 为空，数据库只存 bcrypt 哈希，无法查回明文；忘记密码时只能改写 `users.password_hash` 来重置。
- 当前 OpenAI 分组为 `openai`（group id 2）。新建 OpenAI 账号后必须加入此分组，现有 API Key 才能调度它。
- 添加 OpenAI OAuth 账号时应显式选择代理 `local-7890`；浏览器授权结束后，将地址栏中的完整 localhost 回调 URL 粘回 Sub2API 完成令牌交换。
- 账号列表的 5h/7d 百分比是已用比例，`0%` 表示未记录到消耗。截图中 gpt2 的 7d 实际为 8%；gpt1、gpt3 的旧窗口重置时间已在 2026-09 过期，因此界面归零；gpt4 是新授权账号，当前 5h/7d 均为 0。
- 当前 Codex 接入在用户级 `C:\Users\Lenovo\.codex\config.toml` 使用显式 `sub2api` provider，通过本机 `http://127.0.0.1:18080/v1` SSH 隧道调用 Responses API，并关闭 WebSocket。`requires_openai_auth = true` 复用 Codex 已保存的 API Key，不把密钥写入配置文件。
- 超长线程即使没有达到上下文窗口上限，也可能降低对早期细节的召回质量。一个明确目标完成后应新建对话，并通过对应 handoff 延续项目状态；不要无限延长同一个对话。
- 在当前 Sub2API 登录方式下，如需生成PPT图片，可使用已验证的 `codex-ppt` CLI/API fallback；该路径不依赖会话内置 `image_gen`。当前课程PPT本身已另行选择原生可编辑方案。
- `model = "gpt-5.6-sol"` 只指定 Codex 启动时的默认模型，并不会限制 Sub2API 只能提供该模型；可在 Codex 会话的模型选择器中切换，或把该值改为 Sub2API `/v1/models` 返回的其他文本/编码模型。图像、音频和 Realtime 模型不适合作为 Codex 编码会话模型。

## 卡住的问题
- 无。当前 Cursor 进程需要执行一次 `Developer: Reload Window`，新建进程才会采用关闭 WebSocket 的 provider 配置。

## 下一步
1. 在 Cursor 命令面板执行一次 `Developer: Reload Window`，让关闭 WebSocket 的新配置生效。
2. 课程/PPT 工作不要继续延长原 312 轮线程；在 `F:\UAV` 新建对话并发送“开始”，让新对话读取 `docs/HANDOFF_2b86c7e8-36b9-4ad3-86d5-302402a85e87.md` 后继续。
3. 再运行 EGO Planner/Gazebo 工作负载前，为其 Docker 容器设置内存上限，避免再次拖垮 Sub2API 和 SSH 隧道。

## 踩过的坑
- SSH 报 `remote port forwarding failed for listen port 7890` 通常表示已有会话占用端口，不影响普通 SSH 操作。
- Windows 本机设置了 HTTP 代理时，访问 RKOH 内网地址应让客户端绕过代理，例如 `curl.exe --noproxy "*" ...`。

## 相关指令/链接
- `docs/RKOH磁盘清理与大目录打包回传.md`
- 启动：`ssh RKOH 'cd /home/noah/code/sub2api && docker compose up -d'`
- 状态：`ssh RKOH 'cd /home/noah/code/sub2api && docker compose ps'`
- 日志：`ssh RKOH 'cd /home/noah/code/sub2api && docker compose logs -f sub2api'`
- 停止：`ssh RKOH 'cd /home/noah/code/sub2api && docker compose down'`
