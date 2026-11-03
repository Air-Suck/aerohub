# RKOH 磁盘清理与大目录打包回传

## 当前任务
RKOH 根分区写满（可用 0）的排查与清理。已完成：RKOH 恢复到 341G 可用（61%）。

## 已完成（要点）
- 停止 `catora-px4-gazebo`、`catora-planner-px4` 容器（保留未 rm，可 `docker start`）
- 删除 UAV 两个工程 `PX4-Autopilot/build/px4_sitl_default/rootfs/*/log` 下 >1G 的 `.ulg`（swarm 9.8G×7、catora 3.1G×4）及 804 个空 `.ulg`；保留正常大小的运行日志
- 三个大目录打包（zstd -3）回传本机，本机与远程 SHA256 一致后删除 RKOH 原目录：

| 原目录（RKOH `~/code/`） | 本机压缩包 | 字节数 | sha256 前缀 |
|---|---|---|---|
| gradua_prj（58G） | `F:\RKOH_backup\gradua_prj.tar.zst` | 24,275,289,799 | `10a309dc` |
| robotpi（131G） | `F:\RKOH_backup\robotpi.tar.zst` | 77,951,483,879 | `9f21e627` |
| OH_JAKA（263G） | `E:\RKOH_backup\OH_JAKA.tar.zst` | 158,173,430,143 | `fe6b85df` |

- 每个压缩包旁有 `.sha256` 文件；robotpi、OH_JAKA 用容器内 root tar 打包，root 所属文件（如 thinlto-cache）也已包含
- 恢复：把压缩包传回 RKOH，在 `~/code` 下 `zstd -dc <name>.tar.zst | tar -xf -`

## 有效结论
- 写满主因：PX4 SITL ulog 无上限。七机容器从 09-29 15:25 一直记录到 09-30 09:20（每架约 9.8G），CAT-ORA 容器继续写，约 13:06 写满
- 长时间保持仿真容器运行时，需关闭或限制 PX4 logger（如 `SDLOG_MODE=-1`），或验证结束后及时停容器
- RKOH 与其他会话共用：`~/code/UAV/OH` 有人在跑 OpenHarmony 编译（`./build.sh --product-name robopi --ccache`），约 1 小时内占用 ~100G。做占盘操作前先看 `df`，并留余量

## 卡住的问题
无。

## 下一步
1. 下次启动 CAT-ORA/swarm 容器前，在部署脚本中给 PX4 logger 加限制
2. 可选：Docker 构建缓存 22.5G 可 `docker builder prune` 回收

## 踩过的坑
- RKOH 登录 shell 是 zsh，`==`、`${PIPESTATUS}` 等会出错；复杂命令用 `ssh RKOH 'bash -c "..."'`
- `~/code` 下有 Docker 编译留下的 root 所属文件，普通用户删不掉也读不了；RKOH 无免密 sudo。解决：打包用容器内 root tar（源目录只读挂载）；删除用 `docker run --rm --log-driver none -v <目录>:/w --entrypoint find uav/px4-gazebo:v1.17.0-harmonic /w -mindepth 1 -delete`
- 容器 stdout 输出 tar 流时，Docker 默认 json-file 日志驱动会把整条数据流另存为日志，几分钟吃掉 130G。必须加 `--log-driver none`
- 本机 PowerShell 5 的 `>`/管道会把二进制流当文本处理；接收流式 tar 要用 `cmd /c "ssh ... > file"`
- 打包在 RKOH 本地会占同等空间；空间不足时改为流式直传本机（RKOH 不占空间），局域网约 57–66 MB/s
- SSH 提示 `remote port forwarding failed for listen port 7890` 是代理端口已被其他会话占用，不影响操作

## 相关指令
- 扫描：`ssh RKOH 'du -h --max-depth=2 ~/code | sort -hr | head -80'`；大文件：`find ~/code -xdev -type f -size +2G -printf "%s\t%p\n" | sort -nr | numfmt --to=iec --field=1`
- 本地打包脚本 `%TEMP%\pack_dir.sh <name>`（传到 RKOH `/tmp` 后运行）：容器 root tar | zstd → `~/<name>.tar.zst`，剩余空间 <5G 自动中止并删除半成品，完成后 `zstd -t` 校验并生成 `.sha256`
- 流式直传脚本 `%TEMP%\stream_dir.sh <name>`：`cmd /c "ssh RKOH bash /tmp/stream_dir.sh <name> > E:\RKOH_backup\<name>.tar.zst 2> <err>"`，RKOH 端写 `~/<name>.tar.zst.sha256` 与 `~/stream_<name>.status`（0 为成功）
- 本机校验：`(Get-FileHash <file> -Algorithm SHA256).Hash.ToLower()`
