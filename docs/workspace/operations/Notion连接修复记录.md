# Notion连接修复记录

2026-10-13已完成授权，并通过MCP初始化加载49个Notion工具，包括`notion-search`和`notion-fetch`。本轮没有写入Notion页面，也没有将本地survey同步到远端。

## 原因与修复

授权网址同时包含两个`resource`值：`https://mcp.notion.com/mcp`和`https://mcp.notion.com`。前者由客户端发现，后者来自插件显式配置，Notion返回`invalid_target`及`Only one resource parameter is supported.`。

移除`C:/Users/Lenovo/.codex/plugins/cache/openai-api-curated/notion/0722921d/.mcp.json`中的`oauth_resource`，保留`url: https://mcp.notion.com/mcp`。原文件备份在`F:/UAV/tmp/notion-auth-20261013-160554/plugin-mcp-before.json`。

使用桌面应用自带CLI重新授权：

```powershell
& 'C:/Program Files/WindowsApps/OpenAI.Codex_26.1007.2314.0_x64__2p2nqsd0c76g0/app/resources/codex.exe' mcp login notion
```

该版本为`0.162.0-alpha.17.2`。新请求只有一个`resource`，登录返回成功；随后`mcp list`显示OAuth，`app-server`的`mcpServerStatus/list`返回`oAuth`及49个工具。没有读取或打印访问令牌。

## 后续使用

在新会话中加载Notion工具，或从应用设置重启MCP连接。已有会话未必会更新工具列表。插件更新可能覆盖此缓存修复；若错误重现，先核对重复参数，不再添加同名全局服务器。当前系统PATH仍指向旧Cursor插件CLI，修复期间未修改PATH。

本次仅验证工具加载；远端survey正文的读取和本地增补内容同步需在后续明确任务中执行。
