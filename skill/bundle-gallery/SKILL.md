---
name: bundle-gallery
description: 对照仓库源 Markdown 与 Gallery 页面内容，同步首页和已登记的算法展示并构建网站；保持现有样式和布局。
---

# Bundle Gallery

在仓库根目录执行。输入是用户指定的源 Markdown（尤其是 `src/` 下的算法说明）及现有网站内容；输出是更新后的展示 Markdown、必要的条目配置和本地静态构建。内容同步由 Agent 阅读理解完成，`npm run bundle` 负责校验和构建，不自动总结任意文档。

## 内容对应

| 网页 | 内容来源 | 展示入口 |
| --- | --- | --- |
| 首页标题、简介和 feature 文案 | `gallery/src/content/homepage.md` 的 YAML frontmatter | `gallery/src/pages/index.astro` |
| Gallery 收录范围、稳定 ID、Markdown 路径 | `gallery/content/gallery.json` 的 `items` | `/gallery/` |
| 条目卡片、状态与详情 | 每个 item 的 `markdown`，相对 `gallery/`，限定为 `content/` 内的文件 | `/gallery/<id>/` |

字段与约束以 `gallery/schema/algorithm.ts` 和 `gallery/src/content.config.ts` 为准，使用说明见 `gallery/README.md`。

## 同步过程

1. 查看 Git 差异，读取用户指定的源 Markdown、首页 Markdown、条目配置和本次涉及的条目。源信息不足时标明未知；不要从构建、合成输入或论文描述推导出仿真或真机验收。
   使用 worktree 时先比较展示分支与用户指定源工作区的文档版本。若源文档较新，记录源提交及采用的未提交内容，在页面标注来源版本；仅同步展示副本，无需将源分支的代码或主文档一并合入。主文档同时包含历史与当前记录时，按算法、平台、规模和日期限定结论；专项后续结果仅更新对应条目。
2. 对照源材料与页面字段。首页更新其 frontmatter；算法展示统一写入 `gallery/content/algorithms/<id>/index.md`，从源 Markdown 提炼内容并标明来源，保留源文件原文。在 `gallery/content/gallery.json` 中登记相对 `gallery/` 的 `content/algorithms/<id>/index.md`。网页构建与开发监听仅限 Gallery 内部，不能把内容加载器根目录设为仓库根目录。
3. 默认只改展示 Markdown 和必要的 JSON 条目。保留 ID、既有样式、CSS 类及页面布局；已有模板无法表达用户要求时说明具体缺口，不借内容同步重做风格。不要顺带改写根 README、主文档或其他算法。
4. 从仓库根目录运行 `npm --prefix gallery run bundle`；仅当依赖缺失时先运行 `npm --prefix gallery ci`。沙箱限制用户配置写入时使用 `ASTRO_TELEMETRY_DISABLED=1`。失败后修复本次内容问题，不把构建失败当作已完成。
5. 检查 `gallery/dist/index.html`、`gallery/dist/gallery/index.html` 及涉及的详情页，核对内容、链接、状态和证据。交付源 Markdown → 网页对应关系、构建结果和待确认信息。生成的 dist 不提交，也不自动推送或部署。

## 当前页面未更新时的排查与交付

开发预览和静态构建分别验证。用户报告“仍看不到”时，以其实际访问的服务为准，优先恢复该页面显示，不能用离线截图或重启建议代替修复结果。

1. **确认页面来源**：只读检查实际 URL 返回的 HTML，必要时核对服务进程的工作目录，避免把主工作区、worktree 和 dist 混为一处。开发服务由用户启动时，不自行启停或接管。
2. **区分节点缺失与资源失败**：HTML 没有预期 Demo 节点时，检查内容加载、schema 和条件渲染；已有节点时，提取 HTML 中的实际图片 URL，请求并检查状态码、Content-Type 及图像内容。使用相对 URL 时以页面地址解析，不根据磁盘文件存在就声称资源可访问。
3. **核查内容配置缓存**：Astro 7.4.7 的内容配置摘要以 `src/content.config.ts` 入口文件为依据，独立 `schema/algorithm.ts` 的更新可能未触发正在运行的内容层重新解析。已观察到模板、简介均更新，但新增 `demo` 字段缺失的情况；静态构建成功不能排除此故障。
4. **实施修复并核对在线结果**：当前集合使用 `algorithmSchema.strict()`，让未知元数据显式报错而非静默丢弃。本次在内容配置入口加入该校验后，已有 dev 服务通过热同步恢复了 Demo 字段。此处同时发生配置重新加载，不能据此声称 strict 校验本身修复了通用依赖监听。后续按具体原因修复；确需重启且服务由用户管理时，说明原因并由用户操作。
5. **完成显示验证**：再次读取同一服务，确认 Demo 节点、官方链接与状态标记存在；请求该 HTML 使用的图片 URL，确认 200 及正确图片类型；最后查看该服务的实际页面截图。分别报告在线显示、静态构建和未覆盖项。

2026-11-10 SUPER 案例：修复前当前服务只输出架构图，没有 Demo 节点；修复后输出本地封面，图片请求为 `200 image/jpeg`、58,184 字节，并已查看当前服务截图。未启停用户服务。封面随站点打包并保留原图来源，避免首屏依赖外部图片域名。该案例说明本次恢复与验证方法，不是所有 schema 修改均自动重载的保证。

`gallerize-proj` 完成三层覆盖与依赖核查、展示 Markdown 和条目登记后，衔接本 Skill 做内容对照与本地构建。检查 Tool、Behavior、Function 三层图与实现覆盖、调用及依赖说明一致，多机结构示意不得被描述为实际飞行结果。

## 新增文档集合与共享依赖缓存

新增 Quickstart / Docs 集合后，检查当前开发服务的 `/quickstart/`、`/docs/` 和正文页，确认 HTTP 200 且正文存在。运行 `npm --prefix gallery run check:watch` 覆盖开发模式；静态构建成功不能替代开发服务检查。

如果 worktree 的 `node_modules` 是共享软链接，先检查 Astro 内容缓存实际位置。网站配置将 `cacheDir` 放在本地 `.astro/cache/`，避免不同工作区共用依赖目录中的内容缓存。Vite 的依赖缓存仍可能位于共享 `node_modules/.vite/`，其写入权限需独立处理。

2026-11-10 新增 Docs 后，当前服务 `/quickstart/` 返回 500；配置本地缓存并触发配置重载后，同一服务返回 200，标题与 Markdown 正文存在，开发回归检查覆盖全部新增页面及 Gallery。配置重载与缓存隔离同时发生，因此尚未独立证明缓存共享是唯一原因。
