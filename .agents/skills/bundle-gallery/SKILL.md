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
2. 对照源材料与页面字段。首页更新其 frontmatter；算法展示统一写入 `gallery/content/algorithms/<id>/index.md`，从源 Markdown 提炼内容并标明来源，保留源文件原文。在 `gallery/content/gallery.json` 中登记相对 `gallery/` 的 `content/algorithms/<id>/index.md`。网页构建与开发监听仅限 Gallery 内部，不能把内容加载器根目录设为仓库根目录。
3. 默认只改展示 Markdown 和必要的 JSON 条目。保留 ID、既有样式、CSS 类及页面布局；已有模板无法表达用户要求时说明具体缺口，不借内容同步重做风格。不要顺带改写根 README、主文档或其他算法。
4. 从仓库根目录运行 `npm --prefix gallery run bundle`；仅当依赖缺失时先运行 `npm --prefix gallery ci`。沙箱限制用户配置写入时使用 `ASTRO_TELEMETRY_DISABLED=1`。失败后修复本次内容问题，不把构建失败当作已完成。
5. 检查 `gallery/dist/index.html`、`gallery/dist/gallery/index.html` 及涉及的详情页，核对内容、链接、状态和证据。交付源 Markdown → 网页对应关系、构建结果和待确认信息。生成的 dist 不提交，也不自动推送或部署。

`gallerize-proj` 完成 lib/runtime 核查、展示 Markdown 和条目登记后，衔接本 Skill 做内容对照与本地构建。检查图示与 Behavior 调用说明一致，多机结构示意不得被描述为实际飞行结果。
