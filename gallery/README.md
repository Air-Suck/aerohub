# Aerohub Gallery

本目录是仓库网站工程，承载项目主页、Gallery 列表和算法详情。采用 Astro 静态构建，本地开发与 GitHub Pages 使用同一份内容和页面代码。当前仅有框架占位条目，样式、真实算法展示和交互回放后续完善。

## 本地使用

需要 Node.js 24 和 npm。仓库根目录执行：

```bash
cd gallery
npm ci
npm run dev
```

访问终端输出的地址，默认 `http://127.0.0.1:4321/`。不需要 ROS、Docker、AS2 或算法构建环境。

Astro 7 的开发服务在后台运行，可用 `npm run dev -- status` 查看状态，用 `npm run dev -- stop` 停止。限制用户配置目录写入的环境可在命令前设置 `ASTRO_TELEMETRY_DISABLED=1`。

```bash
npm run build
npm run preview
```

`build` 先检查内容与类型再生成 `dist/`。依赖由 `package-lock.json` 锁定；`node_modules/`、`.astro/` 与 `dist/` 不进入 Git。

## 文件归属

- `src/`：网页路由、布局、组件、样式与通用展示逻辑。
- `src/content/homepage.md`：首页文案与 feature 卡片内容；只改 Markdown 即可更新首页。
- `content/gallery.json`：`items` 声明收录条目的 `id` 和 `markdown`。路径相对 `gallery/`，且必须指向 `content/` 内的 Markdown，例如 `content/algorithms/<id>/index.md`；稳定 ID 决定 `/gallery/<id>/` URL。
- 条目 Markdown：frontmatter 提供卡片及状态信息，正文提供详情。不在 `items` 中登记的 Markdown 不进入 Gallery。
- `schema/algorithm.ts`：元数据字段及约束的唯一实现。
- `public/`：直接复制到站点的公共资源；算法下载文件放 `public/algorithms/<algorithm_id>/`。
- 算法专属、由 Markdown 引用的图片可放条目旁的 `assets/`，使用相对引用。验收摘要可写入正文；大型视频、rosbag 和原始日志通过外部资源引用，不复制算法源码或运行环境进网站。
- 正式迁移 Skill 在仓库根目录 `.agents/skills/`；算法实现仍在根目录 `src/`，运行部署仍在 `deployment/`。

采用 Markdown 原生 YAML frontmatter，合并之前讨论的 `algorithm.yaml` 和 `index.md`，避免为同一条目维护额外加载器。新增条目可复制示例 Markdown，修改元数据与正文，再在 `content/gallery.json` 添加 item。没有适配代码也能生成页面。例如：

```json
{
  "items": [
    { "id": "my_algorithm", "markdown": "content/algorithms/my_algorithm/index.md" }
  ]
}
```

JSON 中保留其他已有条目；每个 ID、Markdown 路径均须唯一，指向已存在的 Markdown。暂未迁移的算法可把展示文件先放在 `gallery/content/algorithms/`，无需提前创建运行代码。

网页的 Markdown、条目配置、资源与构建产物均放在 `gallery/` 内。Skill 可以读取仓库根目录 `src/` 中的说明，并将展示内容整理到 `gallery/content/`；网页构建与开发服务只读取展示副本，不直接加载或监听仓库根目录。

本地执行 `npm run bundle` 即可校验并生成网页，它与 `build` 共用同一入口。`bundle-gallery` Skill 负责阅读源 Markdown、对照网页并同步展示文案；命令本身不自动总结任意 README。`gallerize-proj` 暂仅占位，后续负责新增内容和登记 item 后衔接 bundle。样式和布局继续由现有模板管理。

算法 Markdown 的 frontmatter 示例：

```yaml
---
title: 算法名称
summary: 一句话说明能力
category: planner
source: https://github.com/owner/algorithm
integration: not_started
validation: unverified
---
```

分类为 `planner`、`behavior`、`state_estimator`、`perception`。接入状态为 `not_started`、`in_progress`、`integrated`。验证记录为 `unverified`、`build`、`interface`、`simulation`、`hardware`。验证记录描述证据范围，不等于通用验收认证；具体条件与限制写入正文。

验证记录不是 `unverified` 时，必须填写 `revision` 和至少一条 `evidence`（`label`、HTTP(S) `url`），缺失时构建失败。公开页面不能仅引用读者无法访问的本地 `log/`。未整理公开证据时保留 `unverified`，避免将历史描述自动提升为验收结论。

如遇旧开发进程留下的 `ENOSPC` 文件监听上限错误，先停止再启动：`npm run dev -- stop`、`npm run dev`。本项目曾因内容加载器监听仓库根目录而扫描构建产物，现限制为 Gallery 内部。`npm run check:watch` 可启动临时开发服务检查监听范围和页面响应，结束后自动停止。

## CI/CD

`.github/workflows/gallery-check.yml` 检查 PR 和源码更新，安装锁定依赖并验证根路径及子路径构建。仅检出网站所需的外层仓库，不拉取大型上游子模块。

`.github/workflows/gallery-deploy.yml` 提供 GitHub Pages 发布配置：默认 `main` 更新时运行，也可从默认分支手动触发；若仓库默认分支不是 `main`，同步修改 `push.branches`。首次启用时，在仓库 **Settings → Pages → Source** 选择 **GitHub Actions**。配置文件本身不会启用 Pages，本次仅本地开发，不执行发布。

发布任务从 Pages 配置读取域名和路径，支持项目站点子路径。可以在本地复查：

```bash
SITE_URL=https://example.github.io BASE_PATH=/aerohub/ npm run build
BASE_PATH=/aerohub/ npm run preview
```

访问 `/aerohub/`。页面内部导航统一处理路径前缀；在 Markdown 中编写站内链接时使用相对路径，避免硬编码以 `/` 开头的根路径。页面条目与算法接入状态相互独立，未迁移条目也会进入站点。

参考：[Astro 内容集合](https://docs.astro.build/en/guides/content-collections/)、[GitHub Pages 部署](https://docs.astro.build/en/guides/deploy/github/)。
