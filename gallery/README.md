# Aerohub Gallery

本目录是仓库网站工程，承载项目主页、Quickstart、Gallery 和 Docs 文档中心。采用 Astro 静态构建，本地开发与 GitHub Pages 使用同一份内容和页面代码。当前展示 CAT-ORA、Swarm-Formation、SUPER、FAST-LIVO2、YOPO 和 IPC，以官方 Demo、简短介绍和 Tool / Behavior / Function 三层接入图为核心，接入细节和验证记录放在折叠区域。示例 Markdown 保留为新增条目模板，不登记到公开列表。

本轮 CAT-ORA、SUPER、YOPO 的展示改进设计见 [算法展示改进约定](plans/algorithm-presentation.md)，该文档记录已确认目标与实现检查记录。

## 网站结构与文档维护

- `/`：项目介绍，以及 Quickstart、Gallery、Docs 三个入口。
- `/quickstart/`：首次使用路线，当前提供环境准备、Demo 选择和检查顺序；完整可复制教程待补。
- `/gallery/` 与 `/gallery/<id>/`：能力展示与接入记录，详情链接到使用指南。
- `/docs/`：文档中心，按开始使用、用户指南、深入了解分组。
- `/docs/<id>/`：文档正文，共用侧栏、本页目录与前后翻页。

文档内容位于 `content/docs/*.md`；frontmatter 必填 `title`、`description`、`section`、`order`、`status`。`section` 为 `开始使用`、`用户指南`、`深入了解`；`status` 为 `初版` 或 `待完善`。文件名决定稳定 URL，其中 `quickstart.md` 单独映射到 `/quickstart/`。当前路由只支持单层文件名，请勿创建嵌套目录。`order` 决定文档导航与翻页顺序。新增文档自动进入集合与文档中心。

文档正文使用 Markdown；跨页面内部链接使用相对路径，确保 GitHub Pages 子路径部署有效。内容缺失时写明待补项，不虚构运行命令或验收结果。技术事实继续以仓库对应主文档和 Demo 说明为来源；网站维护展示副本，构建不读取仓库外层。首批内容根据本工作区 2026-11-10 的部署、Demo、项目简介与共享架构约定整理；仓库外链用于查阅对应说明，不将其视为锁定版本的验收证据。

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

Astro 内容缓存保存在当前工程的 `.astro/cache/`，即使 `node_modules` 软链接到其他工作区也保持内容缓存隔离。

Gallery 登记由 `registered-algorithms` loader 在加载和 `content/gallery.json` 变更时重新读取；新增、删除和清空登记会更新运行中的内容集合，无需重启用户服务。`npm run check:registry-watch` 在 Gallery 内隔离副本验证此行为，不改实际登记文件。

已有 dev 服务与静态构建分别验证。修改内容 schema 后，已有服务可能未重新加载新增字段；出现“构建产物有图、开发页面无图”时，先检查当前页面 HTML 及其图片请求，具体排查见 `skill/bundle-gallery/SKILL.md`。开发服务由使用者启动和停止。

## 文件归属

- `src/`：网页路由、布局、组件、样式与通用展示逻辑。
- `src/content/homepage.md`：首页文案与 feature 卡片内容；只改 Markdown 即可更新首页。
- `content/gallery.json`：`items` 声明收录条目的 `id` 和 `markdown`。路径相对 `gallery/`，且必须指向 `content/` 内的 Markdown，例如 `content/algorithms/<id>/index.md`；稳定 ID 决定 `/gallery/<id>/` URL。
- 条目 Markdown：frontmatter 提供卡片及状态信息，正文提供详情。不在 `items` 中登记的 Markdown 不进入 Gallery。
- `schema/algorithm.ts`：元数据字段及验证证据约束；内容配置入口使用严格校验，未知元数据字段报错。
- `public/`：直接复制到站点的公共资源；算法下载文件放 `public/algorithms/<algorithm_id>/`。
- 算法专属图片放条目旁的 `assets/`，正文图片使用相对引用。Demo 封面在 frontmatter 使用相对 `gallery/` 的内容路径，见下方示例；保留原始来源及必要的许可信息。本地封面随站点构建，读取真实图像宽高，避免依赖外部图片域名和固定比例。大型视频、rosbag 和原始日志通过外部资源引用，不复制算法源码或运行环境进网站；预览截图留在根目录 `log/gallery-review/`，不提交。
- 所有 Skill 的实现统一在仓库根目录 `skill/`，包括 `gallerize-proj/` 和 `bundle-gallery/`；`.agents/skills/` 仅通过独立相对软链接启用选定的 Skill。算法实现仍在根目录 `src/`，运行部署仍在 `deployment/`。

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

本地执行 `npm run bundle` 即可校验并生成网页，它与 `build` 共用同一入口。`bundle-gallery` Skill 负责阅读源 Markdown、对照网页并同步展示文案；命令本身不自动总结任意 README。`gallerize-proj` 负责核查 Tool / Behavior / Function 三层的能力覆盖与依赖，查找官方 Demo、新增内容和登记 item 后衔接 bundle。样式和布局继续由现有模板管理。

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

详情页顶部提供官方仓库及已配置的官方视频链接；列表和详情均用灰色“未迁移”、橙色“迁移中”、绿色“已接入”标记迁移阶段，保留文字标签。源码版本与验证数据继续保留在元数据和补充说明中，不再展示为顶部信息表；“已接入”不等于仿真或真机验收通过。

需要 Demo 展示时，在 frontmatter 添加以下可选字段，并将实际封面放入对应条目的 `assets/`：

```yaml
demo:
  image: content/algorithms/my_algorithm/assets/demo-cover.jpg
  alt: 描述画面内容，点击观看官方演示
  caption: 作者公开的演示；不代表本项目已取得相同验收结果。
  source: https://github.com/owner/algorithm#demo
  video: https://www.youtube.com/watch?v=VIDEO_ID
```

`image` 也支持 HTTP(S) 地址，本地素材优先。未配置 `showcase` 的条目沿用封面、简介与关键词的原有布局。SUPER / YOPO 使用下节的桌面紧凑展示；CAT-ORA 等其他条目按明确任务逐步迁移。

核心图聚焦算法自身方法，三层图解释能力链路；技术接口、进程和运行记录放在 `<details>`，案例按任务要求留空或提供真实内容。宽图使用可聚焦的 `.diagram-scroll` 保留窄屏横向滚动。上游素材不计入本项目验证证据。

## 桌面优先的素材展示

页面顺序为：**主视频与精华说明 → 应用卡片 → 算法核心（方法卡片与核心图）→ 三层框架 → 案例 → 折叠技术详情**。桌面为主要使用场景，素材承担引导与解释作用，避免占满宽屏或把核心内容推到很后面。

- 顶部左视频约 70%、右文字约 30%；右侧放核心设计、少量精华说明及 `keywords`，不重复开场简介。当前主视频为封面点击跳转，优先 Bilibili，其他平台保留小型链接。
- 应用卡片内部左图右文，桌面每页两张、手机每页一张。一页放得下时隐藏翻页控件，多页支持左右按钮、横向滚动与键盘方向键，不自动轮播。
- 动图默认静态封面，点击后原位循环，播放图标消失，没有开始/收起按钮。图像 contain 显示，固定媒体高度，方形图和宽图不能撑高卡片或使翻页跳动。
- 算法核心里的上游方法图同样使用紧凑图文卡片，每张讲一个要点，保留自己的核心流程图。上游结果声明仅在整个展示模块底部出现一次，出处与必要署名集中在“来源”。

### 字段与资源

字段约束以 [schema](schema/algorithm.ts) 为准；紧凑展示组件为 [ProjectShowcase](src/components/ProjectShowcase.astro)。`showcase` 必须与 `demo` 一起配置。

| 字段 | 内容与约束 |
| --- | --- |
| `keywords` | 最多 8 个关键词；紧凑布局显示在右侧文字下方 |
| `showcase.headline` | 核心设计的一句话标题 |
| `showcase.highlights` | 1–3 项 `title` / `text`，解释关键机制 |
| `showcase.methodSources` | 可选 `label` / HTTP(S) `url` 列表，登记正文方法图来源 |
| `showcase.sourceNote` | 可选合作署名等来源补充 |
| `videos` | 最多 6 项 `label` / HTTP(S) `url`，提供其他视频入口 |
| `media` | 最多 6 张卡片，包含 `title`、`image`、`animation`、`alt`、`caption`、`source` |

`media.image` 和 `media.animation` 当前均为条目 `assets/` 内的 WebP，字段路径以 `content/algorithms/<id>/assets/` 开头；图片作静态封面，动画通过原始资源 URL 发布以保留多帧。素材从上游仓库、README 链接到的项目网站和作者页面搜集，实际查看后筛选；论文暂不纳入默认素材搜集。Bilibili 优先仍须核查内容与作者对应关系，相关介绍和转载不能冒充官方演示。

配置示例（省略已有 `demo` 等基础字段）：

```yaml
keywords: [未知环境导航, 滚动重规划]
showcase:
  headline: 用一句话点出算法核心设计
  highlights:
    - title: 关键机制
      text: 解释它解决的具体问题。
  methodSources:
    - label: 上游方法示意
      url: https://github.com/owner/algorithm#method
  sourceNote: 必要的作者或合作工作署名。
videos:
  - label: YouTube
    url: https://www.youtube.com/watch?v=VIDEO_ID
media:
  - title: 场景应用
    image: content/algorithms/my_algorithm/assets/scene-poster.webp
    animation: content/algorithms/my_algorithm/assets/scene.webp
    alt: 对画面内容的具体描述
    caption: 说明应用价值或观察要点。
    source: https://github.com/owner/algorithm/blob/REV/docs/scene.gif
```

正文静态方法卡片沿用 YOPO 的结构；Markdown 图片两侧留空行，确保内容处理器解析图片：

```markdown
<div class="method-cards" aria-label="上游方法示意">
<figure class="method-card">
<div class="method-card-image">

![方法示意的具体替代文字](./assets/method.webp)

</div>
<figcaption><h3>方法要点</h3><p>解释该图说明的关键机制。</p></figcaption>
</figure>
</div>
```

所有来源、锁定版本、作者归属、许可与转换方式记录在条目 `assets/SOURCES.md`；动画转码或隔帧采样须保留累计时长与播放速度，等比缩放不裁切，查看转换结果。网络不可用时可先采用本地锁定素材，但要明确网站补充搜集及外部播放仍未核验。

### 检查与当前覆盖

执行 `npm --prefix gallery run bundle`，再检查用户当前开发服务。桌面 1440 / 1920、手机约 390 视口分别查看：关键词、图文比例、图片解码、动图循环、固定高度、全部分页、首尾按钮、方向键、来源展开、宽图滚动与整页溢出；静态构建不代替在线显示检查。检查截图与报告保存在根目录 `log/gallery-review/`。

当前 SUPER 两张应用卡片桌面一页；YOPO 四张（森林实飞、深度视觉平台、随机场景导航、交互目标导航）桌面两页、手机四页，其两张上游方法图位于算法核心并采用统一卡片样式。两页上述布局与交互已检查；外部视频播放未独立核验。其他条目不因本约定自动批量迁移。

## 验证记录与开发服务

验证记录不是 `unverified` 时，必须填写 `revision` 和至少一条 `evidence`（`label`、HTTP(S) `url`），缺失时构建失败。公开页面不能仅引用读者无法访问的本地 `log/`。未整理公开证据时保留 `unverified`，避免将历史描述自动提升为验收结论。

如遇旧开发进程留下的 `ENOSPC` 文件监听上限错误，先停止再启动：`npm run dev -- stop`、`npm run dev`。本项目曾因内容加载器监听仓库根目录而扫描构建产物，现限制为 Gallery 内部。`npm run check:watch` 可启动临时开发服务检查监听范围和页面响应，结束后自动停止。

## CI/CD

`.github/workflows/gallery-check.yml` 在 Gallery 或其工作流改动推送到 `dev` 后执行检查，安装锁定依赖并验证根路径及子路径构建；不再对 PR 和其他分支 push 运行。支持手动触发，但仅在 `dev` 分支执行。仅检出网站所需的外层仓库，不拉取大型上游子模块。

`.github/workflows/gallery-deploy.yml` 在 Gallery 或其工作流改动推送到 `main` 后构建并发布 GitHub Pages；手动触发也仅在 `main` 分支执行。两个工作流均固定使用 `ubuntu-24.04`，避免 `ubuntu-latest` 标签自动迁移造成环境变化。首次启用时，在仓库 **Settings → Pages → Source** 选择 **GitHub Actions**。配置文件本身不会启用 Pages，本次不执行远程发布。

如果任务尚未启动，且运行注释提示账号因 billing 问题锁定，应先处理账号侧阻塞；这不属于网站构建失败，也不能通过修改工作流解除。账号恢复后仍需实际验证远程检查和发布结果。

发布任务从 Pages 配置读取域名和路径，支持项目站点子路径。可以在本地复查：

```bash
SITE_URL=https://example.github.io BASE_PATH=/aerohub/ npm run build
BASE_PATH=/aerohub/ npm run preview
```

访问 `/aerohub/`。页面内部导航统一处理路径前缀；在 Markdown 中编写站内链接时使用相对路径，避免硬编码以 `/` 开头的根路径。页面条目与算法接入状态相互独立，未迁移条目也会进入站点。

参考：[Astro 内容集合](https://docs.astro.build/en/guides/content-collections/)、[GitHub Pages 部署](https://docs.astro.build/en/guides/deploy/github/)。
