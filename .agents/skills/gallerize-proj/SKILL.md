---
name: gallerize-proj
description: 从项目源码与说明建立 Gallery 条目，展示 lib 实现、AS2 Behavior runtime 绑定和多机关系；维护内容 Markdown 与条目配置后调用 bundle-gallery。
---

# Gallerize Proj

将一个算法项目整理成可核查的 Gallery 页面。先读仓库约定及 `gallery/README.md`，核对 Git 差异，保持增量修改。需要建立能力与进程映射时使用 `uav-architecture-analysis`；这里的 lib/runtime 是展示视图，不改变仓库既有运行架构。

## 两层模型

一个 lib 算法条目由 **lib 实现 + runtime 绑定**构成：

- **Lib**：AS2 Behavior 以外的算法与接入实现，包括求解器、代理、adapter、任务/机群协调等。可以按输入输出或职责拆分，也可保留黑盒。注明共用组件及进程边界；lib 不等于必须编译成共享库或合并成单进程。
- **Runtime**：实际执行所绑定的 AS2 Behavior。给出原生类/扩展组件、Action 类型和地址、调用方、调用阶段及前置条件。区分原生接口和本仓库扩展接口；加载了某 Behavior 不等于算法调用它。只提供数据且不调用 Behavior 的项目应如实标明“无直接 Behavior 调用”，未迁移项目只能标记拟议绑定。

平台、控制器和传感器等作为依赖说明，不伪装成算法独立实现或新增第三个算法层。保持 Planner 生成计划、Behavior 执行计划的实际职责。

## 源码核查

1. 读取用户指定项目的 README、版本锁定、配置与入口，沿实际调用追踪到 proxy、adapter、机群协调器及 Behavior。记录来源文件和符号；子模块未检出或没有日志时明确缺口，不自动初始化上游、构建或运行算法。
2. 建立 lib 模块的输入 → 输出 → 责任表，核实 runtime 的真实调用链。标明起飞等前置调用是否由测试/部署入口负责；不要把它写成算法核心自己调用。
3. 对多机项目，核实共享规划次数、车辆 ID/namespace、目标归属、轨迹时间与坐标转换、共同基时间、逐机 Goal、确认期限和失败收口。说明配置声明规模、展示规模和实际验收规模的区别；逐机调用不等于自动同步。
4. 区分源码核查、历史验收记录和本次运行证据。默认不运行 ROS、仿真或飞行任务。页面的验证等级遵循已有 schema，不能把结构图或配置目标解释为实际轨迹/测量结果。

## 内容落盘

- 所有网页内容与资源写入 `gallery/content/algorithms/<id>/`：`index.md` 保存 frontmatter 和正文，`assets/` 保存条目专属图示等小型资源。
- 在 `gallery/content/gallery.json` 的 `items` 中追加稳定 ID 与 `content/algorithms/<id>/index.md`，已有 ID 则原位更新，避免重复。
- 源 Markdown 可从仓库其他目录读取，但网页不得直接加载外部源文件；保留源文件原文。所有构建、监听和产物均限制在 `gallery/` 内，Skill 保持默认 `.agents/skills/` 路径。
- 默认不修改样式、模板或主文档。现有模板可承载的表格、图示和说明直接放 Markdown，不为单个项目扩展页面框架。

正文需要能回答：项目做什么；lib 如何组织；调用哪些 runtime Behavior；多机如何实例化与协调；哪些事实已核实、哪些尚待验证。结构按项目选择，不机械复制不适用章节。

多机优先用“一个共享 lib → 多个带 namespace 的 runtime 实例”的调用图及对应表呈现。SVG 可作为原生可编辑资源，配标题、描述、Markdown 替代文字和文字表格。未提供实际运行产物时使用结构示意，不编造飞行轨迹或动画结果。

参考已核查案例：`gallery/content/algorithms/cat_ora/index.md` 及 `assets/layers.svg`。案例的版本、机数、坐标约定和验收结论只适用于该项目，不作为其他算法默认值。

## Bundle 与交付

读取并执行同目录体系中的 `bundle-gallery` Skill，默认完成内容后自动本地 bundle。检查 Gallery 列表、详情、图示资源及站内链接，确认样式未改。汇报源材料、lib/runtime 绑定、验证范围及待办；不自动推送或发布。

完成后把本次可复用的展示约束回馈本 Skill。算法复现经验归对应架构/适配/验证 Skill；仅做页面整理时不升级算法验收结论。
