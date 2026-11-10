---
title: 开发者指南
description: 找到组件适配、能力卡片和贡献约定的正式入口。
section: 深入了解
order: 60
status: 待完善
---

## 开发与适配

[开发者贡献指导](https://github.com/Air-Suck/aerohub/blob/main/docs/开发者贡献指导.md)是组件扩展、接口和目录约定的主文档。新增能力应说明依赖、生命周期、取消与失败处理，并区分设计目标、实现情况与实际验证。

## 内容归属

自建实现位于 `src/`，部署入口位于 `deployment/`，能力卡片位于根目录 `cards/`。正式 Skill 位于 `skill/`；过程记录与原始运行日志分别使用 `skill/temp/` 和 `log/`。

## 网站维护

网站使用 Astro 与 Markdown，内容保存在 `gallery/` 内。页面构建不加载或监听仓库根目录。详见[网站维护说明](https://github.com/Air-Suck/aerohub/blob/main/gallery/README.md)。

待补：从新增组件到编写卡片、加入 Gallery 和提交验证证据的连续示例。
