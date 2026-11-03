# 当前任务
- 已完成用户要求：cwd仅保留七个编号项目目录及四个隐藏目录；项目、文档、辅助工具、素材和临时文件均已归类。

## 已完成
- 已检查当前目录，无 `docs/HANDOFF.md`；已查看历史交接记录、目录结构和主要工程 README。
- 已原地移动目录（包括隐藏文件、Git元数据和未提交内容），以目录inode一致和旧路径README可读验证；未复制或删除项目内容。
- 用户明确要求直接使用mv整理、不保留原路径。七个兼容符号链接已删除；后续公共资料和辅助工具也已移入对应项目。

## 有效结论
- 按主要工程/交付项目计为7项：aerostack2-integration、catora-airsim、catora-px4-gazebo、swarm-formation-px4-gazebo、demo_code、lecturekit、ppt/uav-lesson。
- 新目录依次为 `01-aerostack2-integration`、`02-catora-airsim`、`03-catora-px4-gazebo`、`04-swarm-formation-px4-gazebo`、`05-demo-code`、`06-lecturekit`、`07-uav-lesson`；上列原路径已移除。
- 这是工作区主项目分类，不是独立Git仓库数量；UAV总体目标仍是1个大项目。
- 辅助工具、资料、临时产物和worktree不另计项目。完整归类见本目录工作区整理索引.md。

## 卡住的问题
- 无。

## 下一步
- 整理已完成。公共docs现位于01-aerostack2-integration/docs/workspace/；本交接文档也随之迁移。后续先读本文，交接文件在同目录维护。
- 原项目README、outline、素材说明保留；已更新新增文档入口的迁移链接。历史正文和代码运行路径未全面适配，未运行工程或构建PPT。

## 踩过的坑
- 历史交接包含 Windows 路径与过期运行状态，不能当作当前环境事实。
- `worktrees/lecturekit-layouts/.git` 原已指向 `F:/UAV/lecturekit/.git/worktrees/lecturekit-layouts`，本轮未修复此迁移遗留问题。
- 根AGENTS.md已按根目录仅留七项目的要求存档于本目录，不再自动作用于全工作区。
- version2.mp4、version4.mp4在迁移执行前已不在根目录，本次未移动或删除。

## 相关指令/链接
- `AGENTS.md`
- `文档整理索引.md`
- `工作区整理索引.md`
