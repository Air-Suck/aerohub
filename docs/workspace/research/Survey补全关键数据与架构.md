# 交接文档

对话 hash：`47c2da13-3b08-42bb-a072-bcdfcce6052e`

## 当前任务

接手全新工作（`docs/` 原先无 `HANDOFF.md`，已按「开始」流程新建本文件）。等待用户下达下一步具体需求。

## 本对话已完成

Survey Notion 页面（`faf8daea7f2783d98b0401f725be03aa`）补全：

- 第 16–140 条亮点追加「关键数据」，并在亮点后写入「架构」行（法规 32–34 故意不加架构）
- 修正 6 处与来源不符的亮点（6X Pro、VINS-Multi、seL4/HACMS、抗隐蔽攻击、ROSflight、AirSim）及推扫式避障「建议核对」
- 恢复意外缺失的第 1 条「2021 IMAV 墨西哥」
- 终审：140 条；无 TOC/子页；比赛均有推荐队伍

## 已知缺口（Survey）

- 无关键数据（无可核实数字）：58, 77–80, 83, 84, 86–88, 91, 93, 115, 122, 127, 132, 133
- 视频未找到：约 14 条（无人争锋、部分控制论文、FlyOS、TOGT、GS-LIVO 等）
- 法规条目无架构行：32–34（有意）

## 卡住的问题

无。

## 下一步计划

等用户明确本轮目标（继续 Survey、还是转向飞控/国产 OS/demo 等）。

## 踩过的坑

- Notion `update_content` 的 `old_str` 必须唯一且精确匹配
- 条目计数须按含「简介」的块，不能只数 `###` 标题
- 审计脚本对 draft 数字的宽松匹配会误报；以页面内是否含「关键数据」/「架构」为准

## 相关指令 / 资源

- Notion Survey：`notion://` / page_id `faf8daea7f2783d98b0401f725be03aa`
- 本地草稿与审计：`C:\Users\Lenovo\.cursor\projects\f-UAV\agent-tools\`（`batch*.json`、`audit.py`、`fill_data.py` 等）
- 工作区：`F:\UAV`（目前仅 `AGENTS.md`、空 `docs/`、`.survey_tmp.md`）
