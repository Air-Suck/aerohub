---
name: uav-package-component
description: 将 `uav-migrate-component` 已迁移并验证的无人机组件注册和交付；统一维护组件清单、能力卡片、Registry、配置、启动停止、健康、回滚和证据状态，不负责重新执行迁移或闭环验收。
---

# 打包无人机组件

输入是 [`uav-migrate-component`](../uav-migrate-component/SKILL.md) 的构建产物、Harness 接口和验证结论。打包只声明已有事实，不提升成熟度。

## 组件清单

复用仓库现有格式，至少记录：稳定组件 ID、版本、源码/补丁/许可证、镜像与架构、完整依赖、入口、配置、挂载、命名空间、时钟、端口、资源、持久状态和外部前置条件。

关联提供的 Function/Behavior/Inference 能力；共享进程、设备和不可独立启停项使用明确 coupling。声明状态、TF、地图、时钟和运动资源的发布权与冲突。

## 生命周期

记录安装、选择、启动顺序、ready/health、停止确认、更新和回滚。健康区分进程存活、核心加载、输入有效、输出新鲜与任务可用。回滚说明地图、隐状态或估计状态是否丢失。

## Registry 与能力卡片

Registry、清单和卡片引用同一组件版本、入口和证据。卡片按真实使用方式声明输入输出、调用权限、生命周期、资源、限制和验证范围；一项能力一张卡，共享实现通过组件 ID 关联。

验证状态使用 Harness 实际达到的枚举值。任何适用测试失败、未执行或缺证据时保持 `partial` 或此前级别，不生成 `reproduced`/`success` 声明。

## 交付

提供清单、卡片、注册项、配置 schema、启动/停止/检查命令、更新回滚方法和证据索引。需要 RoboPi 安装时转入 [`deploy-lib`](../deploy-lib/SKILL.md)，不在此 Skill 内复制板端构建或传输步骤。

