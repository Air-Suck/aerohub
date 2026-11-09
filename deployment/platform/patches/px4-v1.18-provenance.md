# PR 的 PX4 1.18 平台补丁

`as2-platform-px4-v1.18.patch` 是以下两个源码树之间的完整差异：

- 基线：官方 `as2_platform_pixhawk` 的 `fcba062219935acef69dfa009c8be376c707ee8a`，与 main 子模块锁定版本相同。
- 目标：`https://github.com/Popc0rn7/as2_platform-aerohub.git` 的 `80f0f27a72efd053d029d3e6a76c3d5d473db6d2`，即原 PR 锁定的版本。

只在 AirSim / PX4 1.18 的构建副本上应用；主线 PX4 1.17 仍使用原来的补丁，不叠加两个版本的补丁。Docker 在 COPY 后应用；OHOS 在复制到隔离工作区后应用，不修改官方子模块。

此补丁涵盖参考时间戳、PVA 输出、命令回执和相关参数配置。补丁应用后的完整源码树已与目标提交逐文件核对相等；这不等于平台编译或飞行验证。

AS2 核心仍有独立缺口：原 PR 锁定的 `f387e0d733a5102808508bcbec1b9eca2c29d1f0` 未发布，当前无法生成保留该提交全部行为的核心补丁。平台补丁不能替代缺失的 AS2 核心源码。
