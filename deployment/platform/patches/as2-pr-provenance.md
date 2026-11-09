# AS2 PR 构建补丁

- 官方基线：`19c397309c6a41b279e6b551818ec12605cfcecc`，保持 main 的子模块版本。
- 目标：`https://github.com/Popc0rn7/aerostack2-aerohub.git` 的 `f387e0d733a5102808508bcbec1b9eca2c29d1f0`。
- 文件：`as2-airsim-pr.patch`，由两个提交的完整 `git diff --binary` 生成。应用后的 Git 源码树与目标提交相等。

补丁包含 fork 相对 main 的上游基线更新，以及单点轨迹执行、起飞完成条件、executor 复用等修改。大量内容是上游锁文件，不能将整个差异都归为 PR 作者原创。保留原始版权和许可证。

Docker 在复制官方源码后应用补丁；OHOS 先创建临时副本、应用补丁，再复制选定包到隔离工作区。官方子模块不被改写；主线 Gazebo/FAST-LIVO2 构建不应用此补丁。源码补丁不是无侵入适配，后续改为独立插件需要另行评估与行为验证。

验证：Linux 上应用补丁后 tree 为 `3ee9b5481cc32c24826afb82a3a84e5e934063d8`，与目标提交完全相同。必须使用 `git apply`，普通 patch 不完整保留空文件和权限。

7 个 AS2 包（core、msgs、behavior、motion reference、state estimator、motion controller、motion behaviors）编译通过；在该安装上重新编译公共接口和 FollowPath 扩展通过，CTest 1 项通过。PID 使用上游 v1.0（bc4743307db3f18affa232c04f068896cc69b528），本地依赖缓存与上游归档逐文件核对一致。日志在 `log/layout-conformance/as2-core-final.txt` 与 `as2-tree.txt`。这不代表完整 AirSim 镜像、飞行或 OHOS 真机验收。
