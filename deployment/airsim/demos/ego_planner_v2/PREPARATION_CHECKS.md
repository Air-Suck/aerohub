# 整理后检查（2026-11-13）

基线为本地 upstream/dev `4c06d210b08df47059dbe6e3073f0d5db35f3196`。远端刷新两次因TLS/EOF
失败，不能据此声称已核对最新远端；提交前需重新fetch并检查差异。

| 检查 | 结果 |
| --- | --- |
| 全新外部工作区，固定onboard镜像、network none | 原生6包＋公共接口/执行2包＋framework/Adapter2包编译通过 |
| 实际消息/Adapter加载 | EGO包前缀、Adapter模块前缀和FollowTimedPath序列化通过 |
| 原有FollowTimedPath timeline测试 | 1/1通过 |
| 数值与归属单测 | 末端保持8、曲线转换5、每机归属7，20/20通过 |
| 真实Adapter/AS2隔离回归 | 6/6：正常/迟到尾段回放、每机完成/取消/点云中断、兼容入口取消 |
| 子进程正常退出 | 六轮全部退出0，无强杀；本结果不证明长期退出可靠性 |
| 原有部署布局与新增EGO候选 | 5/5＋3/3通过 |
| 单机入口离线契约 | 19/19通过 |
| 正式Skill | 六个quick_validate通过，已核对内容归属与引用 |
| 静态检查 | Python解析、Bash/PowerShell解析、YAML/Compose和源码/证据哈希检查通过 |

六轮真实AS2参考相对原生同时间PVA最大偏差 `1.6564587e-06`，原门槛1e-4保持。
两轮回放只改变可追溯的基准epoch；迟到反例预期拒绝并停止。合成控制器/输入与真实
AS2执行组件的职责保持明确，未连接UE、PX4或RoboPi。结果摘要见
[preparation-ros-results.json](evidence/preparation-ros-results.json)。

原生43个文件与历史OHOS构建输入哈希一致；Adapter来自STEP23快照。
机械变化包括Demo路径、外部构建目录、可维护fixture与新候选前缀；新增构建/部署选择
有独立离线回归。未把这次整理描述为新前缀板端飞行、重新交叉编译或完整PC动力学复验。

检查命令见 [README](README.md#不启动飞行的复验)。本机完整构建/检查日志存于
`log/ego_planner_v2/pr1-preparation/`，不纳入Git。原先失败的检查尝试保留；修正了
默认示例切换后仍假定旧模式的测试，以及静态检查器误匹配自身字符串的问题。
