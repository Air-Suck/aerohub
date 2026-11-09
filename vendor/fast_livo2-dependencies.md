# FAST-LIVO2 构建依赖来源

依赖归档仅用于离线可重放构建；原始版权与许可文件保留在归档中。

| 文件 | 来源与锁定版本 | SHA256 |
| --- | --- | --- |
| `Sophus-79830a1.tar.gz` | `https://github.com/strasdat/Sophus`，`79830a101511fa5a52cf2c68be408b96926d7bcd`；`git archive --format=tar.gz --prefix=Sophus/` 生成 | `e248fa4543695e07cea30b283eec3968f03732bb0d36eedebbd414939b8a6330` |
| `rpg_vikit-6c886c8.tar.gz` | `https://github.com/xuankuzcr/rpg_vikit`，`6c886c8e5d83997806e00294826d528cea3581dd`；GitHub codeload tar.gz | `8162d0b616c443118a52e654394d3eb0807d6dac3b782f4dc3d6823c8815fede` |

FAST-LIVO2 README 指定非模板 Sophus，但其中短提交 `a621ff` 当前无法从官方仓库解析。本次从官方历史选取仍提供非模板 `SO3`／`SE3` 的上述提交，保留算法源文件，仅增加独立构建/安装配置及 C++11 complex 初始化兼容补丁。算法构建与冒烟验证结果见 `skill/temp/fast_livo2/README.md`。

Sophus 源码包含 MIT 许可；Vikit 的 package.xml 声明 GPLv3；FAST-LIVO2 上游声明 GPLv2。分别保留各组件许可，不将这些上游代码标记成本项目自有代码或改写其许可。
