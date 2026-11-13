---
name: oh-ros-build
description: 为 OpenHarmony 交叉编译 ROS 2 包时使用，涵盖构建条件检查、目标依赖准备、增量编译、故障排查和产物验证。
---

# OH ROS Build

先检查与本次构建相关的条件，再开始编译；复用仍有效的检查结果。

- 从现有配置定位构建镜像、SDK、目标依赖和工作区，确认路径可用；不硬编码本机路径或擅自替换版本。
- 确认目标架构、OpenHarmony SDK、ROS 发行版及目标依赖匹配；涉及 Python 扩展时核对目标 Python ABI。已有可信快照时可离线构建，无需每次连接开发板。
- 确认构建输入来自当前源码，记录提交和本地修改；检查临时工作区是否仍是旧副本，以及补丁是否已进入 fork，避免漏同步或重复应用。
- 区分主机代码生成工具与目标头文件、库和 Python 扩展；检查 CMake 查找路径及依赖中的绝对路径，防止误选 PC 库。
- 构建工具会修改 SDK 或依赖时使用隔离副本，共享原件保持不变；核对安装前缀与预期部署路径，处理已知重定位问题。
- 条件缺失时先在已有授权范围内定位和补齐；确实缺少必要信息时，一次说明具体缺项及最少需要的输入。
- 条件满足后复用已有构建入口，按依赖关系增量编译所需包并保留日志。失败时定位首次有效错误，有依据再清理对应缓存；可复用修复写回源码或构建配置。
- 使用目标工具链检查产物架构、动态依赖及适用的 ABI，不在 PC 上直接执行目标程序。分别报告编译、静态检查和板端运行验证结果。

当前仓库的构建经验按需查阅 [OHOS 构建记录](../../deployment/robopi/README.md)；需要板端交互时使用 [hdc-helper](../hdc-helper/SKILL.md)。

## 工作区版本与补丁冲突

主仓库 gitlink 与本地 AS2/Pixhawk checkout 不同时，先记录两者及 dirty 状态。当前构建契约使用 gitlink 加兼容补丁：从子仓 `git archive <gitlink>` 导出隔离副本再应用补丁，保留用户 checkout；不要向已经包含补丁的 checkout 再应用同一补丁。`tools/robopi/prepare_ohos_sources.py` 按此导出并在 current-sources.json 记录锁定提交和实际同步源码哈希。导出成功不证明目标编译或板端加载成功。

## SUPER 的板端构建

用户明确要求 SUPER 在 RoboPi 执行时，复用 `bash tools/robopi/build_lib.sh super`，
从主仓库 native/bridge 同步三个原生包、Adapter、框架和导航输入；不使用参考 upstream。
官方 builder 必须显式传入 `ROS_DISTRO=humble`。本组合已使用现有目标 PCL 1.15 编译，
不需要先重建 PCL。若 PCL IO 出现 `png_*@PNG16_0` 未解析，检查目标 libpng 的动态
符号版本：曾有默认 16.43 无版本符号、已有 16.50 提供所需版本的情况。用
`SUPER_PNG_LIBRARY` 显式选择匹配目标库并在候选局部部署，保留共享 SDK/系统库；
不要使用忽略未解析符号的链接选项。此经验只适用于该依赖快照，不能替代其他版本 ABI 核对。

## 板端导出的目标依赖

复用匹配本板的ROS/第三方依赖时先核对API、架构、Python SOABI、必要头文件和库，
通过归档保留符号链接。导出的CMake/pkg-config元数据可能含构建机绝对路径，重定位只
作用于准备后的隔离副本，不能修改板端系统或原始导出。ROSIDL选择主机Python生成代码时
仍须核对目标Python/NumPy头文件；PC类型支持不能复制到OHOS。EGO实例的元数据处理入口
为 `tools/robopi/relocate_ego_target_metadata.py`，它只接受已准备工作区的root/install。
