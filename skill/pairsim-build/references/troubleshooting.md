# ProjectAirSim 打包排障

本页记录已实际遇到的判据和修正，不是每次打包都要执行的检查表。先按第一处有效错误定位阶段，再选对应分支。

## Stage 调用了不可执行的 env

**信号**：Build/Cook 已成功，StageBootstrapExecutable 调用 PATH 中的本地 `env chmod` 报 Permission denied。

核对 UAT 实际找到的 `env`。仅为本轮 UAT 设置有效系统工具 PATH，保留引擎工具和必要自定义工具的可达性，不改用户全局 shell 配置。保持平台、配置、地图和源输入不变，使用原命令的 `-skipbuild -skipcook` 继续 Stage/Archive；源码或资产已变则重跑相应阶段。

## 枚举 GPU 正常，vkCreateDevice 失败

**已验证环境**：UE 5.2.1 / NVIDIA 580.178.04 / RTX 4090。新包与旧 Blocks 包均在相同初始化点返回 `VK_ERROR_INITIALIZATION_FAILED`，`nvidia-smi` 正常，最小 Vulkan 设备的 1 和 16 队列创建均成功；仅加 `-noraytracing` 后成功实际渲染并启用 HUD。显式 `-vulkan` 或改离屏没有修复。

出现同一信号时，优先用已有旧包作同参数对照，再测试 `-noraytracing` 这一个变量。项目不使用硬件光追且对照成立时，可将该兼容参数写入启动脚本并记录驱动/UE 版本；原场景 SunSky 与屏幕空间 GI 不因此改为点光或合成渲染。参数并非所有 ProjectAirSim 版本/驱动的通用要求；需要光追的场景不得静默禁用其所需能力。

以实际地图加载、HUD HTTP 或原生 RPC 返回验证修正，不能用 GPU 枚举或空进程作为成功。没有改善时保留日志，依据新的设备/驱动证据继续定位，避免反复全量打包。驱动升级或引擎重编译不属于默认修正。

## Reflection captures need to be rebuilt

**信号**：实际游戏窗口仍有反射探针未构建提示，尽管 Cook 成功。

确认探针是否覆盖新几何。适用的探针应构建其反射数据；复制自原建筑且已不适用的探针，可在明确采用实时天空环境反射的运行副本中设为 editor-only。重 Cook 并复拍实际窗口核查，不用 `DisableAllScreenMessages` 掩盖。

UE 5.2.1 的 `ResavePackages -buildreflectioncaptures` 曾返回成功并重存地图，但不等于真的完成反射捕获；检查锁定版本命令实现及构建数据，不把命令退出码当作渲染结论。单独构建反射不应默认扩大为全量静态光照烘焙。

## HUD 编译过，但包里缺 HUD 或字体

区分插件 Source/安装副本是否一致、uproject 是否实际启用、目标 Game 是否链接模块、字体/许可是否以 RuntimeDependencies Stage。Editor 或 standalone-editor HUD 可用不证明 packaged Game 可用。用打包程序的 `/aerohub/status`、字体加载日志和实际窗口验证。

速度为 `--` 时，先区分未加载机器人、未运行真实 telemetry 转发、数据超过 HUD 时效、端口不一致；不要用合成速度把显示“修好”。计时接口只改变观察状态，不证明飞控或 planner 就绪。

## 窗口截图取证

优先使用能证明实际渲染尺寸的游戏截图或准确游戏客户区捕获，避免把编辑器窗口裁切图当作独立运行证据。X11 环境可用 `xwininfo` 定位本任务游戏窗口、`xwd -id` 捕获；只保存目标窗口。

若 Pillow 不识别 XWD，解析文件实际 header、颜色表、字节序、位深和行跨度后再转 PNG；本轮是 32-bit little-endian BGRX，不能当作所有显示服务的固定格式。转换不缩放，核对原始客户区与 PNG 都是要求的尺寸。

## UAT归档成功，随包配置生成失败

JSONC robot 示例含注释，标准 `json.loads` 会失败；使用匹配客户端环境已有的
commentjson，不更改上游格式以迁就解析器。`package_course.py --finalize-only` 仅在
本次 UAT 日志已明确 BUILD SUCCESSFUL 且归档二进制存在时补齐配置/manifest，
保留原始失败记录，不重复 Build/Cook。此时不再套用构建暂存空间阈值；仍检查实际
所需空间。若游戏/插件源码发生变化，则不能用 finalize-only 代替重编译。
