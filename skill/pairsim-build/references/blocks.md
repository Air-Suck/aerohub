# 当前仓库 Blocks/HUD 操作参考

适用：本仓库 `simulator/projectairsim/course/` 中的 UE 5.2.1 / ProjectAirSim Blocks v7 打包实现。其他地图先检查脚本中的固定假设，不直接复用固定布局。下列变量由当前机器资源配置或本轮任务提供；在仓库根目录执行。

## 运行地图准备

`prepare_runtime_map.py` 另存运行地图，保留审阅地图，输出起点机身真实 UE/NED 位姿和 editor-only 对象清单。当前实现识别 `Course_Drone_*` 五个视觉 link，保留 `Course_Entrance` 启动相机，排除原建筑的过期反射探针；这是该实例的约定，不是任意工程的剔除规则。

```bash
export COURSE_REVIEW_DIR="$PWD/log/projectairsim/本次打包任务"
mkdir -p "$COURSE_REVIEW_DIR"
python3 - <<'PY'
import json, os
from pathlib import Path
cfg = {
    'package_map_only': True,
    'review_map': '/Game/BlocksCourseTwoStage',
    'runtime_map': '/Game/BlocksPlannerCourse',
}
Path(os.environ['COURSE_REVIEW_DIR'], 'runtime-map-config.json').write_text(json.dumps(cfg))
PY
export COURSE_CONFIG_JSON="$COURSE_REVIEW_DIR/runtime-map-config.json"
```

已有 `live_session.py` 会话时，可提交唯一 id 的本地 build 请求到其会话目录，使用上面两个路径，等待 `runtime-map.json`，不重启编辑器。`status.json` 的 submitted 仅表示已提交，实际结果另查证据和错误文件。会话请求协议见仓库 [course README](../../../simulator/projectairsim/course/README.md)。

没有编辑器会话、只需保存运行副本时：

```bash
"$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd" "$BLOCKS_PROJECT" \
  -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities \
  -run=pythonscript \
  -script="$PWD/simulator/projectairsim/course/prepare_runtime_map.py" \
  -unattended -nullrhi -nosound -abslog="$COURSE_REVIEW_DIR/runtime-map.log"
```

`-nullrhi` 仅适用于此处无渲染的地图准备，不用于实际运行验收或反射数据构建。`-ExecutePythonScript` 在本轮环境执行完成后曾退出编辑器；持续可视迭代使用控制台 `py <脚本路径>` 或 `-ExecCmds`，脚本完成时保留会话。

## 构建与资源

源码及入口：

- [package_course.py](../../../simulator/projectairsim/course/package_course.py)：比对已安装 HUD 的 Source/Resources，显式启用插件，运行 BuildCookRun，并生成启动脚本、原生场景和 manifest。
- [HUD 插件](../../../simulator/projectairsim/unreal/AerohubDisplay/AerohubDisplay.uplugin) 和 [HUD 构建入口](../../../simulator/projectairsim/build_display.sh)：仅在安装副本缺失或确有源码变化时构建/安装；核对已有用户修改。
- [运行包说明模板](../../../simulator/projectairsim/course/PACKAGE_README.md)：启动、原生机器人 scene、HUD HTTP 及速度转发说明。

```bash
python3 simulator/projectairsim/course/package_course.py \
  --ue-root "$UE_ROOT" --project "$BLOCKS_PROJECT" --source "$PROJECTAIRSIM_SOURCE" \
  --output "$COURSE_REVIEW_DIR/BlocksPlannerCourse-linux-v7" \
  --evidence "$COURSE_REVIEW_DIR"
```

该脚本固定 Linux Development、`/Game/BlocksPlannerCourse`、Blocks 二进制布局、v7 配置与两段各 60 米的 manifest。它包含本实例的空间预检和并发设置。换工程/版本时先参数化或使用工程自己的 UAT 入口，避免套错地图、配置或交付元数据。

HUD 字体通过 `RuntimeDependencies` 以 NonUFS 随包 Stage；构建插件 `.so` 本身并不能证明字体进入最终包。原生 scene 按字符串加载 `/Drone/*` 网格，需保留工程对该内容目录的 Cook 覆盖。

## 实际验证

`run-course.sh` 显式加载运行地图，读取包内 `config/display.json`；原生服务默认 8989/8990、HUD 默认 8991，测试可使用独立端口。当前启动脚本带 `-noraytracing`，原因和适用范围见 [排障参考](troubleshooting.md)。

原生机器人由包内 `load_scene.py` 单独加载，默认 PX4 示例；`--scene simpleflight` 是不连接 PX4 的静态验证方式。脚本不发解锁/飞行指令。

本实例已有 [validate_package.py](../../../simulator/projectairsim/course/validate_package.py)，需要匹配的 ProjectAirSim Python 客户端及已加载的 `ScenePlannerCourse/Drone1`：

```bash
"$PROJECTAIRSIM_PYTHON" simulator/projectairsim/course/validate_package.py \
  --output "$COURSE_REVIEW_DIR/final-validation" \
  --topic-port 9989 --service-port 9990 --display-port 8993
```

它核查 24 根柱、固定起点、单架机器人、无静态参照、真实静止速度及 HUD 计时状态机，**不适用于其他数量/起点的地图**。验证端口与实际启动参数保持一致；有额外授权再扩大到传感器、PX4/ROS 或 planner 飞行检查。

已验证基线：2026-11-13，UE 5.2.1，RTX 4090 / NVIDIA 580.178.04，Linux Development。实际窗口渲染 1280×720，原生静态机器人、HUD、24 根柱和计时接口通过；PX4、AS2、planner 与飞行未执行。该结论是历史实例，不自动适用于下一次构建。执行技能不依赖本轮被忽略的日志或已有压缩包。

## 原生传感器改动后的增量打包

2026-11-13 磁力计修正版已验证：在 ProjectAirSim 92032a6 应用仓库
`simulator/projectairsim/patches/0003-configurable-world-magnetic-field.patch`，
使用现有同 UE Clang/libc++ 的 Release 构建树，先增量构建 `core_sim_gtests`，
执行 `--gtest_filter=Magnetometer.*`（8项），确认 `libcore_sim.a` 已复制到插件
SimLibs 后，再运行普通 BuildCookRun。仅重 Cook 无法将静态库源码修改带入游戏。
保留实际补丁与源文件哈希、插件库和打包二进制哈希，核对原地图哈希未变。

`package_course.py --magnetic-reference <JSON>` 将 `field_ned_gauss` 三分量写入
包内原生 robot 的 `magnetic-field-ned-gauss`，来源信息写入 config 和 manifest。
robot 示例是 JSONC，需使用已有客户端环境的 commentjson 解析器。磁场向量单位Gauss，
世界NED，数值来源于目标位置/目标飞控的地磁参考；固定课程值不是全球WMM。
PX4 本轮保持原始镜像和默认参数。真实测量检查可用：

```bash
"$PROJECTAIRSIM_PYTHON" simulator/projectairsim/verify_magnetic_reference.py \
  --container "$PX4_CONTAINER" --topic-port "$TOPIC_PORT" --service-port "$SERVICE_PORT" \
  --reference "$MAGNETIC_REFERENCE" --output "$COURSE_REVIEW_DIR/ground-validation"
```

默认门限：至少20个姿态/磁场样本、静止真值、未解锁、航向误差≤1°、磁场均值每轴
误差<0.01G、原始 DECL_TYPE=3。此脚本限定原生单位姿态的课程起点，不把其他姿态
强行当作单位姿态比较。已验证最大航向误差0.258°；地面通过不代表空中地图或避障通过。
