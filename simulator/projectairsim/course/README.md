# Blocks 赛道生成与打包

当前交付为 120 m 双阶段柱体赛道 `/Game/BlocksPlannerCourse`，配置见
`config/simulator/blocks_course_two_stage.yaml`。生成、打包与校验脚本在本目录；
飞行入口及验收状态见 [赛道运行说明](../../../deployment/airsim/demos/projectairsim_course/README.md)。
原 `/Game/BlocksMap` 不覆盖。下文保留空场、直柱到双阶段场景的版本依据，
各节的验证结论仅适用于注明的版本，不代表最新 Planner 飞行通过。

## 重建审查稿

需要 UE 5.2 编辑器及其 PythonScriptPlugin、EditorScriptingUtilities。
通过命令行临时启用插件，无需修改 uproject。设置本地路径后运行：

```bash
export COURSE_REVIEW_DIR="$PWD/log/projectairsim/course-review"
"$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd" "$BLOCKS_PROJECT" \
  -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities \
  -run=pythonscript -script="$PWD/simulator/projectairsim/course/inspect_blocks.py" \
  -unattended -nullrhi -nosound -abslog="$COURSE_REVIEW_DIR/inspect.log"
python3 simulator/projectairsim/course/review_plan.py \
  --inventory "$COURSE_REVIEW_DIR/blocks-inventory.json" \
  --config config/simulator/blocks_course.yaml --output "$COURSE_REVIEW_DIR"
```

平面稿依赖 PyYAML、Pillow。检查脚本只加载原地图、导出世界坐标 AABB、网格和材质引用，
不调用地图保存。编辑器仍会写工程缓存与日志，因此运行前需具备外部工程写权限。

## v2 定位依据与待确认项

2026-11-12 已使用 UE 5.2.1 执行检查，导出 169 个含静态网格组件的 Actor。
立方体建筑包络为 X `[-40.1,120]`、Y `[-122.5,117.5]`、Z `[-2.5,26]` 米。
原地面上表面 Z=1 米；原灰色材质、立方体和地板引用记录于导出清单。

提议将赛道放在建筑 +X 侧：绿线中心 `(151.5,0,1.1)` 米，朝 +X，
红线中心 `(211.5,0,1.1)` 米。含缓冲和墙厚的平面范围为
X `[143.5,219.5]`、Y `[-6.5,6.5]` 米，与建筑最大 X 边界相隔 23.5 米。
非地面、非 SunSky 的静态网格 XY 包络与候选区域无重叠。
用户已确认 +X 侧，并要求绿线朝向原建筑。v2 将长轴由 +Y 转为 +X，
绿线入口朝 -X，保持入口缓冲外缘距原建筑 23.5 米，避免原地绕中心旋转造成重叠。

尺寸解释为起终线间 60 米，两端线外各 8 米，墙和地板总长 76 米。
旋转后的摆放见 v2 审查图。地图定位图展示实际 Actor 包络的投影，
不是 UE 渲染俯视截图；二维图片尺寸均为 1280×720。

产物统一在 `log/projectairsim/course-review/`：

- `blocks-inventory.json`、`inspect.log`：实际地图读取证据。
- `blocks-location-v2.png`：实际建筑包络、候选区域和 UE 坐标方向。
- `course-plan-v2.png`：尺寸、缓冲区、绿线和红线。
- `placement-review.json`：候选位置包络和静态网格重叠检查。

## 生成与真实渲染截图

生成器只管理带 `AerohubBlocksCourseV2` 标签的对象，放入 `AerohubCourse/Empty` 文件夹。
加载已有赛道地图后保留未标记对象；若目标地图无所有权标签则拒绝覆盖。
尺寸按实际网格包络及 pivot 换算，单位为米和角度；UE 内部转为厘米。
原网格无简单碰撞时，仅复制到 `/Game/AerohubCourse/SM_<类型>` 并添加盒形碰撞，
不修改原资产。另建 `/Game/AerohubCourse/M_Red` 作为真实红色标线材质。
标线厚 1 厘米，下表面贴地、上表面高于地面，不共面。
已支持 empty 和 sparse_straight；其他布局仍拒绝生成。

几何尺寸、重复生成、保存重开、地板及两侧墙射线阻挡检查已通过，详见证据 JSON。
俯视、入口和侧面三个固定机位的实际渲染截图用于人工视觉审查；
二维平面稿不能代替该验收。空场通过也不代表避障或飞行通过。


生成前将 YAML 转为 UE 内置 Python 可读取的 JSON；系统 Python 需要 PyYAML：

```bash
python3 - <<'PYCONFIG'
import json, os, yaml
from pathlib import Path
root = Path(os.environ['COURSE_REVIEW_DIR'])
root.mkdir(parents=True, exist_ok=True)
root.joinpath('approved-config.json').write_text(json.dumps(
    yaml.safe_load(Path('config/simulator/blocks_course.yaml').read_text()), indent=2))
PYCONFIG
export COURSE_CONFIG_JSON="$COURSE_REVIEW_DIR/approved-config.json"
"$UE_ROOT/Engine/Binaries/Linux/UnrealEditor" "$BLOCKS_PROJECT" \
  -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities \
  "-ExecCmds=py $PWD/simulator/projectairsim/course/build_and_capture.py" \
  -unattended -nosound -windowed -ResX=1280 -ResY=720 \
  -abslog="$COURSE_REVIEW_DIR/build-and-capture.log"
```

需在有显示权限的本地桌面运行，保留本地 DISPLAY/XAUTHORITY 配置，不写入共享配置。
脚本显式加载 `StaticMeshEditor` 模块，否则 UE 5.2 启动时可能尚未暴露对应 Python 类型。
生成后重复生成一次、保存、重开，对地板和两侧墙执行阻挡射线检查；
几何证据保存在 `geometry-review.json`。原地图用 SHA256 检查未变。
截图使用三个保存的 CameraActor，编辑器不启动 Play；等待加载后逐机位请求真实
1280×720 截图，结果写入 `capture-review.json`。完成或失败后均保留编辑器，不自动退出。后续直接在同一会话的控制台运行 `py <脚本路径>`。
截图输出为 `course_overview-v2.png`、`course_entrance-v2.png`、`course_side-v2.png`。

碰撞射线检查不替代动力学闭环；静态截图无法单独证明运动中无闪烁。
最终风格、比例由用户审查，尚未进行柱林避障或飞行验收。

## v2 空场稿交付状态（2026-11-12）

实测起终线间 60 米，净宽 12 米，墙高 6 米，墙厚 0.5 米，
两端缓冲各 8 米，地板与连续侧墙总长 76 米。
重复生成两次共保留 5 个几何对象和 3 个审查相机，未修改未管理 Actor 集合。
保存重开检查通过，地板与两侧墙阻挡射线检查通过，原地图 SHA256 未变。

三个真实渲染机位为俯视全景、绿线入口前视、侧面全景；
俯视相机已改为横向构图，完整展示两端缓冲。
保留原场景光照与灰色材质，当前墙面阴影较深，风格与比例仍待人工视觉审查。
静态图中可见绿红标线，未进行运动中的闪烁验证。
没有创建柱体，没有启动 Play、飞行或改动传感器、飞控及 HUD 接口。

## v3 稀疏直柱（已确认并生成）

用户已接受 v2 空场并要求添加障碍，下轮 UE 截图在起点放置无人机作尺寸参照。
用户已确认障碍平面稿。配置保存在 `config/simulator/blocks_course_sparse.yaml`，
另存为 `/Game/BlocksCourseSparse`，已保存的 v2 空场地图保留。

首版使用 12 根 1×1×6 米灰色方直柱，6 排，每排两根交错，排距 8 米。
同排柱面净间距 3 米，最小柱墙间距 2.5 米，首末柱面分别距起终线 9.5 米。
坐标显式固定，seed=0，无随机采样；两端缓冲区不放障碍。
这些是几何净距，尚未计入无人机包络或飞行安全余量，不代表避障验证通过。

```bash
python3 simulator/projectairsim/course/review_obstacles.py \
  --config config/simulator/blocks_course_sparse.yaml \
  --output log/projectairsim/course-review
```

输出 `obstacle-plan-v3.png` 和 `obstacle-plan-v3.json`，已检查柱体不重叠、
不穿墙且位于起终线内。图中蓝点仅标记无人机位置，不表示已测量的机身包络。
后续放置无人机前核对实际 robot link 与网格组成，保持原始尺度，按实际下缘贴地，
位于绿线中央、机头朝红线；使用静态视觉参照，不启动飞行。
除原有三个机位，追加入口近景，避免全景中机体过小而无法比较尺寸。


### v3 重建与检查

先准备合并配置和无人机视觉 link，运行时路径由本地环境变量提供：

```bash
export COURSE_REVIEW_DIR="$PWD/log/projectairsim/course-review/sparse-v3"
python3 simulator/projectairsim/course/prepare_course.py \
  --config config/simulator/blocks_course_sparse.yaml \
  --robot-config "$PROJECTAIRSIM_SOURCE/client/python/example_user_scripts/sim_config/robot_quadrotor_px4_sitl.jsonc" \
  --output "$COURSE_REVIEW_DIR/approved-config.json"
export COURSE_CONFIG_JSON="$COURSE_REVIEW_DIR/approved-config.json"
"$UE_ROOT/Engine/Binaries/Linux/UnrealEditor" "$BLOCKS_PROJECT" \
  -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities \
  "-ExecCmds=py $PWD/simulator/projectairsim/course/build_and_capture.py" \
  -unattended -nosound -windowed -ResX=1280 -ResY=720 \
  -abslog="$COURSE_REVIEW_DIR/build-and-capture.log"
```

prepare 支持锁定 PX4 示例中的整行 JSONC 注释；其他未支持的 JSONC 形式报错，
不尝试破坏性删除字符串内注释符号。当前视觉组装仅支持此示例的五个零旋转 link。
转换与上游 `UnrealRobotLink::InitializePose`、`UnrealHelpers::ToFVectorTranslation`
一致：NED 米变为 UE `(x,y,-z)` 厘米，网格保持尺度 1，不按惯性包络缩放外观。
机身及四个桨叶整体按最低可见顶点包络落在绿线上表面，机头朝 +X。
这是无碰撞、无控制器的静态网格参照，不是已部署/可飞行的机器人。

2026-11-12 运行证据：26 个受管理对象（5 个基础几何、12 根柱、5 个无人机网格、
4 个审查相机）；重复生成、保存重开、尺寸检查通过。
15 项阻挡射线覆盖地板、两侧墙及每根柱，全部命中预期赛道对象。
无人机实际网格组合包络约为 0.982×0.982×0.293 米，所有 link scale=1。
证据目录为 `log/projectairsim/course-review/sparse-v3/`，图片后缀为 `-v3.png`，
第四张入口近景为 `course_dronedetail-v3.png`。四张图均须为实际渲染的 1280×720。
几何/碰撞检查通过不代表动力学、避障或飞行验收；风格与布局视觉效果仍待用户审查。


## v4 圆柱与走廊补光（2026-11-12）

按用户反馈将 12 根方柱替换为直径 1 米、高 6 米的圆柱，柱位与起点无人机不变。
当前 `blocks_course_sparse.yaml` 对应 v4，仍保存为 `/Game/BlocksCourseSparse`；
v3 的四张截图和检查结果保留在 `sparse-v3/`，可作同机位对比。

圆柱引用 `/Engine/BasicShapes/Cylinder`，复制到
`/Game/AerohubCourse/SM_cylinder`，仅修改副本碰撞为 Complex As Simple。
这是静态场景障碍，三角网格碰撞与可见柱面一致，不能作为动态刚体使用。
原引擎网格不修改。除逐柱横向命中检查外，新增逐柱包围盒角部垂直射线检查：
距轴线 X、Y 各偏移 0.45 米，射线应穿过柱外空间命中地板，验证未残留方盒碰撞。

补光为 14 盏无阴影的可移动点光，纵向位置 `[-4,8,20,32,44,56,68]` 米，
横向 `±4.5` 米、高 5.5 米。每盏 600 流明，衰减半径 16 米，光源半径 1 米，
暖白色 `(255,248,238)`。没有添加可见灯具或顶部几何，走廊仍敞顶。
补光对象带相同管理标签，可重复重建；原场景灯光及相机位置保持原样。
这些数值是 UE 渲染参数，不表示已测量的真实照度。

复用 v3 重建命令，仅将输出目录改为
`log/projectairsim/course-review/sparse-v4/`，重新执行 prepare 后生成与截图。
平面稿脚本自动按 `pillar_shape` 绘制圆形。四张真实渲染图片后缀为 `-v4.png`。
检查证据增加 `radial_collision_checks`、`pillar_shape` 和 `lighting` 字段。
几何/碰撞、静态视觉审查与飞行验收分别报告；未启动 Play 或飞行。


### 光照与编辑器工作流修正

用户指出 v4 密集点光补光形成不自然的光斑与泛白，此方案未通过视觉审查。
60000 流明的首轮过曝；随后降低至 600 流明的结果也不能视为自然光照已验收。
暂停继续调光，后续从原场景日光、天空环境光及曝光关系检查入手，
避免用密集局部点光替代室外环境光。
截图脚本与构建入口已移除自动退出编辑器逻辑；使用同一编辑器会话修改和截图，
保留现场供用户观察。调用时使用控制台 `py <脚本路径>` 或 `-ExecCmds`，
避免使用执行结束可能退出编辑器的 `-ExecutePythonScript` 入口。


## v5 自然环境光与持续编辑会话

移除全部 14 盏 `Course_Fill_*` 点光；当前采用原 `SunSky` 的日光与天空环境光，
日光强度保持 2.75、方向不变，天空光强度从原 1 调为 12（UE 艺术参数，非实测照度）。
相机与曝光参数未改。均匀提亮来自环境天空光，不添加局部灯具和顶棚。
只在赛道地图内更新 SunSky 组件；原 BlocksMap 和空场地图不保存。
圆柱、柱位、绿红标线及原尺寸无人机沿用前版。

首次打开持续会话使用 `live_session.py`，环境变量 `COURSE_SESSION_DIR`
指向被忽略的 `log/projectairsim/course-review/live-session/`。
后续本地 `request.json` 请求由编辑器主线程 tick 消费，避免重复启动编辑器：

```json
{
  "id": "unique-request-id",
  "action": "build",
  "output": "<local-output-directory>",
  "config": "<prepared-json-file>"
}
```

`action` 支持 `inspect`、`build`、`capture`；本地路径只在运行产物内提供。
每个请求使用不同 id；前一轮 `capture-review.json` 更新完成后再发送下一轮，
不得在截图仍使用相机时重建地图。`status.json` 的 submitted 仅代表已提交截图任务，
不是截图成功；成功还需核对新一轮图片时间、尺寸及错误文件。
也可在当前编辑器控制台直接运行 `py <脚本路径>`。

纯光照迭代：在 prepare 输出 JSON 中增加 `"update_mode":"lighting_only"`，
通过 build 入口执行 `update_lighting.py` 再截图。此模式不重建几何，
逐组件比较网格引用与数值变换，保存地图后输出 `lighting-review.json`。
光照检查使用 `lighting_probe.py`，输出实际组件设置，并把编辑器视口切回入口相机。
截图成功或失败均不会退出编辑器，保留现场供用户直接观察。

v5 证据在 `log/projectairsim/course-review/sparse-v5/`，图片后缀 `-v5.png`。
`geometry-review.json` 验证圆柱与碰撞，`lighting-review.json` 记录随后同会话的调光，
最终光照以该文件与 prepared config 为准。所有渲染采用相同四机位与 1280×720。
静态亮度统计只用于检测过曝与对比，不代替用户对自然光效果的视觉审查。


## 第一段确认与第二段连接约束

用户已接受 v5 圆柱及自然光效果，将其作为走廊第一段。
第一段起点绿线位于 UE `(151.5,0,1.1)` 米，终点红线位于 `(211.5,0,1.1)` 米。
后续第二段从此红线接出，线间长度同样为 60 米，末端增加红线；
现有红线成为两段共用分界标线，不重复叠放两条线。
连接区域目前仍有第一段线外 8 米末端缓冲，接段时必须复用或调整该区域的
地板和侧墙，避免共面地板、重复墙体或段间缝隙。
用户随后确认第二段为约 15° 交错斜柱，走廊继续直行；具体实现与验证见 v6 节。
保留第一段柱位、照明及起点无人机参照；继续使用当前编辑器会话。


## v6 两段走廊：直柱 → 15° 斜柱

用户确认第二段“斜”指柱子倾斜，同意约 15° 交错倾斜，走廊沿 +X 直行。
配置 `config/simulator/blocks_course_two_stage.yaml`，地图另存为
`/Game/BlocksCourseTwoStage`，保留已接受的单段 `/Game/BlocksCourseSparse`。

标线的 UE 世界坐标（米）为：绿线 `(151.5,0,1.1)`，
共用分界红线 `(211.5,0,1.1)`，第二段终点红线 `(271.5,0,1.1)`。
两段分别 60 米，总线间长 120 米，净宽仍为 12 米。
仅整体最外两端保留各 8 米缓冲；原第一段末端缓冲区域纳入第二段。
总地板及侧墙长 136 米，使用一块地板和两面连续墙，无段间共面重复。

第一段原有 12 根直圆柱与起点无人机的包络和位置均与 v5 对比一致。
第二段复制相同 12 个底部柱位，纵向加 60 米，沿前进方向交替前/后倾 15°，
不向两侧墙倾斜。柱径 1 米、轴长 6 米，底面中心锚定地面：斜底盖一侧
约 0.129 米埋入地面，避免悬空；柱顶最高约在地面以上 5.925 米。
“6 米”在斜柱中指轴长，不宣称其垂直包络仍为 6 米。

生成器使用具名 `Rotator(pitch=...,yaw=...,roll=...)` 设置斜柱，
并以实际 `up_axis` 验证倾角与方向，避免 UE Python 位置参数顺序误用。
验证实际旋转后的世界包络、底部锚定、柱间包络不重叠、柱墙不相交，
并覆盖 24 根柱的横向阻挡及沿各自局部轴的柱外角部射线。
43 个管理对象（6 个基础几何与标线、24 根柱、5 个无人机网格、8 个审查相机）。
重复生成与保存重开通过；27 项阻挡与 24 项柱外角部射线检查通过。

仍在同一个 UE 编辑器会话中构建、保存和截图，不退出、不启动 Play 或飞行。
prepare 命令改用上述 two_stage YAML，输出目录为
`log/projectairsim/course-review/two-stage-v6/`。其余同 v3/v5 的工作流。
图片为全段俯视、第二段俯视、第二段入口、连接处侧视和起点无人机近景，
统一实际渲染 1280×720，后缀 `-v6.png`。自然光沿用已接受的 v5 参数。
视觉效果仍需用户审查，场景构建与碰撞验证不代表完成飞行或避障验收。


## v7 修正：第二段左右倾斜

用户纠正第二段应左右倾斜，v6 的前后倾斜版本已被取代。
当前 two_stage 配置的 `pillar_tilt_direction: lateral` 表示跨走廊宽度倾斜，
第二段轴方向为 `(0,±sin15°,cos15°)`，不再沿前进方向倾斜。
仍为每排两根、共六排；同一排柱子向同一侧倾斜，相邻排左右交替。
这样保留全部底部柱位，同时避免两根向内对倾时顶端相交。

使用具名 roll 设置横向倾角；验证实际轴、旋转后的世界包络、底部锚定、
柱体互不重叠及不穿墙。按包围盒保守计算，第二段最小柱墙净距约 2.517 米。
第一段、起点无人机、连接处标线、整体地板/墙体和自然光沿用前版。
光照、几何与碰撞检查不代表飞行避障验收通过。

地图仍为 `/Game/BlocksCourseTwoStage`，在同一编辑器会话中更新，
证据改为 `log/projectairsim/course-review/two-stage-v7/`，截图后缀 `-v7.png`。
同机位复拍总俯视、第二段俯视、第二段入口、连接侧视及无人机近景，
均为实际渲染 1280×720；编辑器保持开启。


## v7 Linux 场景与 HUD 打包

用户已审阅并要求打包。正式运行地图为 `/Game/BlocksPlannerCourse`，
由已接受的 `/Game/BlocksCourseTwoStage` 另存。五个静态无人机视觉参照、
多余审查相机及三枚原建筑遗留反射探针标记为仅编辑器对象，不进入运行包。
运行时使用实时 SkyLight 环境反射；不是通过关闭屏幕提示隐藏反射探针问题。
原始地图与审阅地图 SHA256 保持不变。

`prepare_runtime_map.py` 输出起点机身真实 UE/NED 位姿和剔除对象记录。
`package_course.py` 显式启用已安装且与仓库一致的 AerohubDisplay 插件，
使用 Linux Development BuildCookRun，烘焙运行地图并随包分发 HUD 字体。
UAT 的 PATH 使用系统工具目录，避免调用机器本地不可执行的 env 包装器。
构建产物、失败尝试与运行日志均保存在 `log/projectairsim/package-course-v7/`。

```bash
python3 simulator/projectairsim/course/package_course.py \
  --ue-root "$UE_ROOT" --project "$BLOCKS_PROJECT" --source "$PROJECTAIRSIM_SOURCE" \
  --output "$PWD/log/projectairsim/package-course-v7/BlocksPlannerCourse-linux-v7" \
  --evidence "$PWD/log/projectairsim/package-course-v7"
```

先用带 `package_map_only`、`review_map`、`runtime_map` 的配置执行运行地图准备，
再打包；本机路径仅在忽略的运行配置里提供。运行包说明模板为 `PACKAGE_README.md`。
启动脚本加入 `-noraytracing`：UE 5.2 在当前 NVIDIA 580.178.04 驱动下启用
未使用的实验性 Vulkan 光追扩展会令 vkCreateDevice 失败；新旧 Blocks 均可复现。
最小 Vulkan 创建设备及 1/16 队列均成功，关闭光追扩展后真实渲染和 HUD 正常。
未升级驱动或修改共享 UE 工具链；本场景不使用硬件光追。

`validate_package.py` 对真实打包程序的原生 RPC 与 HUD HTTP 验证 24 根柱、
起点单架机器人、无静态参照、静止 ground-truth 速度转发及计时状态机。
SimpleFlight 用于无 PX4 的静态加载检查，不发解锁、起飞或运动指令。
PX4、ROS bridge、AS2、planner 不包含在场景包中，也不能据此声称其闭环已验收。


本次交付压缩包为 `log/projectairsim/package-course-v7/BlocksPlannerCourse-linux-v7.tar.gz`，
同名 `.sha256` 校验压缩包，包内 `SHA256SUMS` 校验各文件。
最终实际渲染截图为 `packaged-final.png`，无过期反射探针屏幕提示；
HUD 和原生 RPC 静态运行检查见 `final-validation/`，复制入包内 `validation/`。
启动与原生 scene 加载分开提供，不把观测计时器接口当作 planner 控制接口。
