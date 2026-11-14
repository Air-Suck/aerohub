# Blocks Planner Course + HUD

Linux x86_64 / UE 5.2.1 Development，包含已审阅的 v7 走廊和 AerohubDisplay。
两段各 60 米、净宽 12 米、墙高 6 米；第一段直圆柱，第二段左右交替 15° 斜圆柱。
绿线为起点，原红线为 60 米分界，末端红线为 120 米终点，两端各 8 米缓冲。
自然日光＋天空环境光。静态无人机尺寸参照仅编辑器可见，不进入运行场景。

## 启动

需要 Linux 桌面及支持 Vulkan 的显卡驱动；不需要安装 UE 编辑器。

```bash
./run-course.sh
```

默认窗口 1280×720，启动地图 `/Game/BlocksPlannerCourse`，启动机位为绿线入口。
启动脚本禁用未使用的实验性 Vulkan 光追扩展，避免 UE 5.2 在部分新驱动上创建设备失败；
本场景的日光、天空环境光与原有屏幕空间全局光照仍保留。
后续 UE 参数可追加；例如 `-topicsport=9989 -servicesport=9990` 可切换原生端口。
`config/display.json` 控制 HUD；也可设置环境变量 `AEROHUB_DISPLAY_CONFIG` 指向自定义 JSON。

## 接入机器人与 planner

UE 地图和原生机器人 scene 分开加载。地图包不会自动启动 PX4、ROS、planner 或飞行。
使用与你的 ProjectAirSim 服务版本匹配的 Python 客户端环境：

```bash
python load_scene.py --scene px4
```

PX4 配置采用上游 PX4 SITL 示例，保留传感器与控制器设置。
MAVLink 默认 TCP 4560、控制 UDP 14540，具体见 `sim_config/robot_quadrotor_px4_sitl.jsonc`。
起点载具为 `Drone1`，位置见 `config/runtime-map.json` 的 `robot_spawn_native_ned_m`。
注意原生坐标为 NED，地图坐标为 UE 的 Z 向上，不能直接把地图 Z 当作 NED Z。
planner 仍通过你的 ROS adapter、Behavior、AS2/PX4 执行链控制机器人。
本包提供仿真世界与 HUD，不包含 planner、ROS bridge、AS2 或 PX4 可执行程序。

仅查看机器人与 HUD、不连接 PX4时，可运行 `python load_scene.py --scene simpleflight`。
两种模式都只加载场景，不调用解锁、起飞或运动指令。

HUD 速度需要原生 ground-truth telemetry 转发；使用同一个客户端环境另开终端：

```bash
python display_telemetry.py --scene sim_config/scene_course_px4.jsonc --vehicle Drone1
```

SimpleFlight 模式替换 scene 文件名。未加载机器人或未启动 telemetry 时速度显示 `--`，
不得将 HUD 就绪或机器人可见当作 PX4/AS2/planner 已就绪。

## HUD 接口

默认只在本机 `127.0.0.1:8991` 提供接口：

```bash
curl http://127.0.0.1:8991/aerohub/status
curl -X POST http://127.0.0.1:8991/aerohub/mission/start
curl -X POST http://127.0.0.1:8991/aerohub/mission/finish
curl -X POST http://127.0.0.1:8991/aerohub/mission/reset
```

这些接口仅控制观察计时器，不启动 planner 或飞行。计时为单调墙钟时间。
HUD 包含任务状态、计时、速度表和峰值；字体及其许可随包提供。
验收结果见 `manifest.json`；场景打包与静态运行检查不代表完成 planner 避障或飞行验收。

## 带磁场参考的修正版

当包内存在 `config/magnetic-reference.json` 时，原生 robot 配置已设置
`magnetic-field-ned-gauss`：三个分量依次为世界北、东、下方向，单位 Gauss。
该可配置磁场在 ProjectAirSim 内按真实载具/传感器姿态转换并添加原有噪声。
本课程的参考来自原版 PX4 对该地理位置的 WMM 输出；它是适用于固定课程的
局部恒定磁场，不是完整的全球 WMM 实现。迁移地理位置或飞控地磁模型时需重新核对。

须使用包内 `sim_config/` 的 robot/scene 配置；外部旧 robot 配置没有该字段，
仍使用默认偶极磁场。PX4/ROS/AS2 作为外部依赖提供，不要求修改 PX4 参数或镜像。
地面姿态、磁场和静态运行结果见 `validation/` 与 `manifest.json`；空中地图注册和
避障仍需另行验证。
