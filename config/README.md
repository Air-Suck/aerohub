# 用户配置入口

仿真 YAML 统一放在 `config/simulator/`，默认读取 `default.yaml`。
默认配置和普通场景 YAML 可以纳入 Git；只有 `*.local.yaml` 本机配置被忽略。
其他场景配置需显式选择，不自动覆盖默认项。

```text
config/simulator/
├── default.yaml         默认配置，纳入 Git
├── blocks.local.yaml    本机场景配置，不入 Git
└── forest.yaml          共享场景配置，可纳入 Git
```

```bash
# 使用默认配置（先设置其中的安装路径）
bash simulator/simulator.sh start

# 或复制为独立场景，修改后显式选择
cp config/simulator/default.yaml config/simulator/forest.yaml
bash simulator/simulator.sh check --config config/simulator/forest.yaml
bash simulator/simulator.sh start --config config/simulator/forest.yaml
bash simulator/simulator.sh status
bash simulator/simulator.sh stop
```

也可以将 `SIMULATOR_PROFILE` 设置为所选 YAML 路径。指定 `--config` 优先于该环境
变量。无配置参数的 status/stop 从运行目录读取已有状态，不需要重复指定配置。
启动前需加载当前桌面会话的
DISPLAY/XAUTHORITY；`render: offscreen` 可选择离屏运行。停止前确认 PX4 已降落并
disarm。此入口管理仿真和传感器 adapter，PX4/AS2 部署仍使用已有部署入口。

```text
config/simulator/default.yaml（或 --config 指定的 YAML）
├── backend              仿真后端
├── render               windowed / offscreen
├── ros_domain_id        仿真传感器 ROS 通信域，需与部署侧一致
├── projectairsim        安装路径：source / binary / python / ros_setup
├── scene                原生场景名，或自定义场景绝对路径
└── vehicles
    └── name / namespace
        ├── px4          可选：覆盖默认端口及 lockstep
        └── sensors
            └── 传感器名称
                ├── enabled
                ├── position / rotation
                ├── width / height / fov / captures（相机）
                └── channels / range / points_per_second / rotation_hz（LiDAR）
```

`position` 为原生机体 FRD 的米，`rotation` 为 roll/pitch/yaw 角度。
`captures` 支持 rgb、depth_planar、depth_perspective、segmentation。
未指定的物理、噪声、扫描参数保留上游模型。已有传感器按名称修改；新增 LiDAR
须填写 `type: lidar`，系统复用上游标准 LiDAR 模型。其他新增传感器在自定义原生
场景中定义，避免维护第二套机体/物理模型。

默认场景使用上游 `scene_px4_sitl.jsonc`，YAML 中的 DownCamera、IMU1 对应该场景
的传感器名称。多车辆使用含有对应 actor 的原生场景，并在 vehicles 中逐一配置。
车辆初始位置、物理模型和环境对象继续由原生 scene/robot 描述。

原生场景加载、默认 bridge 参数和 ROS 接口转换在实现侧完成，运行产物写入
`log/simulator/scene/`，包括 profile.json、runtime.json、resolved_scene.json 和
sensor-contract.json。无需在 log 里维护输入文件，也无需配置旧的 resources.yaml。
桥接器外部 overlay 的首次构建方法见 `simulator/projectairsim/SENSORS.md`。

旧 JSON profile 和 `--profile` 仍可使用。AirSim YAML 可使用同一原生字段：

```yaml
backend: airsim
binary: ~/AirSim/scenes/nh/AirSimNH/Binaries/Linux/AirSimNH
project: AirSimNH
settings: ../../simulator/airsim/config/settings/px4-sensors.json
args: [-RenderOffscreen, -vulkan, -nosound]
```

AirSim 的传感器仍在其原生 settings 中维护；本次简单传感器 YAML 选项已实现于
ProjectAirSim，尚未将其自动翻译到 AirSim settings。两种后端均从 config/ 选择入口。
