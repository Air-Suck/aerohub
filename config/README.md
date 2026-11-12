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

安装资源统一登记在 `config/resources.yaml`，包括 UE4、UE5、AirSim 和 ProjectAirSim。
本机路径差异放在已忽略的 `config/resources.local.yaml`，同名资源覆盖共享值。
旧用户级及 AirSim 资源配置仍可读取；根目录配置优先，本机覆盖最后生效。
场景 YAML 通过 `source_resource`、`binary_resource`、`python_resource` 和
`ros_setup_resource` 引用资源名；原先直接填写路径的方式仍可使用。
UE 资源目前用于登记和查询，尚未提供自动启动编辑器的命令。

客户端环境归属 ProjectAirSim，放在 `~/ProjectAirSim/.venv/`。在仓库根目录准备：

```bash
uv venv --python 3.12 ~/ProjectAirSim/.venv
uv pip install --python ~/ProjectAirSim/.venv/bin/python \
  -r simulator/projectairsim/requirements-client.txt
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
├── projectairsim        安装资源引用，或直接填写 source / binary / python / ros_setup
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

## UE 编辑器开发

打开 Blocks 工程，点击 Play 后，从仓库根目录连接当前实例：

```bash
bash simulator/simulator.sh attach --config config/simulator/blocks.local.yaml
bash simulator/simulator.sh status
bash simulator/simulator.sh stop
```

`attach` 会按同一份 YAML 重新加载无人机和传感器，并启动桥接；只在没有活动
飞行任务时使用。它不会启动第二个 Blocks，也不会管理编辑器进程。
`stop` 仅停止该会话启动的桥接，不停止 UE Play、不卸载无人机、不停止 PX4。
因此飞行结束并降落、disarm 后再 stop。停止 Play 后端口失活时 status 返回失败。
重新 Play 前先 stop 旧桥接，再 attach。当前 status 检查服务端口及桥接进程存活，
不代表传感器持续出数或 PX4 已连接；仍须检查下游数据新鲜度。

## 观察显示

安装独立的 `AerohubDisplay` UE 插件后，同一份场景 YAML 的 `display` 字段控制
顶部状态条与速度仪表盘。启用 display 时 manager 自动管理真实运动学速度转发进程；
当前相机小画面不绘制。初始化安装、数据语义和接口见
[UE 显示说明](../simulator/projectairsim/unreal/README.md)。

```bash
bash simulator/simulator.sh display --event status
bash simulator/simulator.sh display --event start
bash simulator/simulator.sh display --event finish
bash simulator/simulator.sh display --event reset
```

这些事件只操作显示计时器，不代替实际任务或飞控指令。
