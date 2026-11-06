# AirSim 启动

场景不包含在源码或交付镜像中。请先到 [AirSim 仓库 Releases](https://github.com/microsoft/AirSim/releases)
下载对应 Linux 场景并解压；当前 NH Demo 使用 AirSimNH。场景须与 Demo 匹配，
下载其他版本或场景后需要重新验证。然后按下文配置可执行文件路径。

在仓库根目录执行，脚本也支持从其他工作目录调用：

```bash
bash scripts/airsim.sh check
bash scripts/airsim.sh start
bash scripts/airsim.sh status
bash scripts/airsim.sh stop
```

首次运行先配置本机外部资源路径（宿主机 Python 需要 PyYAML）：

```bash
cp config/resources.example.yaml config/resources.local.yaml
# 编辑 resources.airsim_nh.path，填写已安装场景的可执行文件位置
```

路径映射沿用 LeUAV 的资源配置方式：先读取
`~/.config/aerohub/resources.yaml`（支持 `XDG_CONFIG_HOME`），再由
`config/resources.local.yaml` 按资源名覆盖。相对路径以 YAML 所在目录为准，
支持 `~`。本机文件被 Git 忽略，示例文件可以提交。

`nh.json` 通过 `binary_resource: airsim_nh` 选择场景资源，同时配置项目名称、
settings 和渲染参数。替换场景时，资源名与项目名称应一起核对。
脚本继承当前会话的图形环境，不自动猜测 DISPLAY 或桌面授权。

本机路径可临时覆盖：

```bash
AIRSIM_BINARY=/absolute/path/to/AirSimNH bash scripts/airsim.sh start
```

需要调整场景、settings 或渲染方式时，复制 `nh.json` 为同目录的
`nh.local.json`，修改资源名或参数后执行：

```bash
AIRSIM_PROFILE=config/airsim/nh.local.json bash scripts/airsim.sh start
```

profile 内 settings 和显式 binary 的相对路径以该 profile 所在目录为准，支持 `~`。
解析优先级为命令行 `--binary` / `AIRSIM_BINARY`、profile 的显式 `binary`、
最后是 `binary_resource` 的 YAML 映射。运行记录保存解析后的实际 binary 路径。
`args` 默认使用离屏 Vulkan；需要窗口时移除 `-RenderOffscreen`。
settings 和 userdir 参数由脚本管理，请勿加入 args。

| settings | 用途 |
| --- | --- |
| `settings/px4-single.json` | 单架 PX4，仅基础飞控传感器，用于传输验证 |
| `settings/px4-sensors.json` | 默认；单架 PX4、五向相机（七路图像）、16 线 LiDAR |
| `settings/px4-cameras.json` | 单架 PX4、五向相机（七路图像），不含 LiDAR |
| `settings/px4-lidar.json` | 单架 PX4、16 线 LiDAR，不配置额外相机 |
| `settings/px4-two-lidar.json` | 两架 PX4，各带 16 线 LiDAR，间距 3 米 |

单机 settings 都使用车辆名 `PX4`、RPC 端口 `41471`、PX4 TCP 端口 `4580`、
控制端口 `14640/14680`、SteppableClock 和 LockStep。
全部配置显式启用 IMU、GPS、磁力计和低噪声气压计，ClockSpeed 为 1。
没有继承 LeUAV 案例的 GPS 延迟与控制调参，保持本仓库 PX4 1.18 接入约定。
当前 flight_board 验证配置对应 PX4 instance `20`；启动脚本只负责场景，
PX4、DDS、ROS 传感器和 AS2 继续由各自部署入口运行。
修改车辆名称、端口或传感器时，需同步对应接入配置；场景和传感器节点必须
读取同一份 settings。现有部署默认引用这里的 `px4-sensors.json`。

## 场景与 settings 对应

| profile | 资源名 | 项目名 | settings |
| --- | --- | --- | --- |
| `nh.json` | `airsim_nh` | `AirSimNH` | `px4-sensors.json` |
| `town05.json` | `airsim_town05` | `CarlaUE4` | `px4-cameras.json` |
| `plant.json` | `racer_yolo_plant` | `SuperGridStarterpack` | `px4-two-lidar.json` |

```bash
bash scripts/airsim.sh check --profile config/airsim/town05.json
bash scripts/airsim.sh start --profile config/airsim/town05.json
bash scripts/airsim.sh stop
```

`check` 解析资源和 settings，检查可执行文件、车辆连接方式与重复端口，
输出实际路径及车辆配置；不启动进程，也不检查端口占用。start 才检查 TCP
监听端口占用。三个场景共用 RPC 端口和默认运行目录，应停止当前场景后再切换。

Plant 双机 settings 使用 `Drone0` / `Drone1`，TCP `4580/4581`，
控制端口分别为 `14640/14680`、`14641/14681`，出生点 NED 为
`[0,0,0]` / `[0,-3,0]`。LiDAR 使用 SensorLocalFrame，XYZ 为机体安装位置，
16 线、25 米、10 转/秒、80000 点/秒；与本仓库传感器后端保持一致。
相机使用 256×256、90 度视场，前/下 RGB 与 DepthPerspective，左/右/后为
DepthPerspective。深度是视线距离，不能直接当作光轴 Z 深度。

当前 flight_board Compose 仍只接单机 `PX4`；Plant 双机需要按上述车辆名与
端口装配两个 flight_board，不能直接套用单机 Compose。这些 profile 是场景
启动配置，不包含 LeUAV 的 AeroVLA、Racer 任务与算法；新增组合尚未实景验收。

## 参考来源

对照本地 LeUAV `69e1226dd4fe1479ce423b9eea4919da714d900f` 的
`resources.local.yaml`、`profiles/settings/airsim.json`、
`cases/aerovla/config/settings.json`、`cases/racer_yolo/config/settings.airsim.json`
整理场景路径、相机参数和双机布局。端口、LiDAR 坐标及基础飞控参数按本仓库
重新配置，不依赖相邻仓库运行，也没有迁入算法源码或场景二进制。

运行记录默认写入 `artifacts/airsim/scene/`，包括实际 settings、profile、命令、
进程身份和 `airsim.log`。可用 `AIRSIM_RUN_DIR` 更换目录，后续 status/stop
须使用同一目录。`--timeout 90` 等参数可追加到脚本命令末尾。

start 等待本次进程监听 RPC 和 PX4 TCP 端口，不连接 PX4 TCP 端口；
监听就绪不代表飞控或飞行就绪。stop 核对 PID、启动时间、可执行文件与
settings 参数后关闭本次场景。不会覆盖 `~/Documents/AirSim/settings.json`。

生命周期实现沿用本仓库从 LeUAV 借鉴的进程身份及监听检查，源码参考
`leuav/src/leuav/runtime/processes.py`，运行时无需 LeUAV。
