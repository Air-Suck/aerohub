# AirSim 仿真集成

本目录管理 AirSim 的场景配置、启动与传感器桥，不包含 AirSim 引擎源码，也不改变主线 `simulator/gazebo/` 子模块。

- `config/`：通用场景 profile、PX4/传感器 settings、外部场景可执行文件路径映射。
- `scenarios/`：CAT-ORA、Swarm-Formation、SUPER 演示所用的场景输入配置。
- `manager/`、`airsim.sh`：模拟器进程的检查、启动、状态与停止入口。
- `sensors/`：AirSim 数据到 ROS 2 的传感器桥。

管理器在所有 profile 启动后运行顶视跟踪器，它与管理器使用同一个主机 `python3`，需要 PyYAML 和 msgpack。Ubuntu 22.04 主机用系统包安装，与 flight-board 镜像的 `python3-msgpack` 来源一致：

    sudo apt-get install python3-yaml python3-msgpack

无法使用系统包时改为 `python3 -m pip install --user -r deployment/swarm/requirements.txt`；需要代理时在当前 shell 显式设置，不写入仓库。`manager.py check/start` 会先检查 msgpack，缺失时在启动 AirSim 前失败。

在仓库根目录运行 `bash simulator/airsim/airsim.sh --help` 查看入口。运行场景需要事先配置外部可执行文件；管理入口不自动启动 PX4、AS2 或算法。跨组件编排在 `deployment/airsim/demos/`，板端管理在 `tools/robopi/`。

本机路径通过 `config/resources.local.yaml` 或用户级 `~/.config/aerohub/resources.yaml` 配置，不提交机器专属路径。运行产物放在仓库 `log/airsim/`，历史验收证据在忽略的 `log/pr-history/`，不属于场景源码。
