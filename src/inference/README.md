# Inference Robotic Function

Inference 是与 Perception、State Estimator 并列的基础能力：目标对外使用 ROS 2，对内通过 FastAPI 隔离每模型的独立 uv 环境。

当前提交提供公共 FastAPI runtime、ROS HTTP 客户端库、支持模型 manifest、用户 YAML 选择与准备入口，以及原生 AS2 双容器通信基准。尚未登记具体模型，也未提供可调用的模型 ROS Service；模型 schema、ROS 类型和节点在后续模型接入时交付。

## 文件职责

- `runtime/`：公共 FastAPI 应用工厂，每进程加载并预热一个模型，提供 `/health/live`、`/health/ready`、`/infer`。模型提供 factory、请求与响应 Pydantic 类型。
- `ros2_adapter/`：`uav_inference` ROS Python 包，当前提供内部 HTTP 客户端；具体 ROS 服务节点按模型接入。
- `manifest.json`：已支持模型 ID、仓库相对目录和描述。当前模型集合为空。
- `models/`：后续模型各自提供 README/setup、依赖声明和锁文件、schema 与推理 adapter。算法源码按功能归属保留原目录。

## 模型选择与准备

用户配置统一使用 YAML。将下面片段加入已有用户配置，通过 `--config` 指定路径。独立配置默认路径为 `config/inference.local.yaml`，该本地文件被忽略，示例见 [inference.example.yaml](../../config/inference.example.yaml)。读取脚本使用部署环境已有的 PyYAML（`import yaml`）。

```yaml
inference:
  models: []
```

```bash
python3 deployment/inference/setup_models.py --list
python3 deployment/inference/setup_models.py --config config/inference.example.yaml --dry-run
python3 deployment/inference/setup_models.py --config config/inference.example.yaml
```

有已登记模型后，在 `inference.models` 中填写其 ID，脚本先验证全部选择，拒绝未知或重复 ID，再按顺序调用各模型 `setup.sh`；失败停止。`--check-cuda` 将同名标志转交模型准备脚本。空列表不操作，不卸载未选模型、不启动服务。现有本地配置如选择尚未登记的模型，会明确报错，不自动改写用户配置。

manifest 不重复保存模型版本、权重或依赖。新增模型必须有 README、setup.sh、pyproject.toml 和 uv.lock，过程使用 [model-migration-cuda](../../skill/model-migration-cuda/SKILL.md)。每模型环境与 ROS overlay 分离，不重建稳定 AS2 镜像。

## 公共运行约定

模型 worker 和 ROS Adapter 属于算力角色，工作站通过 ROS 调用。内部 HTTP 不取代对外 ROS 契约；不同容器需显式配置可达 endpoint。具体请求类型、时间/坐标、ROS Service/Action/Topic 在模型接入时定义。

runtime 忙时返回429，不无限排队；调用方断开或取消后，底层推理完成前保持 busy。HTTP 超时不代表 GPU 计算已停止，客户端不自动重试旧观测。当前不提供跨模型 GPU 仲裁或自动启停管理。

## 验证

### 地面站到 onboard 的原生 AS2 通信基准

Inference 的部署验收以“地面站 ROS 客户端 → onboard ROS 服务端”为基准，客户端和服务端必须在不同容器/进程；同进程的本地 smoke 不替代这一层。先运行原生 AS2 只读查询基准：

```bash
python3 deployment/inference/tests/cross_container_baseline.py
```

脚本复用固定 `osrf/ros:humble-desktop-full-jammy` 与 `onboard_computer:v0.1.0`，在独立 Docker bridge/domain 中运行原生 Pixhawk Platform。地面站仅从 onboard 镜像的规范 `.srv` 定义局部生成 `ListControlModes` 查询类型，不复制 AS2 二进制，不运行替代服务。调用30次 `platform/list_control_modes`，要求每次返回 `source=Platform` 和非空模式列表；只测量时延，不设置性能达标门限。无 PX4、无运动调用，不修改现有运行实例。测试后清理本次临时容器和网络，镜像不重建。

2026-11-10 实测30/30成功，p50 3.32 ms、p95 3.88 ms；证据 `log/inference/as2-baseline/1391046/`，包含镜像 ID、规范服务定义哈希、逐次时延、构建及原生节点日志。此为单宿主双容器通信基准，不代表跨物理设备、完整 AS2 飞行链路或 Inference 已在该部署拓扑通过。

后续具体模型仍需验证地面站容器客户端 → onboard 容器中的 Inference ROS Adapter → 模型 worker → GPU 的链路；原生 AS2 基准通过不代表该模型链路通过。

### 公共逻辑测试

配置选择测试仅需系统 Python 与 PyYAML；runtime 测试使用独立临时环境，不依赖任何模型：

```bash
python3 -m unittest discover -s deployment/inference/tests -p test_selection.py -v
PYTHONPATH=src/inference/runtime uv run --no-project --python 3.8 --with-editable ./src/inference/runtime python -m unittest discover -s deployment/inference/tests -p test_runtime.py -v
```

当前证据覆盖公共逻辑和单宿主双容器 AS2 只读通信；不代表具体模型、跨物理设备、真实传感器、AirSim 或飞行闭环验收通过。
