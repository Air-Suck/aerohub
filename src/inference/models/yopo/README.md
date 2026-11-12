# YOPO 推理模型

后端选择及使用者/开发者准备条件见 [Inference 前置条件](../../README.md#开始前准备什么)。

本目录仅管理 YOPO 的推理适配、独立环境和模型产物。算法源码仍是 `src/planner/yopo/upstream/` 中的 Planner 子模块，锁定版本 `bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132`，上游采用 MIT 许可证。ROS 对外接口和公共服务结构见 [Inference 说明](../../README.md)。

## 准备

前置条件：Linux x86_64、Git、uv、可获取 Python 3.8 和锁定 wheel 的网络或缓存。正常运行另需兼容 NVIDIA 驱动与 GPU；无需安装 ROS 到模型 venv。

在仓库根目录执行：

```bash
git submodule update --init src/planner/yopo/upstream
bash src/inference/models/yopo/setup.sh
```

YOPO 在公共 `src/inference/manifest.json` 中的 ID 为 `yopo`。也可在已有用户 YAML 的 `inference.models` 中选择它，再运行 `python3 deployment/inference/setup_models.py --config <用户配置.yaml>`；具体选择规则见公共 Inference 说明。

也可从任意工作目录通过脚本路径执行。脚本会：

1. 检查算法版本，保留已有 checkout，不自动切换或更新源码。
2. 从上游已跟踪的 `YOPO/saved/YOPO_1/epoch50.pth` 准备 `artifacts/epoch50.pth`，按 `weights.sha256` 校验来源和目标。无需另外下载权重；已有目标校验不符时失败，不覆盖。
3. 用 `uv sync --locked` 创建或同步本目录 `.venv`，不修改系统 Python 或其他模型环境。
4. 显式使用 CPU 加载权重并预热，验证依赖与模型文件组合。不会启动 HTTP/ROS 服务，也不代表 GPU 已验收。

需要同时检查 GPU 时执行：

```bash
bash src/inference/models/yopo/setup.sh --check-cuda
```

CUDA 不可用时该选项失败，不降级。`CUDA_VISIBLE_DEVICES` 可以选择 GPU。准备脚本固定准备默认权重，不消费调用环境中的 `YOPO_WEIGHTS`；自定义权重只在运行时显式指定。

## 独立依赖

- Python 3.8、PyTorch 2.4.1 + CUDA 11.8 wheel，保持既有可用基线。
- NumPy、SciPy、headless OpenCV、ruamel.yaml：网络和输入预处理实际使用的依赖。
- 公共 FastAPI runtime：通过仓库相对路径依赖加载。
- PyTorch 所需 NVIDIA 库及 `triton` 编译器按同一官方 cu118 索引和精确版本锁定。这里的 `triton` 是 PyTorch 依赖，不是 Triton Inference Server。

不安装训练、数据可视化、ROS 或原仓库全部 requirements。准确版本和来源以 `pyproject.toml`、`uv.lock` 为准；日常准备不更新锁文件。改变依赖时单独更新锁并重新验证模型输出。

## 运行

```bash
bash deployment/inference/start_yopo.sh
```

默认加载本目录准备好的 `artifacts/epoch50.pth`，要求 CUDA。`YOPO_WEIGHTS` 可覆盖权重路径，`YOPO_THREADS` 控制 PyTorch CPU 线程数，`INFERENCE_HOST`/`INFERENCE_PORT` 控制内部 HTTP 地址。模型仅加载一次并预热，具体 ROS 启动及请求契约见公共说明。

CPU 合成冒烟：

```bash
CUDA_VISIBLE_DEVICES='' YOPO_DEVICE=cpu bash deployment/inference/start_yopo.sh
```

## 文件与验证边界

`processing.py` 对照锁定 `test_yopo_ros.py` 的参考规划入口，包含 resize、20m截断、uint8量化、inpaint、CPU normalize/prepare/primitive解码；不使用会重复处理状态的简化 wrapper。`schemas.py` 定义内部 HTTP 输入输出；`serve.py` 接公共 runtime。`.venv/` 和 `artifacts/` 均忽略，不提交模型产物或环境。

已验证 CPU 固定输入与迁移前输出一致、ROS→HTTP→CPU 模型往返，以及准备脚本在可访问设备的环境中的 CUDA 加载/预热。GPU 的双容器 ROS 往返已验证（320×185合成输入10/10）；真实传感器与 AirSim 的逐轮结果见 [Demo验收](../../../../deployment/airsim/demos/yopo_single/validation.md)。受限沙箱可能无法访问 GPU，不能据此直接判定宿主驱动不可用。输出是候选终端状态与评分，不是完整轨迹或控制指令。

`YOPO_VELOCITY` 可选择原算法已有的速度缩放（本轮支持0到6 m/s，不含0）；未指定保持上游6 m/s。`LatticeParam` 同步缩放速度、加速度和轨迹时长，不能仅限制输出速度而保留原时间。单机 Demo 使用2 m/s，对应5 s候选时长、约0.667 m/s²原语加速度尺度。响应 model_version 包含权重SHA256、预处理版本和速度配置；HTTP readiness 另给出锁定源码版本。修改速度须重启 worker 并按同配置重新对照原入口，不在任务中热换。

可重放基线（CPU，atol=1e-5、rtol=1e-4）：

```bash
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' YOPO_DEVICE=cpu YOPO_VELOCITY=2 src/inference/models/yopo/.venv/bin/python src/inference/models/yopo/test_baseline.py
```

该测试从锁定原 ROS 文件 AST 提取预处理代码，覆盖无效深度、不同尺寸、非零速度/加速度和非恒等姿态，比较全部候选和评分。ROS stride/端序/单位转换由独立 `deployment/inference/tests/test_contract.py` 验证，不将其当作相机几何证明。

Houmo 后端适配与工具链试验维护在 [houmo/](houmo/README.md)，使用容器 `houmo-migrate-yopo`。HMM 已完成 RoboPi 真实 ROS → HTTP → NPU 快速验证；下游闭环仍待验收。


## 共用模型逻辑与执行后端

CUDA 与 HMM 共用 `processing.py` 和 `predictor.py`：前者只依赖 NumPy/OpenCV，
负责图像、状态与候选语义；后者统一输入检查、预热、推理、输出解码与响应组装。
`adapter.py` 负责锁定 YOPO 源码/配置，并构造公共 `TorchBackend`；
`houmo/adapter.py` 读取导出参数并构造 `HmmBackend`。板端打包携带这两份共享文件，
无需 Torch 或上游源码。两种 backend 的[张量契约](../../runtime/aerohub_inference/backends/README.md)
统一，具体设备与模型加载由各自实现负责。

Torch backend 位于 `src/inference/runtime/aerohub_inference/backends/torch.py`，
可供其他 Torch 模型使用。它只接受命名张量和模型 factory，不包含 OpenCV、ROS 或
YOPO 逻辑。公共 runtime 不在导入时加载 Torch；板端 HMM 环境也不安装 Torch。
当前 YOPO 仍沿用原有环境、启动脚本及环境变量，依赖锁未改变。

两种 worker 的 timing 字段统一为 `preprocess`、`input_transfer`、
`forward_and_sync`、`output_transfer`、`decode`、`total`；保留
`forward_and_transfer` 作为三项引擎计时之和，兼容既有 ROS timing 消费者。
该聚合项与分项重叠，不能把所有字段相加。GPU 前向时间包含设备同步。

2026-11-11 整理后的验证：

- RTX 4090 真实 CUDA：6 和 2 m/s 各三组固定输入，与旧实现全部候选、评分及原有
  model_version/device/时长完全一致（候选最大差异 0）。
- 两种速度通过原 ROS 预处理 AST 和锁定上游模型全候选对照；Torch 后端真实 GPU
  执行、输入契约、输出独立性及关闭测试通过。
- 工作站容器 → onboard ROS adapter → 新 CUDA worker，320×185 合成输入 10/10
  成功，RTT 中位约 22.34 ms，最大 36.01 ms；这是本轮合成回归记录。
- 同一共享处理代码在 RoboPi 通过原 ROS/HMM 快速矩阵，包括三个正常请求、直接
  HTTP 候选对照、健康诊断、过期输入和 worker 离线。

证据在 `log/yopo-cuda-refactor/`。这些检查验证整理没有改变已覆盖语义，
不扩大此前真实传感器、量化精度或飞行验收范围。
