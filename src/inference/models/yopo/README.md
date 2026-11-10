# YOPO 推理模型

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

`adapter.py` 对照锁定 `test_yopo_ros.py` 的参考规划入口，包含 resize、20m截断、uint8量化、inpaint、CPU normalize/prepare/primitive解码；不使用会重复处理状态的简化 wrapper。`schemas.py` 定义内部 HTTP 输入输出；`serve.py` 接公共 runtime。`.venv/` 和 `artifacts/` 均忽略，不提交模型产物或环境。

已验证 CPU 固定输入与迁移前输出一致、ROS→HTTP→CPU 模型往返，以及准备脚本在可访问设备的环境中的 CUDA 加载/预热。GPU 的双容器 ROS 往返已验证（320×185合成输入10/10）；真实传感器与 AirSim 的逐轮结果见 [Demo验收](../../../../deployment/airsim/demos/yopo_single/validation.md)。受限沙箱可能无法访问 GPU，不能据此直接判定宿主驱动不可用。输出是候选终端状态与评分，不是完整轨迹或控制指令。

`YOPO_VELOCITY` 可选择原算法已有的速度缩放（本轮支持0到6 m/s，不含0）；未指定保持上游6 m/s。`LatticeParam` 同步缩放速度、加速度和轨迹时长，不能仅限制输出速度而保留原时间。单机 Demo 使用2 m/s，对应5 s候选时长、约0.667 m/s²原语加速度尺度。响应 model_version 包含权重SHA256、预处理版本和速度配置；HTTP readiness 另给出锁定源码版本。修改速度须重启 worker 并按同配置重新对照原入口，不在任务中热换。

可重放基线（CPU，atol=1e-5、rtol=1e-4）：

```bash
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' YOPO_DEVICE=cpu YOPO_VELOCITY=2 src/inference/models/yopo/.venv/bin/python src/inference/models/yopo/test_baseline.py
```

该测试从锁定原 ROS 文件 AST 提取预处理代码，覆盖无效深度、不同尺寸、非零速度/加速度和非恒等姿态，比较全部候选和评分。ROS stride/端序/单位转换由独立 `deployment/inference/tests/test_contract.py` 验证，不将其当作相机几何证明。
