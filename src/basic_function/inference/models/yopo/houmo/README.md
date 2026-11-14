# YOPO Houmo 迁移试验

仅运行已有 HMM 包或开发迁移模型所需的条件见 [Inference 前置条件](../../../README.md#开始前准备什么)。

工具准备与通用流程见 [model-migration-houmo](../../../../../../skill/model-migration-houmo/SKILL.md)。外部工具是固定 Houmo 工具链镜像和仓库已有 OHOS Native SDK；postmo-modelzoo 不是依赖。容器名为 `houmo-migrate-yopo`。

在忽略的 `.env` 中设置 `OHOS_NATIVE_SDK` 和已有 AArch64 `TCIM_RUNTIME_PATH`，初始化锁定算法源码后进入迁移容器：

```bash
git submodule update --init src/tool/navigation/planner/yopo/upstream
bash skill/model-migration-houmo/scripts/run.sh yopo
```

模型网络边界是 YOPO `YopoNetwork.forward`，不是完整的 `inference()`。实际接口为：

| 接口 | shape | HMM dtype |
| --- | --- | --- |
| `depth` | `[1,1,96,160]` | float16 |
| `obs` | `[1,9,3,5]` | float16 |
| `predictions` | `[1,9,3,5]` | float16 |
| `scores` | `[1,3,5]` | float16 |

状态归一化、lattice 输入展开和候选 `pred_to_endstate` 仍在 CPU 侧，不能因 HMM 成功而省略。

## 转换

导出额外需要 `requirements.txt` 中的 `ruamel.yaml`。默认容器无网络，先准备 wheel，再安装到容器临时目录：

```bash
mkdir -p src/basic_function/inference/models/yopo/houmo/artifacts/wheels
python3 -m pip download --no-deps \
  --dest src/basic_function/inference/models/yopo/houmo/artifacts/wheels \
  -r src/basic_function/inference/models/yopo/houmo/requirements.txt
python3 -m pip install --target /tmp/yopo-deps --no-index --no-deps \
  --find-links src/basic_function/inference/models/yopo/houmo/artifacts/wheels \
  -r src/basic_function/inference/models/yopo/houmo/requirements.txt
PYTHONPATH=/tmp/yopo-deps python3 src/basic_function/inference/models/yopo/houmo/export_onnx.py

cd src/basic_function/inference/models/yopo/houmo
hmatc gen --onnx artifacts/yopo.onnx --output artifacts/generated.yml
hmatc quant -c config.smoke.yml
hmatc build -c config.smoke.yml
PYTHONPATH=/tmp/yopo-deps python3 compare_hmquant.py
```

`export_onnx.py` 默认要求真实 CUDA，固定 opset 17，并比较八组固定输入的全部输出（rtol=`1e-4`、atol=`1e-5`）。`config.smoke.yml` 和合成 NPZ 只验证转换链路，不能作为代表性量化校准。

## 板端 runner

`runner/` 按固定路径装载 HMM、两路 FP16 输入并写回两路输出。在迁移容器内构建：

```bash
bash src/basic_function/inference/models/yopo/houmo/build_runner.sh
```

将 `yopo_hmm_runner`、HMM、`libtcim_runtime_lite.so`、匹配的 HAL 和两路输入上传到 RoboPi 隔离目录后执行。runner 输出 `setup_ms`（加载模型并装填输入）和 `run_ms`（单次同步前向）。完成后取回输出并使用 `compare_npu.py` 对照参考。

## 本轮状态

2026-11-11 已实际执行：

- ONNX 对 PyTorch 的八组固定合成输入全部通过，最大绝对误差 `1.72e-6`。
- HMATC 量化与 HMM 编译成功；HMM 哈希 `b685a2ec...75470bc4`。
- HMONNX 对八组参考的最佳候选一致 `6/8`；输入只量化到 FP16 时，另外两组最佳索引发生变化。
- RoboPi AArch64 NPU 上五次推理均成功且输出哈希一致；标准容器内构建的 runner 单次前向 `1.241–1.719 ms`，中位数 `1.342 ms`。
- 第一组样本的 NPU 最佳候选为 13，原始参考为 14；该样本是随机合成输入，不能据此判断真实场景精度。

日志在 `log/yopo-houmo/`。尚未完成代表性校准数据、全部候选语义门槛、真实传感器输入、下游规划和 AirSim/PX4 闭环验收。最初转换试验使用显式 CPU 模式，真实 GPU 导出/量化尚未验证。早期沙箱内 GPU 探测失败不能据此判定宿主驱动故障；后续 CUDA 整理已在真实 RTX 4090 上完成模型推理回归，见上层模型说明。

## RoboPi Python 服务

共用执行后端在 `src/basic_function/inference/runtime_houmo/aerohub_houmo/backend.py`：加载任意
HMM，读取命名输入输出契约，检查 shape 并转换 dtype，串行执行 TCIM 并复制输出。
YOPO 的共用 `../processing.py` 负责深度预处理、状态展开和候选解码，
共用 `../predictor.py` 负责请求执行，`adapter.py` 只构造 HMM 后端；`serve.py` 复用现有
`aerohub_inference.create_app`，提供 `/health/live`、`/health/ready`、`/infer`。
ROS adapter 使用相同请求响应，只需将 `endpoint` 指向该服务。

已查询的板端环境为 Python 3.12.12（`cpython-312-aarch64-linux-ohos`）、ROS 2
Humble，已有 numpy、cv2、Pydantic 2、anyio、click、h11。缺少的 FastAPI、Starlette、
Uvicorn 放在服务私有目录，版本见 `requirements-board.txt`。其他系统镜像应先核对这些
依赖，不能将此板环境当作所有 RoboPi 的默认安装。

构建 TCIM Python 绑定时，在 `.env` 追加 `OHOS_SYSDEPS` 指向现有目标依赖目录
（含 `include/python3.12/Python.h`、`lib/libpython3.12.so`）。`TCIM_RUNTIME_PATH`
需要包含同版本 `python/` 绑定源码；无需 postmo 源码。执行于迁移容器：

```bash
bash skill/model-migration-houmo/scripts/build_tcim_python.sh \
  /workspace/src/basic_function/inference/models/yopo/houmo/artifacts/board-tcim
python3 -m pip download --no-deps \
  --dest src/basic_function/inference/models/yopo/houmo/artifacts/wheels/board \
  -r src/basic_function/inference/models/yopo/houmo/requirements-board.txt
python3 src/basic_function/inference/models/yopo/houmo/build_board_runtime.py \
  --tcim-lib /opt/houmo/tcim/lib/libtcim_runtime_lite.so \
  --hal-lib /path/to/target/libhal_xh2a.so
```

下载 wheel 这一步在能联网的环境执行；默认迁移容器无网络。HAL 路径使用匹配板端的
实际文件。两个构建脚本的输出目录必须尚不存在，重复构建用新的输出路径。
`build_board_runtime.py` 默认生成 `artifacts/board-runtime/`，含 HMM、导出的
`yopo-runtime.npz`、TCIM 绑定、目标动态库与服务代码；无需在板上安装 Torch。

将整个 `board-runtime/` 上传到 RoboPi 应用目录，确认匹配的 NPU 驱动已加载，然后执行：

```sh
INFERENCE_PORT=8000 /system/bin/sh /path/to/board-runtime/start_board.sh
```

脚本默认监听 `127.0.0.1`、单 worker。板端 ROS 节点配置
`endpoint:=http://127.0.0.1:8000`，使用原有 `InferYopo` 接口；其接口包必须先按
OHOS 构建流程交叉编译部署。CUDA uv 环境不用于板端。

2026-11-11 修正观测张量转置后：后端测试 3 项、YOPO 对照上游预处理/解码测试 2 项
全部通过；真实板端 `/health/ready` 返回 200，三次 `/infer` 均返回 15 个候选且最佳
索引一致，缺少字段返回 422。前向为 1.31–1.51 ms；服务内部总耗时首次约 43.1 ms，
随后约 6.5–7.3 ms。这是固定合成请求的服务冒烟，不是实时性能或精度验收。
后续已补齐板端 ROS overlay 并完成 service 调用，结果见下节。

板端服务启动后可运行随仓库分发的 HTTP 冒烟脚本：

```sh
INFERENCE_ENDPOINT=http://127.0.0.1:8000 \
  /sys_prod/robot/out/bin/python3 /path/to/tests/http_smoke.py
```

它检查健康接口、连续三次合成请求的候选数及设备标识，并要求无效请求返回 422。
最终验证日志保存在 `log/yopo-houmo/python-http-final.log` 和
`log/yopo-houmo/python-contract-tests.log`；测试 worker 已停止，测试加载的驱动已卸载。

## 板端 ROS service 快速验证（2026-11-11 已通过）

复用原有 `uav_inference.yopo_node`，真实链路为板端 ROS 客户端 →
`/inference/yopo/infer`（`uav_inference_interfaces/srv/InferYopo`）→ HTTP → HMM/NPU。
接口包与 adapter 部署在应用私有的 `ros/install/`，系统 ROS 与 AS2 安装保持原样。

在按 OHOS 构建说明准备好的隔离工作区中执行（`ws/src/` 下这两个包应尚不存在）：

```bash
export OHOS_WORK_ROOT=log/yopo-houmo/ros-build
export OHOS_SYSDEPS=/path/to/target/deps/out
export OHOS_INSTALL_PREFIX=/path/on/board/yopo/ros/install
bash deployment/inference/build_ros_ohos.sh
tar -czf log/yopo-houmo/ros-overlay.tar.gz \
  -C "$OHOS_WORK_ROOT/ws" install
```

`OHOS_INSTALL_PREFIX` 必须对应板端实际路径。将归档解到板端应用的 `ros/`，并将
`deployment/inference/tests/verify_houmo_ros_board.py` 上传为应用根目录的 `verify_ros.py`，
将同目录的 `run_houmo_ros_board.sh` 上传到板端。应用根目录同时包含此前的
`board-runtime/`。确认匹配驱动已加载、测试端口 18082 空闲，执行：

```sh
env -i HOME=/data/local/tmp PATH=/system/bin /system/bin/sh \
  /path/to/run_houmo_ros_board.sh /path/on/board/yopo
```

第二个可选参数是已有 ROS 环境入口，默认 `/data/aerohub/as2/board_env.sh`。
验证使用既有 FastDDS UDP 配置和独立 domain 187（可用
`INFERENCE_TEST_ROS_DOMAIN_ID` 覆盖）；板端系统 CycloneDDS 因 `mq_unlink` 缺失不可用。
验证脚本自行启动并回收 worker 和 ROS adapter，不加载或卸载驱动。

正常长期启动可加载相同 ROS 基础环境与应用 overlay 的 AMENT、LD_LIBRARY_PATH、
PYTHONPATH 后，保持 HTTP worker 运行并启动：

```sh
python3 -m uav_inference.yopo_node --ros-args \
  -p endpoint:=http://127.0.0.1:8000
```

本轮全部快速检查通过：

- Python 3.12 OHOS 类型支持扩展加载、ROS service 发现成功。
- 三次 ROS 请求均成功，返回 15 个候选及 `device=houmo:xh2:0`。
- 候选 PVA、score、最佳索引、model_version、时长与直接 HTTP 结果一致；请求 ID 和两个 header 保留。
- `/inference/status` 报告 ready；目标生成器的诊断 byte 字段表示为 `bytes`，验证时兼容处理。
- 过期输入返回 `INVALID_INPUT`；停止 worker 后返回 `WORKER_UNAVAILABLE`。
- 最终三次 ROS 调用总耗时约 116/97/97 ms，包含 ROS 消息、转换与 HTTP，不能当作 NPU 前向耗时。

证据：`log/yopo-houmo/ros-board-run.log`、`ros-results/result.json`、
`ros-results/worker.log`、`ros-results/adapter.log`，构建证据 `ros-build-final.log`。
测试进程已退出，修复了原 ROS 节点收到 SIGINT 后重复 `rclpy.shutdown()` 的问题。
以上验证仅覆盖声明的接口快速检查；代表性量化精度、真实传感器和规划闭环仍未验收。
