---
name: model-migration-houmo
description: 使用固定 Houmo 工具链 Docker 将 ONNX 量化编译为 HMM，并衔接 RoboPi 验证；按模型隔离容器，复用已有 OHOS SDK。
---

# Houmo 模型迁移

## 任务入口

先按 [Inference 前置条件](../../src/inference/README.md#开始前准备什么)确定任务。
仅运行已有板端包时，按模型 README 启动并验证服务；需要重新导出、量化、编译或
打包时再进入下列迁移步骤。开始前核对各阶段资源是否已存在，明确缺少的是工具、
目标开发资源还是模型文件，避免把模型适配过程列成额外安装工具。

## 工具准备

完成迁移需要两项外部工具：

1. Houmo 工具链镜像
   `harbor.houmo.ai/toolchain/release:Dadao-xh2-v1.5.0-ubuntu24.04-x86.64`。
   用 `docker pull` 获取镜像；镜像仓库需要凭据时由发行方提供。镜像已包含 HMATC、HMQuant、HMM 编译器和 x86 Houmo SDK，不再安装宿主机 Houmo SDK。基线镜像 ID 为 `sha256:ec2762ddf97360f64e54214f6c7c60ef1aabaf5deb5f83e718b048400f351aea`。
2. OHOS Native SDK
   优先复用仓库现有构建环境，准备方式见[OHOS 构建说明](../../deployment/oh-ros-build/README.md)。在忽略的 `.env` 中设置 `OHOS_NATIVE_SDK` 指向 `native/`；迁移容器只读挂载该目录。它用于编译 RoboPi runner/Python 扩展，不参与 HMM 编译。当前 `run.sh` 入口会检查 SDK 路径，进入容器前需配置。

RoboPi 侧复用与目标系统匹配的 TCIM/HAL/驱动。需要编译 runner/绑定时，
`TCIM_RUNTIME_PATH` 指向 AArch64 TCIM 开发包；编译绑定还要有同版本 `python/`
源码和 `OHOS_SYSDEPS` 中的目标 Python 头文件、`libpython`。已有可用绑定可直接
打包复用。目标 HAL 库由打包脚本参数传入，驱动由板端环境提供。
新开发者缺少这些资源时需取得匹配的目标包，镜像内 x86 SDK 不能替代它们。
ROS overlay 另复用已有 OHOS ROS builder/工作区；阶段与配置映射见上述前置条件。

`postmo-modelzoo` 可作实现参考，不是用户依赖。ONNX 导出、图改写、输入处理及模型特有 Python 包按实际模型在模型目录内解决并记录，不提升为所有模型的前置工具。

## 开始迁移

读取模型现有入口、锁定源码与权重。算法源码保持原分类；Houmo 适配维护在 `src/inference/models/<model>/houmo/`，与原有推理实现并列。先确定输入输出契约和对照门槛，再导出或改写图。

```bash
docker pull harbor.houmo.ai/toolchain/release:Dadao-xh2-v1.5.0-ubuntu24.04-x86.64
bash skill/model-migration-houmo/scripts/run.sh yopo
bash skill/model-migration-houmo/scripts/run.sh yopo python3 --version
```

容器统一命名为 `houmo-migrate-<model-name>`。脚本复用本工作区的同名容器；其他工作区占用时失败并保留容器。仓库挂载为 `/workspace`，OHOS SDK 挂载为 `/opt/ohos-sdk/native`，TCIM 开发包挂载为 `/opt/houmo/tcim`。转换容器默认无网络、默认不申请 GPU；量化工具明确要求 CUDA 时设置 `HOUMO_MIGRATE_GPU=1`，并用真实 GPU 执行。

模型特定依赖先离线下载到该模型目录，再在容器中安装到私有路径，不修改共享镜像。使用镜像内 HMATC，不默认用 ModelZoo 源码覆盖工具链。

## 转换与验证

1. 从模型定义和固定权重导出 ONNX，检查输入输出名称、shape、dtype 和顺序。
2. 固定测试输入，对照原模型与 ONNX 的全部输出。多输入样本必须成对，不能分别随机组合。
3. 执行 `hmatc gen --onnx ...`，补全真实预处理和校准数据后执行 `hmatc quant -c ...`。
4. 执行 `hmatc build -c ...`。编译成功只证明图可编译；无 NPU 设备时 golden check 会跳过，不能记为板端通过。
5. 对照 HMONNX 与原模型输出。分类/规划类模型除张量误差外，还要比较候选顺序、最佳索引或其他会改变下游决策的语义。
6. 用 AArch64 TCIM runner 在 RoboPi 上执行固定张量，取回实际输出再对照。记录加载时间、前向时间和重复运行哈希。
7. 最后才验证 ROS 接口、真实传感器输入和下游消费；这些结果与 HMM 生成分开记录。

合成输入可以验证导出和工具链，不能证明代表性量化精度。实际部署前按需读取 [oh-ros-build](../oh-ros-build/SKILL.md) 和 [hdc-helper](../hdc-helper/SKILL.md)，核对目标 ABI、HAL、驱动和模型中的 TCIM 版本。

YOPO 的已验证入口、数值和限制见[模型说明](../../src/inference/models/yopo/houmo/README.md)。过程保存在 `skill/temp/<任务>/`，原始证据保存在 `log/`，转换产物保存在模型忽略的 `houmo/artifacts/`。

## Python HMM 服务

沿用 `src/inference/runtime/aerohub_inference.create_app` 的服务契约；
`src/inference/runtime_houmo/aerohub_houmo.HmmBackend` 提供通用命名张量执行，
模型 CPU 前后处理在模型共享层维护，`<model>/houmo/adapter.py` 只准备参数并构造后端。ROS adapter 通过原 HTTP
协议调用，后端选择由 endpoint 对应的 worker 决定。

板端没有 `tcim_lite` 时，用 `scripts/build_tcim_python.sh OUTPUT` 从匹配的 TCIM
开发包交叉编译。额外复用目标 Python 依赖快照，通过 `OHOS_SYSDEPS` 挂载为
`/target-out`；需匹配 Python 头文件和 `libpython`。OHOS Native SDK 内的 Python 是
主机程序，不能作为目标 Python 库。当前脚本适用 Python 3.12 / AArch64 OHOS，显式
链接目标 `libpython3.12`，并将扩展命名为目标 SOABI；其他 Python 版本需先调整核对。

YOPO 的打包脚本、板端依赖核对、启动方式和已验证范围见[模型说明](../../src/inference/models/yopo/houmo/README.md#robopi-python-服务)。
先验证 CPU 前后处理与上游一致，再验证实际 HMM HTTP 请求；服务冒烟与 ROS 调用
分别记录。板端服务缺少自定义 ROS 接口包时，按 OHOS Skill 编译部署后再验收 ROS，
不以 HTTP 成功代替 ROS 成功。测试结束回收本任务 worker，保留部署文件及证据。


板端 ROS 快速验证可复用 `deployment/inference/build_ros_ohos.sh` 构建接口与 adapter，
用 `deployment/inference/tests/run_houmo_ros_board.sh` 和
`verify_houmo_ros_board.py` 执行真实 service 请求。构建前设置最终板端 install prefix，
部署在应用私有 overlay；沿用板端已验证 FastDDS 环境。当前 RoboPi 系统 CycloneDDS
缺少 `mq_unlink`，不要作为该环境的默认 RMW。验证需同时覆盖候选对照、header、
健康诊断、过期输入和 worker 离线；脚本启动的进程由脚本回收。
目标诊断消息 byte 字段可能以 `bytes` 返回，检查 OK 时兼容 `b'\x00'` 和整数 0。
2026-11-11 YOPO 已在 Python 3.12 / Humble / TCIM 1.5.0 环境完成上述真实 NPU ROS
快速检查；这不代表其他模型、其他板端版本或传感器闭环已验证。


YOPO 已将 CUDA/HMM 的前后处理和请求组装收敛为模型目录的 `processing.py`、
`predictor.py`，板端打包必须同时携带它们。后端 close、命名张量和计时键与 Torch
保持相同接口；共用层修改后用板端 ROS 矩阵回归，不能仅在宿主假后端测试通过。
