---
title: FAST-LIVO2
summary: 融合激光雷达、IMU 与图像的实时定位建图系统。FAST-LIVO2 在运动中估计自身位姿，并输出三维配准点云，为机器人定位和环境重建提供基础能力。
keywords: [激光惯性视觉融合, 直接法, 顺序滤波更新, 统一体素地图, 定位建图]
category: state_estimator
source: https://github.com/hku-mars/FAST-LIVO2
revision: 0d2c0346107b75b59934975adec9a6eeeb913c64
demo:
  image: content/algorithms/fast_livo2/assets/demo-cover.jpg
  alt: FAST-LIVO2 作者视频中的三维重建与飞行平台封面，点击观看官方视频
  caption: 作者公开的定位与建图演示；不代表本项目已完成真实数据或飞行验收。
  source: https://github.com/hku-mars/FAST-LIVO2#11-related-video
  video: https://www.youtube.com/watch?v=6dF2DzgbtlY
integration: integrated
validation: unverified
---

FAST-LIVO2 面向机器人运动中的实时定位与三维重建，将激光雷达、IMU 与图像融合到同一状态估计中。算法采用顺序滤波更新：激光直接配准提供几何约束，图像直接对齐提供光度约束，两者复用统一体素地图中的几何与视觉信息。

## 算法核心

<div class="diagram-scroll" style="--diagram-min-width: 1120px" tabindex="0" role="region" aria-label="FAST-LIVO2 算法核心图，可横向滚动">

![FAST-LIVO2：IMU 传播、激光几何更新与视觉光度更新，共享体素地图，输出自身状态与配准点云；红色表示整体输入输出](./assets/algorithm-unified.svg)

</div>

算法方法依据[作者论文](https://arxiv.org/abs/2408.14035)：地图中的激光平面先验辅助图像对齐，参考图像块随新图像对齐更新。本图展示完整 LIVO 模式。图中反馈表示跨时刻的地图复用，不代表两个独立定位器。内部统一地图与本项目对外发布的配准点云应分别理解。

## 在 Aerohub 中如何接入

<div class="diagram-scroll" style="--diagram-min-width: 1120px" tabindex="0" role="region" aria-label="FAST-LIVO2 三层架构图，可横向滚动">

![FAST-LIVO2 在 Tool、Behavior、Function 三层中的能力与数据依赖](./assets/layers-capabilities-unified.svg)

</div>

- **Tool**：本接入未注册任务 Tool，状态和点云可由上层直接读取。
- **Behavior**：无直接运动 Action 调用，不将定位输出包装成飞行动作。
- **Function**：一个联合算法实例，经共享输出桥分别接入 State Estimator 与 Perception；两路能力共用算法和部署依赖。

联合算法只保留一个实例。配准点云不自动等于占据图或 ESDF，AS2 接入也不代表 PX4 已融合外部定位。

[Bilibili 备用观看入口](https://www.bilibili.com/video/BV1Ezxge7EEi)

## 案例

<!-- 暂留空，等待可公开核查的案例材料。 -->

## 补充信息

<details>
<summary>接入细节与验证记录</summary>

Demo 画面来自[作者公开素材](https://i.ytimg.com/vi/6dF2DzgbtlY/maxresdefault.jpg)，保留未修改的本地副本，用于链接官方演示；素材来源见上方作者仓库，演示效果不计入本项目验收。

### 三层接入关系


本项目的实际覆盖集中在 **Function 层**。图中保留 Tool 与 Behavior 两层并说明未绑定，避免将输出被上层读取误写成算法已经提供了任务工具或运动执行。联合算法只保留一个实例，状态与点云两路输出共享算法和部署依赖。

| 架构层 | 实际接入 | 边界 |
| --- | --- | --- |
| Tool | 未注册任务 Tool | 数据可供上层订阅，不宣称已接入某个 Navigation Tool |
| Behavior | 无直接运动 Action 调用 | 状态发布不具有运动 Behavior 生命周期 |
| Function：State Estimator | FAST-LIVO2 输出桥与 AS2 插件 | 适配自身位姿、速度及 TF，保留统一状态接口 |
| Function：Perception | 共享桥的配准点云输出 | 保留三维点云，不自动产生占据图或 ESDF |
| Function：传感器输入 | ROS 1 驱动、回放或专用 AirSim RPC 输入桥 | AirSim 已有 shadow 接入记录，精度门限未通过；传感器模型和标定仍按部署核查 |

### Function：一个算法进程，两类能力

FAST-LIVO2 保持官方 ROS 1 Noetic 算法进程，通过只读输出桥连接 ROS 2 Humble。位姿与速度回答“自身在哪里、如何运动”，配准点云回答“周围有什么”；职责分开，不要求把联合算法拆为两个独立估计器。

| 模块 | 输入 → 输出 | 实现与消费者 |
| --- | --- | --- |
| 联合算法 | IMU、图像、所选激光消息 → 状态与配准点云 | `src/state_estimator/fast_livo2/upstream/`，保留一份算法源码 |
| 输出桥 | ROS 1 状态/点云 → ROS 2 数据 | `src/state_estimator/fast_livo2/ros1_proxy/` 与 `plugin/`；保留测量时间和点云原生字段 |
| AS2 State Estimator 插件 | 算法里程计、安装外参与世界对齐 → 自身位姿、速度、TF | `src/state_estimator/fast_livo2/plugin/`；统一输出供控制、Behavior 与上层消费者读取 |
| Perception 接口 | 配准点云 → namespaced 环境点云 | `src/perception/fast_livo2/` 描述接口，与状态接入共享算法和依赖；不是第二个独立算法进程 |

### 状态、点云接口与共享部署

本组件**无直接运动 Behavior 调用**，不创建 Planner 或运动 Behavior。状态经 `/<namespace>/self_localization/pose`、`self_localization/twist` 与 TF 提供；点云经 `/<namespace>/perception/fast_livo2/registered_points` 提供。

同一架飞机的主状态与 TF 应有明确发布权；示例不自动替换多机现有定位源。AS2 消费 FAST-LIVO2 状态，不代表 PX4 已融合外部定位。配准点云也不等于占据图、ESDF 或可直接消费的完整导航地图。

状态与环境接口共用算法、ROS 1 master 及输出桥；仅停止某一路消费者不等于停止算法，重置算法时需处理两路消费者的局部世界变化。核查依据为 `src/state_estimator/fast_livo2/README.md`、`plugin/src/plugin.cpp`、`plugin/scripts/bridge.py` 以及 `src/perception/fast_livo2/README.md`。图中没有把它们拆成两个可独立启停的估计器。

### 已验证范围与接入缺口

组件 README 记载 Docker 编译、两套 AS2 宿主插件契约及合成三传感器端到端冒烟通过。后续有屏 AirSim + PX4 SITL shadow 测试已记录真实仿真 IMU、RGB 和 LiDAR 进入算法，ROS 2 状态与配准点云有下游消费；但 LIO 与完整 LIVO 的空中／水平误差超过预声明的 `0.5 m` 门限，并出现航向跳变。因此定位精度未通过，authority 状态源测试受安全门约束未执行。

AirSim 输入桥现已存在：`ros1_proxy/airsim_input_bridge.py` 将 RPC 测量发布为 ROS 1 输入。AirSim 1.8.1 缺少逐点时间，桥使用 `time=0`，配合构建补丁 `preprocess/instantaneous_scan=true` 按瞬时扫描处理；不能把补齐字段理解为恢复了真实旋转扫描时间。输出桥仍只负责算法结果方向，不是通用双向传感器桥。

上述结果来自当前组件 README 的历史记录，其引用证据目录为 `log/main-gpu-e2e-20261109/`；本工作区没有该证据包，本次未重新运行或独立复核。真实数据集、权威仿真飞行和真机验收仍未通过，元数据保留 `unverified`。

### 本次核查与来源

本次仅更新 Gallery，读取了状态估计与环境接口 README、`plugin/src/plugin.cpp` 的 `receive()`、`ros1_proxy/output_bridge.py` 的状态／点云订阅以及 AirSim 输入桥。锁定上游 gitlink 为 `0d2c0346107b75b59934975adec9a6eeeb913c64`，本工作区上游子模块未检出；算法核心图按公开方法绘制，并链接作者论文供对照；本次在线获取源码超时，论文页面也未成功取得，锁定版本的方法细节仍待逐函数核查。

- [状态估计组件说明](https://github.com/Air-Suck/aerohub/blob/main/src/state_estimator/fast_livo2/README.md)
- [环境输出接口说明](https://github.com/Air-Suck/aerohub/blob/main/src/perception/fast_livo2/README.md)
- [作者论文：FAST-LIVO2](https://arxiv.org/abs/2408.14035)

以上仓库链接用于定位文档；本页同步的是 2026-11-12 本地工作区内容，不将可变的 main 链接当成公开验收证据。单机接口示例不证明多机共享地图、全局对齐或协同定位已实现；多机部署需要每机独立实例、namespace 与状态发布权配置。

</details>
