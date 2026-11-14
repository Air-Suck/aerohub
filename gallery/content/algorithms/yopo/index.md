---
title: YOPO
summary: 以深度图、自身状态和目标为输入，一次网络推理预测多条候选运动原语及评分，选择并重建轨迹，实现未知环境中的滚动避障导航。
keywords: [深度视觉导航, 学习型规划, 运动原语, 候选代价预测, 滚动避障]
category: planner
source: https://github.com/TJU-Aerial-Robotics/YOPO
revision: 'Aerohub 9dffa3e; upstream bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132'
demo:
  image: content/algorithms/yopo/assets/demo-cover.jpg
  alt: YOPO 作者森林实飞演示中的机载视角，点击观看官方视频
  caption: 作者公开的森林实飞素材，非 Aerohub 仿真录像；点击观看官方实飞演示。
  source: https://github.com/TJU-Aerial-Robotics/YOPO/blob/bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132/docs/realworld_1.gif
  video: https://www.bilibili.com/video/BV1jBpve5EkP/
showcase:
  methodSources:
    - label: 运动原语与候选预测示意
      url: https://github.com/TJU-Aerial-Robotics/YOPO/blob/bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132/README.md#introduction
  headline: 一次推理，多条候选；边飞边选下一段路径
  highlights:
    - title: 同时预测运动与代价
      text: 深度图、运动状态和相对目标共同输入网络，一次得到多个候选终端状态与评分。
    - title: 选优重建，滚动避障
      text: 选择低代价候选，重建可执行轨迹；随新观测重复规划，不把网络输出直接当作控制量。
videos:
  - label: YouTube · 森林实飞
    url: https://www.youtube.com/watch?v=LHvtbKmTwvE
  - label: Bilibili · 方法与演示
    url: https://www.bilibili.com/video/BV15M4m1d7j5/
  - label: YouTube · 方法与演示
    url: https://www.youtube.com/watch?v=m7u1MYIuIn4
media:
  - title: 森林实飞
    image: content/algorithms/yopo/assets/forest-flight-poster.webp
    animation: content/algorithms/yopo/assets/forest-flight.webp
    alt: YOPO 机载视角在密集树木间穿行
    caption: 在密集树木间连续穿行，以机载视角观察未知障碍不断进入视野的导航场景。
    source: https://github.com/TJU-Aerial-Robotics/YOPO/blob/bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132/docs/realworld_2.gif
  - title: 深度视觉平台
    image: content/algorithms/yopo/assets/platform-poster.webp
    animation: content/algorithms/yopo/assets/platform.webp
    alt: YOPO 飞行平台旋转展示，顶部为双目深度相机
    caption: 双目深度相机提供环境输入，机体与传感器配置展示视觉导航的硬件组成。
    source: https://github.com/TJU-Aerial-Robotics/YOPO/blob/bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132/docs/platform.gif
  - title: 随机场景导航
    image: content/algorithms/yopo/assets/random-scenes-poster.webp
    animation: content/algorithms/yopo/assets/random-scenes.webp
    alt: 随机森林与三维 Perlin 场景的点云及导航画面
    caption: 随机森林与三维 Perlin 场景并列展示，观察不同障碍结构下的导航行为。
    source: https://github.com/TJU-Aerial-Robotics/YOPO/blob/bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132/docs/new_env.gif
  - title: 交互目标导航
    image: content/algorithms/yopo/assets/goal-navigation-poster.webp
    animation: content/algorithms/yopo/assets/goal-navigation.webp
    alt: RViz 选取目标后无人机在模拟森林中导航
    caption: 在 RViz 点选目标，观察机载画面、候选轨迹与飞行响应如何衔接。
    source: https://github.com/TJU-Aerial-Robotics/YOPO/blob/bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132/docs/click_in_rviz.gif
integration: integrated
validation: unverified
---

## 算法核心

<div class="method-cards" aria-label="YOPO 上游方法示意">
<figure class="method-card">
<div class="method-card-image">

![上游运动原语示意：蓝色轨迹向前覆盖多个候选方向](./assets/primitive_trajectories.webp)

</div>
<figcaption><h3>运动原语：覆盖多个方向</h3><p>预设运动原语作为候选锚点，覆盖前方搜索空间，为不同绕障方向保留选择。</p></figcaption>
</figure>
<figure class="method-card">
<div class="method-card-image">

![上游网络预测示意：多条候选轨迹按预测代价呈现不同颜色](./assets/predicted_trajectories.webp)

</div>
<figcaption><h3>候选预测：选优后重建</h3><p>网络预测候选终端状态与代价；后续选取低代价候选并重建轨迹，彩色候选不会全部执行。</p></figcaption>
</figure>
</div>

<div class="diagram-scroll" tabindex="0" role="region" aria-label="YOPO 算法核心图，可横向滚动">

![YOPO：输入深度图、自身状态与相对目标，经网络预测、选优与重建输出局部轨迹；红色标明输入输出，按从左到右阅读](./assets/algorithm-io.svg)

</div>

## 在 Aerohub 中如何结合

<div class="diagram-scroll" tabindex="0" role="region" aria-label="YOPO 三层架构图，可横向滚动">

![YOPO 在 Tool、Behavior、Function 三层中的能力及调用关系](./assets/layers.svg)

</div>

- **Tool**：选择候选、重建轨迹并协调滚动更新；是 Navigation Tool 的实现基础。
- **Behavior**：复用时间化路径跟随；起飞与降落另行调用原生基础 Behavior。
- **Function**：YOPO 网络通过 Inference 提供候选预测；复用深度、自身状态和运动控制，不新增对外地图。

单机 `drone0` 的规划、推理与执行由算力侧提供，工作站下达任务目标。通用 Tool / Mission / Agent 尚未实现。

## 案例

<!-- 按展示约定保留案例位置，内容暂留空。 -->

## 补充信息

<details>
<summary>实现、部署与验证范围</summary>

### 输入、输出与进程边界

| 模块 / 层 | 输入 → 输出 | 实现与职责 |
| --- | --- | --- |
| YOPO Planner / Tool | 深度、定位、目标、候选 → 时间化轨迹 | `src/tool/navigation/planner/yopo/ros2_adapter/`；选优、ENU 变换、五次重建、20 ms yaw 计划与确认后的时间线 |
| Inference / Function | 视轴深度、对齐系速度/加速度/相对目标 → 全部候选及 cost | `src/basic_function/inference/models/yopo/`；ROS Adapter → 内部 FastAPI → 独立模型 worker，不维护任务目标、不生成控制量 |
| FollowTimedPath / Behavior | 时间化 PVA/yaw → Motion Reference | `src/behavior/follow_path/` 公共扩展；首段 Action，后续 modify，保留连续性校验、反馈与停止 |
| 状态、相机与控制 / Function | 定位/TF/深度 → 规划输入；运动参考 → 平台控制 | 复用 AS2 和平台输入；相机几何须独立核验，网络隐式处理环境不等于输出地图 |

算法源码仍归 `src/tool/navigation/planner/yopo/upstream/`，锁定官方 YOPO-Simple 提交 `bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132`，不是 YOPO-MINCO 分支。模型环境为 Python 3.8、PyTorch 2.4.1 + cu118；权重为 `epoch50.pth`，SHA256 `09ec31094ed83e09702efc1facf18076564ebbe1afc164960c41036e16ba229f`。模型和 Planner 不共享 Python 环境，Planner 不导入 PyTorch。

### 时间、坐标与执行

采用上游已有的 `plan_from_reference=True`，不宣称与原 Demo 默认 measured 模式等价。图像采集时间与未来规划参考时间分别保留；相机对齐系中的候选转换到车辆固定 ENU，单机客户端避免机群客户端的重复 NED → ENU 转换。

默认交接为 now + 0.2 s，推理超时 0.1 s，发送至少预留 0.05 s。仅一个推理及一个未确认执行更新在途；接续点从已确认曲线计算，等待生效 revision 反馈后再更新。拒绝、迟到、旧目标响应不会替换旧时间线，也不事后平移时间戳。连续失败、覆盖不足、定位失效或时钟回退触发停止。

相机使用 AirSim DepthPlanar 视轴深度，320 × 185、水平 FOV 90°，匹配训练约 90° × 60° 几何后缩放至 160 × 96。核对 CameraInfo、光学 TF 和同刻状态；不能靠 resize 或修改 frame_id 适配距离深度。原 ROS 的 20 m 截断、uint8 量化、inpaint 和状态归一化在 worker 内保留，避免重复归一化。

### 部署与调用

当前四角色在单台 NVIDIA 主机共置：AirSim 对应仿真器，PX4 SITL 对应飞控板，Planner / Inference Adapter / GPU worker / AS2 对应算力角色，ROS 任务客户端对应工作站。共置结果不是 RoboPi 板端或跨物理设备验收。

任务脚本负责原生起飞、目标与取消、降落恢复及解除武装确认。到达条件保留实际位置距目标小于 5 m；Planner 停止规划并确认停止后，要求速度低于 0.2 m/s 连续 1 s（ROS 时间），最长等待 10 s（墙钟），随后由脚本调用 Land。停止失败报告 FAULT 并进入任务恢复，不宣称精确停在目标点。

共享镜像保持不变，以外部 overlay 补编译。AS2 checkout 与 gitlink 存在差异，构建使用稳定 onboard 镜像匹配的库，未擅自切换子模块。组件卡片为 `cards/planner/yopo-single.yaml` 与 `cards/inference/yopo.yaml`。模型准备、构建和运行入口见下方源码说明。

### 已记录的验证

以下摘自 2026-11-10 验收文档，本次建档没有重新运行仿真。

| 范围 | 已记录结果 | 适用限制 |
| --- | --- | --- |
| 数值与接口 | 原入口预处理、全部候选/评分对照通过（atol=1e-5、rtol=1e-4）；五次 PVA、yaw、未来交接及确认/拒绝等通过 | 不单独证明飞行动力学 |
| 跨容器 GPU 服务 | 工作站 ROS → onboard Adapter → GPU worker，2 m/s 配置 10/10；RTT 中位 18.38 ms、最大 43.48 ms | RTX 4090；合成图像，不是实机板端性能 |
| AirSimNH + PX4 SITL 短程 | 开放方向与森林短程各 3/3；进入 5 m 门槛、停止、原生 Land / disarmed，任务检查未发现碰撞 | 2 m/s 速度配置、5 s 候选时长与 Demo 局部控制增益 |
| 故障收口 | 人工取消、陈旧深度、GPU worker 退出各 1/1，停止及降落解除武装通过 | 真实 /clock 回退动力学未验收 |
| 有屏森林长距离 | 水平约 234.79 m，单轮通过；347 revisions，导航 122.66 s，速度中位 1.992 m/s、峰值 2.309 m/s | 未做长距离三次重复；不是任意障碍保证 |

长距离停止耗时 5.26 s、低速保持 1.020 s；飞行/停止阶段碰撞检查为 false，降落后单独查询记录了 `Landscape_0` 地形接触（原始 has_collided=true、penetration_depth=0）。不能写成所有阶段原始碰撞标志均为 false。

原始逐轮日志、深度和曲线保存在本地 `log/yopo-single/`，尚未整理成读者可访问的公开原始证据，因此页面元数据仍为 `unverified`。这与“代码已接入”分别表达。未完成真机、高速、精确到点或长期可靠性验收；旧失败尝试与恢复记录见验收文档。

### 来源与素材

- [Planner 说明](https://github.com/Air-Suck/aerohub/blob/9dffa3e/src/tool/navigation/planner/yopo/README.md)
- [模型准备与数值基线](https://github.com/Air-Suck/aerohub/blob/9dffa3e/src/basic_function/inference/models/yopo/README.md)
- [Demo 与四角色运行入口](https://github.com/Air-Suck/aerohub/blob/9dffa3e/deployment/airsim/demos/yopo_single/README.md)
- [分阶段验收与长距离单轮记录](https://github.com/Air-Suck/aerohub/blob/9dffa3e/deployment/airsim/demos/yopo_single/validation.md)

封面取锁定官方仓库 `docs/realworld_1.gif` 的首帧，转为轻量 JPEG，原始宽高 360 × 203；只用于链接作者实飞视频，不是 Aerohub 运行证据。来源为页面顶部的官方素材链接。上游采用 [MIT 许可证](https://github.com/TJU-Aerial-Robotics/YOPO/blob/bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132/LICENSE)，版权 © 2024 TJU-Aerial-Robotics, Tianjin University, China；随本条目保留完整许可于 `assets/LICENSE.txt`。

</details>
