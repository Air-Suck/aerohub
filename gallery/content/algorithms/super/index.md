---
title: SUPER
summary: 面向未知、障碍密集环境的无人机自主导航。SUPER 将局部建图与轨迹规划结合，随新观测滚动更新飞行路径，适用于点到点导航和避障。
keywords: [未知环境导航, 模块化规划, ROG-Map, 探索与安全备份, 滚动重规划]
category: planner
source: https://github.com/hku-mars/SUPER
demo:
  image: content/algorithms/super/assets/demo-cover.jpg
  alt: SUPER 作者官方导航演示视频封面，点击观看完整 Demo
  caption: 上游作者公开的导航演示，展示原项目效果；不代表 Aerohub 已达到相同效果。
  source: https://github.com/hku-mars/SUPER#11-autonomous-navigation-in-challenging-environments
  video: https://www.bilibili.com/video/BV1BSFgeJEJn/
showcase:
  sourceNote: 自主探索片段：Kong 等人与 STAR Lab 合作工作。
  headline: 向未知环境探索，为下一步保留安全退路
  highlights:
    - title: 探索轨迹 + 安全备份
      text: 主轨迹向目标推进，备份轨迹保留可衔接的退路；两者在同一规划过程中配合。
    - title: 随观测滚动重规划
      text: LiDAR 更新局部地图，规划器据此调整路径与轨迹，持续应对新出现的障碍。
videos:
  - label: YouTube
    url: https://www.youtube.com/watch?v=GPHuzG0ANmI
media:
  - title: 自主探索
    image: content/algorithms/super/assets/exploration-poster.webp
    animation: content/algorithms/super/assets/exploration.webp
    alt: 机载视角穿过建筑与植被，展示 SUPER 的自主探索应用
    caption: 将局部导航用于更大范围的探索，随新观测持续规划穿行路径。
    source: https://github.com/hku-mars/SUPER/blob/3bfa4033361bbdc7cd814c94aff826e6e1679018/misc/exp.gif
  - title: 目标跟踪
    image: content/algorithms/super/assets/tracking-poster.webp
    animation: content/algorithms/super/assets/tracking.webp
    alt: 树林中的目标跟踪画面与局部环境可视化
    caption: 目标持续移动时，随目标位置和周围障碍更新飞行路径。
    source: https://github.com/hku-mars/SUPER/blob/3bfa4033361bbdc7cd814c94aff826e6e1679018/misc/tracking.gif
integration: integrated
validation: unverified
---

## 算法核心

<div class="diagram-scroll" style="--diagram-min-width: 1120px" tabindex="0" role="region" aria-label="SUPER 算法核心图，可横向滚动">

![SUPER：输入目标航点、LiDAR 点云与自身状态，经探索和安全备份规划输出时间化轨迹；红色标明输入输出，按从左到右阅读](./assets/algorithm-unified.svg)

</div>

## 在 Aerohub 中如何结合

<div class="diagram-scroll" tabindex="0" role="region" aria-label="SUPER 三层架构图，可横向滚动">

![SUPER 在 Tool、Behavior、Function 三层中的能力及调用关系](./assets/layers.svg)

</div>

- **Tool**：局部导航与滚动规划，将提交轨迹交给基础执行能力；是 Navigation Tool 的实现基础。
- **Behavior**：复用时间化路径跟随，保持轨迹时间与滚动接续语义。
- **Function**：提供 ROG-Map 局部环境表示，复用自身状态与运动控制。

三层图表达单机能力及依赖，不表达进程拆分。当前 `four-part` 入口已接通工作站任务调用算力侧规划与执行；通用 Tool 尚未实现。

## 案例

<!-- 按展示约定保留案例位置，内容暂留空。 -->

## 补充信息

<details>
<summary>接入细节与验证记录</summary>

Demo 封面来自[作者官方视频的公开封面](https://i.ytimg.com/vi/GPHuzG0ANmI/maxresdefault.jpg)，以未修改的本地副本随站点发布，用于链接作者原演示；来源和原演示链接保留在页面中。

### 三层职责与源码

| 架构层 | 涉及的能力 | 实现或依赖关系 |
| --- | --- | --- |
| Tool | SUPER 规划、ROS 2 bridge、公共 FollowPath Client | 原生曲线适配和执行协调，构成 Navigation Tool 实现基础 |
| Behavior | 扩展 FollowPath | 复用公共执行能力；Demo 另行编排起飞、停止与降落 |
| Function | 导航输入适配、ROG-Map | 已接入点云/里程计对齐及算法自带环境表示，ROG-Map 与 SUPER 规划同进程 |
| Function | State Estimator、Motion Controller | 复用平台现有状态与 AS2 控制，没有新建定位算法 |

### Tool：原生规划与时间化适配

| 模块 | 输入 → 输出 | 实现与职责 |
| --- | --- | --- |
| 导航输入 | 位姿、点云 → 规划所需导航输入 | `src/perception/navigation_inputs/`；环境输入与自身定位职责分开 |
| 原生 SUPER | 导航输入、目标 → 已提交的多项式轨迹 | 当前构建使用主仓库 `src/planner/super/ros2_adapter/native/`；`upstream/` 保留为参考，来源记录见 native 内的 `SOURCE.json` |
| ROS 2 bridge | committed polynomial → `TimedTrajectorySegment` → Action 调用 | `src/planner/super/ros2_adapter/bridge/uav_super_adapter/node.py`；保留原生曲线与执行时间，借助公共 `FleetFollowPathClient` 管理滚动更新 |

接入时参考的派生仓库为 [Popc0rn7/SUPER-sim](https://github.com/Popc0rn7/SUPER-sim)，作者官方演示来自 `hku-mars/SUPER`。此页记录的实现版本以集成仓库 `ca3581e` 为依据，不能用参考子模块的版本替代实际 native 构建来源。当前展示为单机 `drone0`，不推导出多机规划或同步能力。

### Behavior：扩展 FollowPath

Adapter 通过 `/<vehicle>/follow_timed_path` 调用 `uav_planner_interfaces/action/FollowTimedPath`，由扩展 FollowPath 在 Behavior 内采样，再进入 Motion Reference → AS2 控制器/平台 → PX4。轨迹包含位置、速度与加速度，原生多项式用于核对执行参考。

起飞、目标下发、到点保持、停止确认和降落解除解锁由 Demo 任务编排负责，不能把这些职责写成 SUPER 求解器自身的能力。滚动更新须等待初始执行参考，Action 接受不等于已经产生运动参考。

### Function：导航输入与 ROG-Map

`as2_navigation_inputs` 是独立 ROS 2 节点，将平台 Odometry 和传感器点云变换到统一局部世界系，输出 `/<vehicle>/navigation/odom`、`navigation/points`。该节点保留输入时间，并按点云时刻查询 TF，不发布自身 TF，也不把同源平台里程计自动升级为融合定位。

SUPER 内部 ROG-Map 消费对齐点云和对应时刻的 TF，维护环境占据表示，供规划使用。规划与地图使用同一局部参考系。三层图的虚线表示数据依赖；ROG-Map 与规划同进程，导航输入节点独立运行，能力分层不表示进程拆分。

AS2 State Estimator 继续提供现有状态及 TF，Motion Controller 直接消费参考与自身状态。Adapter 还读取 `sensor_measurements/odom` 监测位姿时效。传感器、TF 与时钟是启动依赖；仿真时间回退不能直接沿用旧地图。平台边界仍为 AS2 平台适配与 PX4。

核查入口：`src/perception/navigation_inputs/README.md`、native 中的 `super_planner/AS2_INPUTS.md` 与 `config/as2_inputs_ros2.yaml`，以及 bridge 中 `SuperAdapter` 的订阅与公共 Client 调用。这里描述接入关系，不扩大已有地图精度或飞行验收结论。

### 四方部署目标与历史拓扑

目标为工作站经 ROS 调用就绪的 RoboPi 能力；SUPER、Adapter 和 AS2 执行链在算力板，高频控制由板内产生与消费。AirSim 提供环境与模拟传感器，PX4 SITL 对应飞控板。工作站负责交互、任务编排及仿真验收。

历史 `robopi-all` 把 `mission.py` 也放在 RoboPi，脚本同时启动算法、编排任务和查询碰撞真值。当前已提供 `four-part` 入口，将任务放在工作站、算法与执行放在算力侧；`robopi-all` 保留为历史拓扑。入口接通不代表通用 Mission/Tool 已实现，也不扩大下方历史短程结果的验收范围。当前运行边界以 `deployment/airsim/demos/super_forest/README.md` 为准。

### 已记录的短程结果

2026-11-09 主文档记录一次独立 4 米短程仿真闭环：真实 SUPER 规划、板端滚动更新与执行、停止确认、降落解除解锁。到点误差约 0.205 米，执行至 revision 28，144 条运动参考均找到原生曲线候选。

该结果只适用于当次短程用例，不替代 forest8 长任务、异常恢复、长期可靠性或真机验收；时钟持续同步可靠性仍未确认。本次没有重新运行仿真。页面保留主文档对历史结果的描述，原始运行证据未在此整理为公开证据，验证记录仍为“未登记验证证据”。

内容来源：[项目简介](https://github.com/Air-Suck/aerohub/blob/ca3581e6c8a1fddce4c521736e2959af81780333/docs/项目简介.md)，同步版本 `ca3581e`。源码目录用于定位实现，不由网页构建直接加载。

</details>
