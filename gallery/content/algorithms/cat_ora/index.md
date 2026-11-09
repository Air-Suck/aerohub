---
title: CAT-ORA
summary: 集中式多机队形重排；lib 侧生成并协调时间化轨迹，runtime 侧逐机调用扩展的 AS2 FollowPath。
category: planner
source: https://github.com/ctu-mrs/catora
revision: 4bc41f593eebde65dd74108a97e80430a145dfa0
integration: integrated
validation: unverified
---

## 一个 lib 算法，两层视图

本条目把 CAT-ORA 的接入组织成 **lib 实现 + runtime Behavior 绑定**。这里的 lib 是算法展示分层，包含算法、代理、适配与机群协调；不表示这些实现已经合并成一个进程或编译成共享库。runtime 指实际承担执行的 AS2 Behavior。

![CAT-ORA 的 lib 实现、共享机群协调与四机 AS2 FollowPath runtime 绑定](./assets/layers.svg)

图示对应仓库的四机任务配置，表达调用关系与执行实例，不是飞行记录或仿真结果回放。

## Lib：算法与独立接入实现

四架飞机共享一次批量规划和一套机群调用协调，不为每架飞机复制整个 CAT-ORA。

| 模块 | 输入 → 输出 | 职责与实现位置 |
| --- | --- | --- |
| CAT-ORA 规划核心 | 初始构型、目标构型、速度与加速度约束 → 逐机轨迹采样点 | 上游服务 `/mrs_formation_reshaping/get_reshaping_trajectories`；源码位置 `src/planner/cat_ora/upstream/` |
| ROS 1 proxy | TCP/逐行 JSON → ROS 1 服务调用 → 轨迹数组 | `src/planner/cat_ora/ros1_proxy/ros1_proxy.py`，保留独立 ROS 1 进程与原生服务接口 |
| ROS 2 adapter | 四机位姿、目标和轨迹数组 → `TimedTrajectorySegment` | `src/planner/cat_ora/ros2_adapter/uav_cat_ora_adapter/node.py`，负责坐标、车辆归属、时间与统一消息封装 |
| 共享机群协调 | 经校验的机群轨迹段 → 每架飞机的 `FollowTimedPath.Goal` | `src/planner/framework/uav_planner_framework/follow_path_client.py`，维护 Action 接受、结果、更新、取消与机群失败收口 |

框架的 Registry、任务配置与轨迹校验也属于这个视图的 lib 侧，共用代码保留在 `src/planner/framework/`，不复制进 CAT-ORA。这里按职责拆分，并不把这些模块描述成四个独立可替换的公共 API。

本地 CAT-ORA 上游子模块尚未检出。规划核心在图中作为服务黑盒，内部求解步骤不作未经核查的展开；其输入输出来自代理实际使用的服务字段及本仓库适配代码。

## Runtime：实际调用哪些 AS2 Behavior

| 作用 | Behavior 与调用接口 | 谁调用、何时调用 |
| --- | --- | --- |
| 轨迹执行 | **扩展 FollowPath**：`ExtendedFollowPath` 继承原生 `FollowPathBehavior`；入口为 `uav_planner_interfaces/action/FollowTimedPath`，地址 `/<vehicle>/follow_timed_path` | 机群协调器在轨迹通过校验后逐机提交，接收各机结果；这是 CAT-ORA 默认执行链的 runtime 绑定 |
| 起飞前置 | 原生 **TakeoffBehavior**：`as2_msgs/action/Takeoff`，地址 `/<vehicle>/TakeoffBehavior` | 部署/测试入口通过 `deployment/platform/fleet_takeoff.py` 单独调用；CAT-ORA adapter 不负责发起起飞 |

`FollowTimedPath` 是本仓库扩展的时间化入口，和原生 FollowPath 共处同一个扩展组件，二者互斥；它们不是前后串行调用的两个 Behavior。实现见 `src/behavior/follow_path/src/follow_path_extended.cpp`。

启动文件还加载了 Land 与 GoTo，但这不构成 CAT-ORA 调用了它们的证据，因此不列为算法调用。`legacy_setpoint` 是显式选择的旧执行链，本页图示只描述默认的 `follow_path` 后端。

runtime 在 Behavior 内采样时间化轨迹，再输出 Motion Reference，由 AS2 控制器与平台执行到 PX4。lib 不绕过 Behavior 直接输出默认运动参考。

## 四机如何协同

示例配置为 `src/planner/framework/examples/cat_ora_four_px4.yaml`，部署映射来自同目录的 `deployment_four_px4.yaml`。

| 车辆 | runtime 执行实例 | Goal 坐标系 | 配置中的目标（机群 NED，米） |
| --- | --- | --- | --- |
| drone0 | `/drone0/follow_timed_path` | `drone0/odom` | `(-6, -4, -1.5)` |
| drone1 | `/drone1/follow_timed_path` | `drone1/odom` | `(-2, -4, -1.5)` |
| drone2 | `/drone2/follow_timed_path` | `drone2/odom` | `(2, -4, -1.5)` |
| drone3 | `/drone3/follow_timed_path` | `drone3/odom` | `(6, -4, -1.5)` |

这些是任务配置目标，不是测得的到达位置。`z=-1.5` 来自 NED 向下为正的约定；下发到各机 Behavior 前会转为该机本地 ENU。

统一测试脚本 `run_planner.sh` 实际会先起飞部署中的七架飞机，再选择 CAT-ORA 四机任务；图中只显示参与该规划任务的 drone0–drone3，不把部署总机数等同于算法参与机数。

多机执行的关键约束如下：

- **车辆绑定**：代理数组按任务的 `vehicles` 顺序解释；adapter 验证轨迹终点仍属于指定车辆。若上游换配违反固定“车辆 → 目标”契约，报告 `NO_SOLUTION`，不会静默换配执行。
- **共同执行时刻**：adapter 为整段轨迹设置同一个未来 `valid_from`，默认提前量为 3 秒；各机采样点保留 `time_from_start`。协调器逐机发送 Action，依靠共同时间基准执行，并不声称请求同时送达。
- **轨迹内容**：CAT-ORA 当前输出位置采样，默认间隔 0.2 秒，`valid_fields=0`；不将未声明的零速度、零加速度或 yaw 当作有效算法输出。长度较短的轨迹以自身终点补齐到共同结束时间。
- **机群收口**：某机拒绝、执行失败，或机群在共同开始时刻前未完成请求确认，协调器进入失败处理并向机群发起取消/停止；全机 Action 结果成功才满足协调器的 complete 条件。发送停止请求不等于已证明各机瞬时停止。
- **暂停语义**：时间化执行暂停后要求取消并重规划，不能在不处理共同时间基准的情况下直接续跑。

Manifest 声明 1–16 机是配置能力范围；本页四机展示不代表所有规模都已通过动力学验证。

## 核查依据与验证边界

本页依据本地源码与配置建立展示，没有启动 ROS、AS2、PX4 或重新运行 CAT-ORA。仓库主文档记载过四机无障碍 PX4/Gazebo 验收；本次未重验，也未把对应运行证据打包进 Gallery，因此页面的验证记录仍为“未登记验证证据”。这不是否定历史结果，而是限定本条目的证据范围。

| 核查内容 | 本地依据 |
| --- | --- |
| 上游来源与锁定版本 | `.gitmodules`、`src/planner/cat_ora/upstream` gitlink：`4bc41f593eebde65dd74108a97e80430a145dfa0` |
| AS2 基线 | `as2/aerostack2` gitlink：`19c397309c6a41b279e6b551818ec12605cfcecc`；当前工作树另有既存修改，本页未验证其动力学影响 |
| 服务字段与坐标/时间转换 | `ros1_proxy.py` 的 `generate()`；adapter 的 `_generate()`、`_message()` |
| 车辆对应、Action 名称、失败收口 | `FleetFollowPathClient` 与 `make_goal()` |
| Behavior 继承、互斥及 Motion Reference 输出 | `follow_path_extended.cpp`、`motion_behaviors.launch.py` |
| 起飞调用方及其与 Planner 的先后关系 | `deployment/platform/fleet_takeoff.py`、`deployment/integration/run_planner.sh` |
| 历史验收范围 | `docs/项目简介.md` 的 FollowPath 扩展与多机验收章节；不代表本次重跑 |

集成仓库核查基线：`cf6f6418184e0bccc3d174d3f87b96991735b777`，以上路径用于定位源码，不要求网页加载仓库外文件。展示资源全部位于本条目目录。
