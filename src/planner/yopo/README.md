# YOPO 单机 Planner

源码为官方 [YOPO](https://github.com/TJU-Aerial-Robotics/YOPO) 子模块，提交 `bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132`，MIT。网络与权重仅由 [Inference 模块](../../inference/models/yopo/README.md) 引用，Planner 环境不导入 PyTorch。

当前实现 `plan_from_reference=True` 分支：地面站 ROS 目标 → 算力侧深度/状态快照 → typed InferYopo → 最小预测 cost 候选 → 固定车辆 ENU 中的五次 PVA/yaw → 现有 FollowTimedPath。原 ROS Demo 默认 measured 分支不在本轮等价范围；未新增 YopoBehavior，未启动原 PositionCommand timer。任务起飞/降落独立调用原生 Behavior。

## 接口与时间线

`ros2_adapter/` 提供 `uav_yopo_planner` 包、`yopo_planner` 可执行文件。相对于车辆命名空间：

| 接口 | 类型 | 语义 |
| --- | --- | --- |
| `yopo/goal` | PoseStamped | 固定 `<vehicle>/odom` ENU 目标，完整 XYZ |
| `self_localization/pose` | PoseStamped | 自身状态，保留测量时刻；父坐标通过该时刻 TF 转到固定 ENU |
| `self_localization/twist` | TwistStamped | 相同参考点线速度；body速度使用严格同stamp的权威Pose旋转，固定父系才查TF，拒绝未知参考点 |
| `yopo/depth`、`yopo/camera_info` | Image、CameraInfo | 视轴深度、内参；采集 stamp，不以 now 替换 |
| `inference/yopo/infer` | InferYopo Service | 推理，仅候选相对 PVA/cost，无控制量 |
| `follow_timed_path` | FollowTimedPath Action | 首段提交 |
| `follow_timed_path/_behavior/modify` | Action SendGoal Service | 后续更新，不经过机群 NED→ENU 转换 |
| `yopo/cancel` | Trigger | 异步请求停止；成功返回只代表发起停止，必须等 status |
| `yopo/status` | JSON String | state/reason、task_id、ready、接受 revision、生效 revision |

`InferYopo.observation_header` 是实际位姿观测时间与来源固定系，`depth.header` 是相机采集时间；两者默认相差最多 50 ms。`state_header` 是规划参考起点时间及相机对齐的 x前/y左/z上系，允许未来最多 300 ms；服务不查 TF。`yopo_aligned` 是每个request_id上下文中冻结的相机对齐逻辑系，旋转取图像配对姿态，原点取未来参考位置；不发布或查询未来机体TF。响应原样保留这两个 header、request_id、模型权重哈希和 device。无效深度归零（正无穷截为20 m），在 worker 按原 ROS uint8 量化/inpaint 一次；速度/加速度/目标仅在 worker normalize 一次。

规划默认在 now+0.2 s 交接，推理墙钟超时0.1 s，发送至少留0.05 s。固定快照包含观测、旋转、参考 PVA/yaw、目标世代和请求ID。一次只保留一个真实未结束的推理，忙时深度覆盖最新值；新图像等待同时间定位期间可继续旧计划，持续失配才停止。body Twist可先于同stamp Pose到达，使用有界队列等待；不依赖冗余动态TF的到达时延。过期响应不会提交。一次只允许一个未确认执行请求；拒绝保留旧参考，不后移原时间戳。执行应答超时存在接纳歧义时直接停止收口。

接续点从最后**确认接受**的曲线在未来交接时刻求 PVA/yaw。时间线仅在 Action/modify 确认后替换；首次和每次更新均等待匹配 task/revision 的 elapsed>0.12 s、非 waiting 反馈以及控制器 TRAJECTORY 模式后，才允许下一更新。此等待覆盖原控制模式切换阻塞窗口；不是对控制参考已实际发送的独立证据。候选位置是相对参考起点的位移，终端速度/加速度是对齐系中的向量，旋转后不再叠加起点v/a。PVA 由同一五次曲线求导，yaw 按原策略每20 ms 生成，执行端仍使用其现有 quintic/yaw 插值和连续性门槛。

到达条件保留实际位置距目标<5 m。使所有旧请求失效、调用停止；等待服务确认、控制器 HOVER/POSITION、状态新鲜且速度<0.2 m/s 连续1 s（ROS时间），最长10 s（墙钟）。STOPPED 表示该门控完成；FAULT 必须由任务脚本降落恢复。目标更换先完成相同停止门控再启动新任务。连续三次失败、覆盖不足、状态/输入失效、时钟回退都请求停止。推理或提交拒绝本身不覆盖旧计划。

## 相机契约

训练 FOV=90°×60°，网络尺寸160×96。示例采用 AirSim **DepthPlanar / ImageType=1**、320×185、水平90°，其针孔垂直视场约60.07°，再 nearest resize；这不是用 resize 修正视场。DepthPerspective / ImageType=2 距离不能改 frame_id 冒充视轴深度。

首期要求相机与车辆原点共置，仅配置 pitch；必须有真实静态 TF，且 `R_body_optical = R_body_aligned * [[0,0,1],[-1,0,0],[0,-1,0]]`，平移为零。Planner 校验 TF、内参、尺寸、无畸变和视场（1°容差），未验证 `depth_model` 默认拒绝任务。非零杆臂或其他旋转需要明确扩展输入几何，不用占位参数掩盖。

## 构建、运行与证据

操作入口和四方部署见 [单机 Demo](../../../deployment/airsim/demos/yopo_single/README.md)。AS2 checkout 与 gitlink 当前不同，构建复用稳定 onboard 镜像自身头文件/库，不切换用户 checkout；不要混入另一个 checkout 的旧插件。

数值复验：

```bash
CUDA_VISIBLE_DEVICES='' YOPO_DEVICE=cpu src/inference/models/yopo/.venv/bin/python src/inference/models/yopo/test_baseline.py
PYTHONPATH=src/planner/yopo/ros2_adapter src/inference/models/yopo/.venv/bin/python -m unittest discover -s src/planner/yopo/ros2_adapter/tests -v
```

验证分层以 Demo 中的验收记录为准；数值对照、ROS 传输、GPU 服务链都不能单独证明 AirSim 动力学通过。
