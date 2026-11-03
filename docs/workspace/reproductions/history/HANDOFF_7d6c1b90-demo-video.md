# Planner Gazebo Demo 录制

## 当前任务

- CAT-ORA：16 台 PX4/Gazebo x500 从 4×4 方阵变为圆环，成品已完成。
- Swarm-Formation：7 台 PX4/Gazebo x500 穿越官方参数生成的随机障碍场，局部避障并在终点恢复六边形队形；官方风格动态编队连线版已完成。

## 已完成

- 官方 `normal_hexagon` 环境已移植到 Gazebo：`34×15×3 m`、60 根随机圆柱、20 个随机椭圆环、`0.1 m` 点云分辨率、`0.8 m` 最小柱间距。
- 上游生成器使用 `random_device`，没有唯一固定地图；录制固定 seed `31`。四象限柱数为 `18/14/13/15`，18 个等面积网格每格 `1–5` 根，3 根柱直接阻挡名义路径，不是走廊布局。
- 修复 `position_command_bridge.py`：删除 x/y/z 动态槽位裁剪和防倒退改形，向 MAVROS `PositionTarget` 原样发送 planner 的位置、速度、加速度和 yaw；非有限指令才保持当前位置。
- 修复低空穿越：全局点云加入 `z=0` 地面层，任务前保持高度从 `0.5 m` 提高到 `1.0 m`。
- 扩充验收指标：分别计算实际/命令最小机间距、障碍净空和最低高度，并记录最近实际事件的车辆和障碍编号。
- 为避免无 GPU 软件渲染影响七机控制，将相机改为 `640×360 @ 3 Hz`，并预计算圆环线段。
- 最新正式录制完整到达，终点中心 `(23.923, 0.042) m`，最大/最终编队 RMS `1.184/0.096 m`，实际/命令最小机间距 `1.065/0.965 m`，实际/命令最小障碍净空 `0.147/0.204 m`，实际/命令最低高度 `0.781/0.707 m`。
- planner 的 `obstacle_clearance` 已从 `0.8 m` 提到 `0.9 m`，镜像为 `sha256:6e3bd71fe135cfb39793043f64a8dabc019756d4be3c992f2a9b6535fcd9d1a0`；容器内配置已核验。
- 任务脚本按墙钟时间记录七机实际 odometry，编码器按 PPM 文件时间对齐并投影当前机位，绘制官方 12 边拓扑：中心到外圈 6 条边，加外圈六边形 6 条边；不绘制圆点、机号或历史轨迹。
- 成品为 H.264/AVC、`yuv420p`、640×360、12 fps、609 帧、50.75 秒。3 fps 原始画面在安全验收后绘制编队图并使用 FFmpeg `minterpolate` 离线插帧，不占用飞行控制周期。
- 首帧、两个中段帧和终点帧已检查，连线清晰并随实际队形形变，未发现旧轨迹、圆点、黑帧或破损。
- `verify_videos.py` 的最低分辨率门槛已调整为 640×360；CAT-ORA 和 Swarm 两份成品已同时通过 H.264、16:9、帧数及首中尾解码检查。
- 成品本地与远端 SHA-256 一致：`7d6fab0863b3d44afe92fbc504c0b403c10427054ee350aef8cb6bd16124330a`。
- 当前 planner 镜像：`sha256:6e3bd71fe135cfb39793043f64a8dabc019756d4be3c992f2a9b6535fcd9d1a0`。
- CAT-ORA 成品此前已验证：16 机圆环最大终点误差 `0.107 m`，最小机间距 `2.267 m`。

## 有效结论

- 此前碰撞的主因是软件接入，不是 Swarm-Formation 在动力学下必然失效：旧 bridge 丢弃速度/加速度前馈并改写 planner 轨迹，使 PX4 滞后、超调并在曲线路径间切角。
- 前馈修复后的多轮无录像验证与正式录制均通过实际轨迹安全门槛。上游官方 `poscmd_2_odom` 仍是 100 Hz 理想位置复制，因此 PX4 复现不应期待完全相同的瞬时轨迹。
- `max_tracking_error_m` 当前是未按消息时间戳对齐的最新命令与最新 odometry 差值；录像时仿真时间跳变会夸大它，只保留作诊断，不作为验收条件。
- Gazebo Harmonic 可用 `--headless-rendering` 发布 RGB 图像流，再编码为兼容 MP4，无需桌面会话。

## 卡住的问题

- 无。

## 下一步

- 当前任务已完成；后续若修改 planner、场景或录制参数，先运行无录像验收，再正式录制。

## 踩过的坑

- 两排树或人工 S 形航点只会形成走廊，已废弃。
- 单纯提高障碍权重或更换 seed 无法修复执行端丢失前馈造成的碰撞。
- 没有地面点云时，二维障碍场允许 planner 从接近地面的区域穿过。
- 960×540@5 Hz 录制会明显挤占无 GPU 主机的七机控制时序；640×360@3 Hz 可稳定录制。
- 640×360@5 fps 的命令/实际障碍净空为 `0.128/-0.047 m`，4 fps 为 `0.220/0.097 m`，均未通过 `>0.1 m` 门槛；提高交付帧率应在飞行结束后离线插帧。
- 纯细线首轮重录的实际障碍净空为 `-0.003 m`，被验收脚本拒绝且未覆盖成品；第二轮实际净空 `0.166 m`，用于交付。
- 官方拓扑连线在 `obstacle_clearance=0.8 m` 下两轮实际净空仅 `0.041/0.037 m`，均被验收拒绝；提高到 `0.9 m` 后实际净空为 `0.147 m`，通过验收。
- OpenCV `mp4v` 兼容性不足；交付视频需转 H.264 Baseline、`yuv420p`、faststart，并做完整解码。

## 相关指令/链接

- 远端机器：`RKOH_Remote`
- 远端工程：`/home/noah/code/UAV`
- 无录像复验：`cd /home/noah/code/UAV && SWARM_RECORD=0 ./demo/run_swarm_demo.sh`
- 正式录制：`cd /home/noah/code/UAV && ./demo/run_swarm_demo.sh`
- 远端视频：`/home/noah/code/UAV/demo/catora_px4_gazebo_headless.mp4`、`/home/noah/code/UAV/demo/swarm_formation_px4_gazebo_headless.mp4`
- 本地视频：`F:\UAV\demo\deliverables\catora_px4_gazebo_headless.mp4`、`F:\UAV\demo\deliverables\swarm_formation_px4_gazebo_headless.mp4`
- 正式指标：`F:\UAV\demo\deliverables\swarm_forest_metrics_official_graph.json`
- 完整复现说明：`docs/CAT-ORA与Swarm-Formation的PX4-Gazebo复现说明.md`
