# GCOPTER 验收状态（2026-11-12）

结论：GCOPTER 的未知环境 LiDAR 局部规划切片已验收。验收范围为单机、有屏 AirSimNH + PX4 SITL、约 3m/s、实时注册 LiDAR、私有滑动 ROG-Map、独立 GCOPTER 局部路线/FIRI/MINCO、FollowTimedPath/AS2 实际执行。未声明与 SUPER 规划算法等价。

## Forest 长路径

正式证据为 `log/gcopter/local-forest-super-align-02`：

- 地图来自实时 LiDAR 射线更新，不加载 PCD、AirSim 碰撞几何或其他先验地图。
- 246 个滚动计划 revision 全部被 FollowTimedPath 接受并实际执行。
- 记录航程约 251.80m，航段 117.999 仿真秒，峰值 2.9637m/s。
- 到点误差 1.3635m，满足与 SUPER forest 对齐的 1.5m 到点及速度低于 0.3m/s 持续 2 秒语义。
- yaw 跟随水平速度；运动记录航向误差中位 0.0702rad。
- 起飞后无新增 AirSim 碰撞事件；停止确认、原生 Land、disarmed 和本轮资源清理通过。

`local-forest-super-align-01` 在仿真启动前因系统 Python 缺 msgpack 退出，没有解锁或飞行，不属于算法失败。原始结果、轨迹、LiDAR、状态、飞行记录、碰撞基线、源码/工件摘要和运行日志均保存在上述独立证据目录，未纳入 Git。

## 对齐与回归

`deployment/gcopter/check_super_alignment.py` 对照现有 SUPER forest 配置，检查21项共享 ROG-Map 参数以及速度方向 yaw、1.5m 到点、低速保持、碰撞保留和 Planner/Behavior 双重完成语义。GCOPTER 使用相同的0.2m概率地图、1.0m占据膨胀、滑动/射线模型与 LiDAR 范围；规划核心和滚动策略保持独立。

同版本验证通过：

- 五个 ROS 2 包构建和核心 CTest；包含锁定上游固定走廊数值对照、障碍/无解、长图薄墙、运动 P/V/A 边界。
- `local-ros-super-alignment`：真实 ROG-Map/GCOPTER 节点在合成扫描下完成四次未来轨迹交接，并在 LiDAR 断流时停止。它是算法/接口回归，不替代飞行证据。
- 20m真实 LiDAR 飞行 `local-yaw-03`：23个 revision、峰值2.9701m/s、到点误差0.157m，停止、Land、disarmed通过。
- 快照批处理基线的正常、取消和地图 reset 链仍保留，证明锁定上游核心、ROS接口和 FollowTimedPath 停止语义。

## 已知边界

GCOPTER 复用了地图实现和执行链，但没有复制 SUPER 的探索与备份轨迹优化器，不承诺复杂死胡同的全局探索完备性。求解尚不支持计算中硬取消；迟到结果会丢弃。当前验收不覆盖多机、RoboPi、真机和完整故障矩阵。

历史开发中曾使用 AirSim 全场景碰撞图，也曾因0.4m膨胀不足出现交接阻断和一次实际碰撞；这些轮次保持在 `log/gcopter/`，不作为当前验收证据。最终版本移除了先验地图入口，将地图膨胀及 Demo/App 验收语义对齐 SUPER，并以新的独立飞行完成验收；旧失败不被改判。

复验：

```bash
bash deployment/gcopter/build.sh
bash deployment/gcopter/check.sh
bash deployment/gcopter/local_check.sh
log/gcopter/host-venv/bin/python deployment/gcopter/run.py \
  --binary /path/to/AirSimNH \
  --output log/gcopter/NEW-forest \
  --case forest
```
