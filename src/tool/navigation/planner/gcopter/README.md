# GCOPTER

当前实现并已验收未知环境的 LiDAR 局部规划：私有 ROG-Map 副本、GCOPTER 独立局部前端与滚动轨迹交接，不抽象公共地图模块。全场景先验生成及加载入口已移除。验收范围和限制见 [VALIDATION.md](VALIDATION.md)。

上游锁定 `e0444f6d47b84f972ced91746b05feb36ce1fd4f`，保留原树和 MIT 署名。上游 package.xml 的 BSD 字段与根许可证不一致，本项目按根 LICENSE 及核心逐文件许可记录 MIT。

`ros2_adapter/native` 提供可独立构建的上游核心及两个 ROS2 入口：`gcopter_local_function` 消费实时 LiDAR，独立执行局部目标/路线搜索、上游 FIRI 走廊与 GCOPTER/MINCO 优化；`gcopter_function` 保留原静止快照批处理用于基线。执行桥将完整 P/V/A/yaw 与未来执行时刻提交 FollowTimedPath，局部模式通过同一 task 的递增 revision 更新，不直接发布运动参考。契约见 [CONTRACT.md](CONTRACT.md)，实际状态见 [VALIDATION.md](VALIDATION.md)。

```bash
bash deployment/gcopter/build.sh
bash deployment/gcopter/check.sh
bash deployment/gcopter/local_check.sh # 合成ROS真实算法，模拟执行确认；不是飞行验收
# 主机 Python 需 PyYAML/msgpack；显式指定已准备的 AirSimNH 可执行文件。
log/gcopter/host-venv/bin/python deployment/gcopter/run.py --binary /path/to/AirSimNH --output log/gcopter/NEW-local --case local
log/gcopter/host-venv/bin/python deployment/gcopter/run.py --binary /path/to/AirSimNH --output log/gcopter/NEW-forest --case forest
# 历史快照基线：显式 --case short / cancel / map-reset
```

构建使用已有固定 Humble 镜像与外部 overlay；OMPL Debian 包版本固定并保存 SHA256，解包到一次性容器，不修改共享镜像。执行复用已有 SUPER 应用镜像中的 AS2 和导航输入依赖，GCOPTER 来自本次 overlay。该依赖关系不代表 SUPER 算法参与 GCOPTER 求解。

当前运行入口按四方职责分容器：有屏 AirSim 在宿主，flight_board 提供 PX4/传感器/时钟；计算 runtime 在已有机载镜像启动 AS2、导航、GCOPTER 和执行 adapter；工作站任务在固定 Humble desktop 容器中只通过 ROS 调用能力。工作站 AS2 消息从锁定子模块 `as2/aerostack2` 补编译，构建前先执行 `git submodule update --init as2/aerostack2`。局部模式由 Function 直接消费实时注册 LiDAR；保留的批处理基线通过 `gcopter/snapshot` 发布单帧快照。工作站不转发传感器流或高频控制。入口避开运行容器占用的ROS domain，在190..219中选择并记录；解锁前要求时钟和权威状态各只有一个发布者且未检测到时间回退。

Function：`gcopter/map` 接收同固定 ENU 系内单值 FLOAT32 XYZ 静态快照；拒绝大端、行 padding、非法字段和非有限点。`sensor_measurements/odom` 接收权威状态。`gcopter/health` 检查新鲜度，`gcopter/plan_once` 触发求解，`gcopter/reset` 清除输入并使执行端取消。参数 goal/map_low/map_high/resolution/inflation/rrt_timeout、物理参数与惩罚项显式可配。当前参数在启动时冻结，运行中改参数需先停止重启。

`gcopter/plan` 的 TimedPath 是未承诺执行的计划，generated_at/valid_from 相同，不能直接发送给 Behavior。执行 adapter 在新鲜状态、速度≤0.05m/s、起点偏移≤0.05m时建立新的静止执行承诺，valid_from=提交时刻+0.5s。仅支持一轮 batch；再次调用须停止确认后重启任务。新的地图快照使正在执行的旧静态地图任务退出，因此 Demo 只提交一次真实配准扫描，不持续转发实时地图。

`gcopter/enable` SetBool 接管或请求退出；以 `gcopter/state.stop_confirmed` 确认停止，服务响应不是物理停车证明。共享 FollowPath 协调器内部使用 fleet NED，adapter 显式往返转换本地 ENU；不改变原有公共坐标契约。原生/时间化 FollowPath 在宿主互斥，但跨全部 Behavior 的全局资源仲裁未实现。局部执行状态另含 accepted_revision，只有 FollowTimedPath 实际接受后才递增；失败原因通过 gcopter/failure 传到执行桥。

快照批处理基线沿用上游的未占据即自由假设；局部模式不沿用该假设，只在实时地图明确观测为自由的空间规划。无解、越界、占据端点、非有限输入或轨迹约束复查失败均不发布旧计划。求解服务使用串行 executor，优化器目前无硬超时/计算中取消。

几何安全适配与构建补丁见 `ros2_adapter/native/SOURCE.json`。原生FIRI将体素表面中心视为点障碍；本封装对走廊加入半体素对角线的保守内缩，并在求解后使用原膨胀占据图以1ms采样复查。CMake在构建目录生成上游空valid_pc访问修正和OMPL检查分辨率适配，锁定源码不变；固定凸走廊数值对照不使用走廊几何调整。

长图搜索在构建目录将OMPL运动检查间隔限制到半体素；搜索图比原碰撞图额外膨胀一体素，为走廊内缩预留净空，并检查走廊非空/连通。最终仍使用原膨胀占据图复查。此保守方案可能拒绝窄通道。

局部模式使用 `native/vendor/rog_map` 的私有地图副本（保留 LGPL 与原来源），以扫描原 stamp 匹配 navigation/odom 历史中的姿态后更新滑动概率地图。独立配置在 `native/config/local_map.yaml`：0.2m 地图、障碍膨胀1.0m；局部前端0.5m搜索栅格、8m范围、名义速度3m/s、默认目标容差1.5m。未知格不参与可达路线；精确目标未观测时仅能在容差内选择实际已观测自由终点。FIRI 对实际膨胀障碍建走廊，优化后的完整停车轨迹再按0.01s检查已观测自由及速度。该检查为离散中心线检查，不宣称连续机体扫掠证明。

首次从悬停开始，之后以已获执行确认的轨迹在未来交接时刻的 P/V/A 为新边界；求解赶不上交接时刻就丢弃结果，不改时间戳。yaw 跟随水平速度方向，低速起终点沿用邻近运动方向，并跨采样解绕；更新处由相同运动边界保持 yaw 连续。地图改变时核查旧轨迹交接前缀与新轨迹；新障碍、传感器断流或可行轨迹耗尽会请求执行停止。当前采用已观测自由范围内、终端零速度的保守方案，没有 SUPER 的未知区探索轨迹与备份轨迹，也没有复杂死胡同的全局探索策略。
