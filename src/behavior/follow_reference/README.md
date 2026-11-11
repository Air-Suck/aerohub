# 外部角速度／姿态与推力 FollowReference 插件

本包扩展原生 `FollowReferenceBase`，保留 `as2_msgs/action/FollowReference`，不新增导航 Behavior 或 Tool。算法、地图、MPC、推力估计和控制转换留在外部 Robotic Functions；此包只管理执行会话和运动参考。已通过接口验证及独立 [AirSim + flight_board 短程控制闭环](../../../deployment/airsim/demos/ipc_control/README.md)；真实IPC自由XYZ、SUPER同起终点约238m森林已以名义3m/s两次完整通过；旧20m定高结果仅作历史，范围及失败历史见 [IPC validation](../../planner/ipc/validation.md)；板端及真机尚未验收。

## 构建与加载

使用匹配当前 AS2 构建的共享工具镜像和外部 overlay；不重建镜像。已验证环境为 `uav/apps-super-forest:repo-native-20261109`，AS2 安装在 `/root/aerostack2_ws/install`。准确 image ID 随验证脚本保存。复验：

```bash
bash src/behavior/follow_reference/tests/verify.sh
```

可将持久构建工作区作为第一个参数传入。源码从本仓库只读挂载，编译产物及每次日志写入该工作区。脚本在无网络容器内运行真实 AS2 宿主与插件，模拟算法、状态和控制器服务，不连接 PX4。

加载本包 overlay 后，运行以下入口默认禁止执行目标：

```bash
ros2 launch uav_follow_reference_acro external_acro.launch.py namespace:=drone0
```

pluginlib 类为 `follow_reference_plugin_external_acro::Plugin`，宿主参数 `plugin_name=follow_reference_plugin_external_acro`。宿主复用原 FollowReferenceBehavior 类，采用双线程事件驱动 executor，默认组串行执行、独立组处理模式反馈；原 CLI 宿主的30Hz spinLoop不适合直接承担100Hz控制流。配置 `run_frequency=100.0` 仅设定执行定时器，不等于已测得100Hz全链闭环。

## 姿态模式扩展（2026-11-11）

部署时通过 `control_mode:=body_rates|attitude` 选择模式，默认body_rates。模式在宿主启动后固定，`ControlSession.control_mode`明确公布；每个`ControlFrame.control_mode`须匹配。未知模式或会话内不匹配帧拒绝且不刷新有效期。模式改变须停止、重启并重新接管，不能在流中隐式切换。

- BODY_RATES=0：保留原body_rates(rad/s)＋thrust_newtons(N)语义；attitude字段忽略。
- ATTITUDE=1：使用`attitude`四元数与`attitude_frame_id`，表示固定ENU规划系中FLU机体的目标姿态；thrust_newtons仍为牛顿。`header`保持计算时刻和FLU机体系，供推力和时效契约使用。四元数须有限且模平方与1偏差<1e-3，目标姿态frame须等于配置planning_frame；body_rates字段忽略。

未激活字段允许存在，不根据其值推断模式。新增消息字段保持源码默认行为，但改变ROS类型定义：**所有发布者、订阅者及IPC等消费者必须在同一overlay重新生成消息／重编译**；不宣称旧二进制DDS兼容。

```bash
ros2 launch uav_follow_reference_acro external_acro.launch.py \
  namespace:=drone0 control_mode:=attitude output_enabled:=true use_sim_time:=true
```

姿态模式输出同源stamp的`motion_reference/pose`和`motion_reference/thrust`，请求ATTITUDE/YAW_ANGLE。控制器将姿态转换到本地odom系，再由Pixhawk平台完成ENU/FLU→NED/FRD和推力归一化。必须显式加载[姿态验证overlay](../../../deployment/airsim/demos/attitude_control/README.md)，其中两项附加补丁在控制器和平台执行配对、新鲜度和四元数校验、保留源时间；共享镜像不变。

该模式保留FollowReference目标/约束会话，接受FIXED_YAW；ATTITUDE模式还显式接受YAW_FROM_TOPIC，由外部完整姿态流确定航向。BODY_RATES模式仍拒绝YAW_FROM_TOPIC，其他yaw模式不自动兼容。插件不增加位置或姿态反馈控制器，也没有跨Behavior资源仲裁。原角速度和姿态接口回归均通过，真实平台配对测试通过；有屏AirSim/PX4验证范围见上述独立Demo，DiffPhys算法任务证据另见其森林Demo。

launch参数`solve_timeout`默认0.1秒；15Hz策略可显式选择0.2秒求解有效期，不更改独立控制流watchdog或刷新旧求解时间。超时设置必须与任务输入新鲜度及实际推理预算共同核查。

## 接口

| 接口（相对车辆命名空间） | 内容 |
| --- | --- |
| `FollowReferenceBehavior` Action | 原生目标、yaw、逐轴速度上限及生命周期 |
| `external_acro/session` | `ControlSession`，可靠/transient-local深度1，包含目标、约束、会话ID、revision和active |
| `external_acro/control` | `ControlFrame`，best-effort深度1，一个消息携带选定模式的完整角速度／姿态和推力 |
| `motion_reference/twist`、`motion_reference/thrust` | 经ACROMotion输出给AS2控制器；输入模式BODY_RATES/YAW_SPEED |

角速度入口仅接受FIXED_YAW，姿态入口另接受上述YAW_FROM_TOPIC；目标必须位于`external_acro.planning_frame`（默认AS2 earth），移动目标系仍拒绝。max_speed是该固定规划系的逐轴上限；外部算法必须按完整session确认并执行，不能只回显ID忽略约束。IPC native adapter负责目标确认及对应算法能力，不由插件求解导航。

控制帧须回显当前session_id/revision，sequence从正数递增。header为控制计算时间和车辆FLU机体系（如 `drone0/base_link`）；body_rates单位rad/s，thrust_newtons为牛顿。`last_successful_solve`保留真实最后成功求解时间，`solver_ok=false`不进入执行。超过角速度/推力界限、非有限值、旧版本、倒序、未来或陈旧数据均拒绝；拒绝不延长旧帧有效期。未提供单独ack消息，首个匹配版本且有效的控制帧表示外部算法已准备该目标版本。

激活默认由 `external_acro.output_enabled=false` 拒绝。启用前必须完成下述底层验证。有效输入出现后才请求模式；首次模式切换后丢弃切换前帧，等待新帧及ControllerInfo确认后输出。辅助session在此时重新发布更新时间，算法不能重盖旧解时间。

修改目标先撤销旧流并请求保持，再发布新revision；暂停撤销会话，不支持直接恢复，需取消后提交新Goal。跟随保持RUNNING，不自动将位置到达解释为完成。外部调用协调器判定导航任务完成后取消执行。取消/失败请求AS2保持；模式服务成功仅表示接受模式请求，不是物理停止确认。调用协调器仍须核对状态并完成后续降落/交接。

ROS时间、定位、控制器模式确认及控制流新鲜度受检查。墙钟watchdog在流已开始后检测断流和ROS时钟停顿，撤销输出并请求保持；ROS时钟冻结时Action结果可能延迟到ROS执行定时器恢复。该保护依赖进程和executor仍运行，不覆盖宿主卡死。本包宿主使用双线程executor，生命周期/状态/执行保持默认互斥组；模式反馈使用独立组和原子字段。尚未验证任意组件容器。

## 接入前置与当前边界

- IPC上游为归一化油门，不能直接填入牛顿字段。算法adapter必须明确与平台max_thrust一致的映射及标定边界。
- 控制器透传推力赋值缺口已由AirSim验证入口的独立补丁及overlay修复，并完成短程闭环。共享镜像仍保持原样，普通部署必须显式加载对应修复，不能仅打开output_enabled。
- ACROMotion输出为两个话题；验证overlay要求两个分量具有相同源时间戳并且有效，再一起输出。短程中逐样本数值匹配通过，不等于全部丢包/重排故障组合通过。
- 不允许use_actuator_commands直接平台路由；当前只验证controller motion-reference路径。
- 未实现跨Behavior资源仲裁；现有FollowPath、GoTo、起降等运动发布者须由运行时协调停止和交接。不能同时加载两个相同名称的FollowReference宿主。
- 默认参数是工程起点，不代表实机标定。当前闭环限定为独立PC AirSim/PX4控制用例，真实IPC控制门和4m导航结果单独记录，不代表RoboPi或真机通过，且实测未达到100Hz。

验证覆盖：流会话/时效单元检查、pluginlib动态加载、真实宿主启动、原Action目标映射、错误yaw拒绝、错误revision/超限/求解失败帧拒绝、正常角速度/推力参考、断流失败/保持请求、新会话、目标修改旧帧拒绝、暂停后恢复拒绝、取消后静默，以及默认关闭输出。测试控制服务为模拟端，不作为PX4消费证据。

模式仅在接管准备阶段协商；确认真实ControllerInfo后，插件直接向AS2 Motion Reference发布同时间戳角速度/推力，不在每次发送时通过第二份模式缓存重复协商。模式丢失仍立即失败并请求保持。
