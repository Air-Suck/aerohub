# FollowReference 姿态＋推力 AirSim/PX4 验证

验证现有插件的ATTITUDE模式，经AS2控制器与Pixhawk平台实际进入PX4姿态环。控制源为显式的验证用位置反馈器，输出目标姿态与推力；没有外部姿态到角速度反馈环，没有运行DiffPhys模型，不作为DiffPhys导航/避障验收。

## 复验

复用现有稳定镜像和IPC控制测试的独立部署／恢复流程，不重建镜像。每次选择新证据目录；不会停止其他场景。宿主Python需要PyYAML与msgpack（场景管理器要求）；图形环境须提供真实DISPLAY/XAUTHORITY。

```bash
bash deployment/airsim/demos/attitude_control/build_overlay.sh
AIRSIM_RENDER_MODE=windowed python3 deployment/airsim/demos/attitude_control/run.py \
  --output log/diffphys/attitude-fresh-loss
python3 deployment/airsim/demos/attitude_control/analyze.py log/diffphys/attitude-fresh-loss
AIRSIM_RENDER_MODE=windowed python3 deployment/airsim/demos/attitude_control/run.py \
  --stop-mode cancel --output log/diffphys/attitude-fresh-cancel
```

默认overlay在`log/diffphys/attitude-overlay`；同一目录不可同时构建或测试。首次准备依次加载已有三个BODY_RATES修复，然后应用`as2-attitude-pairing`、`pixhawk-attitude-pairing`两项有哈希清单的补丁；重复执行校验已应用源码，不覆盖未知改动。控制器、平台、插件及消息一起补编译，原有消费者需使用一致消息定义重编译。

实际平台类的配对回归覆盖姿态和推力先后到达、非法四元数、过期样本与正常配对；插件接口回归另验证原角速度入口、新姿态入口、未知模式拒绝和未激活字段忽略。姿态模式不直接路由actuator，仍经过控制器运动参考通路。

部署沿用IPC控制测试的PC四角色共置：宿主AirSim；flight_board提供PX4/clock；独立算力进程提供AS2和控制源；任务进程仅调用ROS和只读真值评分。域182、instance72/system73、端口占用均预检；资源详情逐轮记录。不代表跨物理设备或RoboPi。

姿态验证关闭不消费的相机采集桥，仍使用有屏GPU渲染、真实PX4/AS2状态。默认时钟桥请求100Hz，并限制数值库线程；这不是100Hz闭环性能保证。位置、倾角、停止和100ms控制流门槛均未放宽。

## 判据与边界

预设：起飞至约2m后接管并保持5秒墙钟，漂移≤0.75m、倾角≤0.35rad；AirSim NED yaw变化在-0.4至-0.03rad；断流或主动取消后恢复保持，速度≤0.5m/s；无新增飞行碰撞，原生Land及disarmed确认。PX4内部须Offboard、姿态/角速度环开启、位置/速度环关闭。

`analyze.py`按同源stamp核对AS2成对姿态/推力，再检查PX4四元数ENU/FLU→NED/FRD和推力按max_thrust=15归一化，容差1e-5。PX4使用墙钟±0.5s内的数值匹配、允许q和-q等价，不声称唯一序号对应。Hover测得的归一化推力仅是本仿真映射基准，不是真机牛顿标定。

版本化结果见`validation.json`。失败与修复记录保留在`log/diffphys/attitude-sitl-*`：01为场景管理器缺msgpack、未运动；02/03/04为100ms watchdog导致提前退出，均降落/disarmed；05完成断流场景，06完成主动取消场景；两轮均完成原生Land/disarmed及资源清理。相机桥停用后通过是时序配置结果，不证明已隔离全部因果。所有适用测试通过后才可宣称此处控制接口验收通过；DiffPhys自身视觉/推理/执行闭环、动态航向任务、多机、真机与长期可靠性未覆盖。

恢复未确认时沿用IPC的保留机制，不自动停止控制／时钟／场景；确认解除武装后可调用`deployment/airsim/demos/ipc_control/recover_ground.py --output <本轮目录>`回收本轮资源。
