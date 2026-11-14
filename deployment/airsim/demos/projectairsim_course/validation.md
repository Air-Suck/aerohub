# ProjectAirSim赛道验证记录

各轮结果只适用于注明的配置。当前入口与最新状态以 [README](README.md) 为准；保留历史失败，不事后改判。

## 当前验证记录（2026-11-13）

YOPO ROS四个包在固定镜像补编译通过；独立赛道前视相机/原生传感器配置加载通过。
GPU worker启动失败：当前执行环境最小 `libcuda.so.1` 的 `cuInit(0)` 返回999，
PyTorch CUDA初始化失败。`nvidia-smi`能看到RTX4090不代表CUDA可执行。
启动器在场景复位之前检查CUDA，失败即退出。未用CPU或模拟输出替代。
该段为重启前历史结果；重启后的实际验证见下节。
没有重置GPU或卸载驱动，以免影响其他工作负载。

进一步诊断已定位：`/dev/nvidia-uvm` open返回EIO；NVIDIA内核报告Xid31、
Xid154（Node Reboot Required），并明确写出`uvm encountered global fatal error 0x60,
requiring os reboot to recover`。证据`log/projectairsim/yopo-kernel-errors.txt`。
需要用户安排主机重启后再运行上述YOPO命令；仅重启场景或容器不足以恢复该内核状态。

## 重启后实际验证（2026-11-13）

CUDA张量计算、YOPO官方权重GPU加载/warmup通过。修复一次性CameraInfo晚订阅：
仅使用RELIABLE / TRANSIENT_LOCAL / depth=1标定订阅，发布端须提供匹配QoS。实际独立冷启动地面ready通过；
ROS回归在独立domain验证仅在Planner启动前发布一次标定，轨迹更新/拒绝/取消/停止门通过。

第一轮真实飞行在25–33m往复，取消后停车及降落失败，停止所属SITL收口，未标记成功降落。
真实起飞深度显示机臂桨叶遮挡，前视相机前移0.5m并同步camera_x_m外参检查。
不遮罩真实近距像素，不改训练权重；该杆臂是本机体专属配置，尚未完成原生共置模型的完整等价性证明。
修正后的地面真实深度已无机臂遮挡，图见本轮ground-depth.png。

第二轮仍失败：约30–40m跟踪偏差增大，RUNNING样本最远推进55.69m，触发[-6,6]m横向边界；
ARRIVED_5M=false，stopped=false，land_success=false，最终disarmed=true（PX4独立确认）。
证据：`log/projectairsim/yopo-course-20261113-155136-859755/`。
原生碰撞话题仍未覆盖，不能凭跟踪偏差独断碰撞根因。5m/s全程避障未通过。
一键入口现可真实启动并飞行，但不声明本场景成功复现。失败退出码为1，结果与恢复字段保留。

观察与算法采集分别配置：原生Chase保持enabled、RGB streaming，供主窗口观察；
前视yopo_front仅启用DepthPlanar capture、关闭streaming，供YOPO消费。
下视相机关闭，不因关闭无消费者相机而禁用Chase。

用户确认移除YOPO任务的高度越界终止检查；仅保留x∈[-6,6]、y∈[-2,123]m检查。
历史3m/s轮在相对高度5.5011m触发旧门限，最大RUNNING推进84.30m、峰值3.276m/s；
取消后日志STOPPED，原生Land/disarmed通过。该轮仍记未完成，不能事后改判成功。


## IPC 单轮验证

2026-11-13单轮实际飞行：证据`log/projectairsim/ipc-course-20261113-170256/`。
`arrived=true, land_success=true, disarmed=true, success=true`，最终uORB arming_state1。
导航44.877仿真秒，速度中位2.9451、峰值3.6914m/s，记录终点误差0.01247m，
横向最大偏差0.80864m，高度相对地面1.929…2.131m。实际悬停油门0.59334。
所有compute子进程正常清理，无强制kill。该轮取消后直接转原生Land，未独立执行
停车驻留/跨Behavior交接专项验收；旧继承结果中的stopped=false不代表到点失败。
结果success仅表示本轮任务到点与Land/disarmed，不代表完整算法复现成功。
原生碰撞事件、真值评分、严格逐帧控制映射和故障恢复矩阵尚未验收。
原始ROS bag包含IPC求解/会话/控制/PX4模式与位姿，samples日志保留导航状态。


## DiffPhys 与标准入口整合记录

2026-11-13 历史实飞 `log/projectairsim/diffphys-course-20261113-195306`：
原生起飞约1.99m，策略接管后下降至约0.7–1.2m，约101m处用户观察撞击，任务倾角
超过0.6rad退出。Land Action未成功返回，PX4最终已上锁；不得标记到达或安全降落通过。
图像传输恢复与最终指令校验修复后能持续执行，但本场景高度/避障适配尚未完成。

当时将 DiffPhys 起飞/目标高度改为 3 m、目标速度仍为 5 m/s，修改时尚未实飞；
最近 2 m 起飞轮次出现横向越界或最终推力超限，不能视为复现成功。

## 源码基线与整合约定（2026-11-14 用户确认）

YOPO 已有验证过的标准代码，用户指定 `src/planner/YOPO/native` 为后续对齐基线；
DiffPhys 同样以既有标准实现和验证为基线。当前工作树目录为小写 `src/planner/yopo/`，
尚无该 native 路径，后续先核对目录/版本对应关系，再统一入口，不因本次审查重做模型迁移。
YOPO_SOURCE / DIFFPHYS_MODEL_DIR 临时覆盖已移除，源码与资源按标准目录定位；
服务启动复用已有 Inference 脚本。赛道仍负责其本轮worker生命周期，不表示新增常驻服务管理器。
本节记录调整方向，不表示已完成入口整合或改变既有数值验证结论。

当前 YOPO 使用 ProjectAirSim 原生 camera 类型的 DepthPlanar（image-type=1），
现由上游 front_center 模板复制为 yopo_front，分辨率320×185、HFOV90°、
机体FRD位置[0.46,0,0]、姿态[0,0,0]，保留真实相机TF；Chase只用于观察。
此前 DownCamera 模板/0.5m 配置为历史轮次，不修改其原始证据。
上游 robot_quadrotor_classic 的 front_center 同为原生前视相机，HFOV90°、
FRD位置[0.46,0,0]、分辨率256×144。当前采用相同传感器机制，配置并非逐项相同。
非零杆臂下是否与 YOPO 官方部署几何等价，需结合实际标准实现核对；仅凭未显式加入
杆臂补偿不能直接断言相机选择错误或解释历史撞击。

### front_center 模板实测（2026-11-14）

改为从上游 front_center 复制，保留0.46m前视杆臂与0.03s采集间隔，按YOPO契约设置
320×185/HFOV90°/float DepthPlanar；Planner外参校验同步0.46m。未改训练权重或算法核心。
地面未解锁检查通过：`yopo-course-20261114-185544-097038`。
3m/s真实单轮证据：`log/projectairsim/yopo-course-20261114-185617-572107/`，
success/arrived_5m/stopped/land_success/disarmed均true，最终uORB arming_state=1。
RUNNING记录跨度41.745仿真秒，速度中位2.973、峰值3.262m/s；相对地面高度约1.988–7.662m，
横向最大偏差4.159m。任务按原生5m入圈与停车判据通过，不等于恒高2m或严格终点收敛；
原生碰撞事件仍未接入，不能宣称自动无碰撞验收或完整复现通过。
结果保留set_offboard_mode释放请求拒绝记录；原生Land与最终上锁反馈均确认成功。

### 标准模型定位与启动入口实飞（2026-11-14）

移除YOPO_SOURCE/DIFFPHYS_MODEL_DIR后，使用标准子模块与模型资源目录，复用已有
Inference启动脚本实飞两轮：

- YOPO3m/s：`yopo-course-20261114-190858-790815`，success/arrived_5m/stopped/Land/disarmed通过；
  RUNNING跨度41.718simsec，速度中位2.981、峰值3.268m/s；后半程仍升高，非恒高验收。
- DiffPhys5m/s、起飞3m：`diffphys-course-20261114-191143`，真实CUDA产生14次接纳策略输出；
  记录倾角/推力decode拒绝后控制流watchdog退出，Land/disarmed通过，未到终点。

以上区分服务/执行入口已工作与导航任务通过，不改变原生碰撞事件未覆盖的边界。
