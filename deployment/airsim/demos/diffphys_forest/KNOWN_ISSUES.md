# DiffPhys 森林：问题与分析

本文件保留后续实现所需发现，不是默认部署开关或已完成方案。当前基线和运行方法见
[README](README.md)。原始运行证据保留在`log/diffphys/`，不删除、不重判历史失败。

开发期独立探针、配置gate和额外刹车调度已移除；森林任务也移除了继承自姿态测试的
航向变化诊断、未消费缓存和固定取消模式的死分支。控制链计数、未解锁输出抑制检查、
采集/推理时序日志及相机断流注入仍服务于当前验收，保留。历史result中的nonzero_yaw、
truth_yaw_change_rad等字段不属于森林验收指标，后续不再生成；原始证据保持原样。

## 验收边界与失败

| 证据目录（均在log/diffphys） | 发现及结论 |
| --- | --- |
| forest-long-35-height-lookahead10、36-strict7-repeat | 7m/s目标速度两次正常通过，实际速度中位5.90/5.75m/s |
| forest-long-37-strict8 | 8m/s约219m处Landscape碰撞，不由后续成功覆盖 |
| forest-long-38-strict8-lookahead5 | 正常任务通过，但5m lookahead曾隐式限速，实际中位4.578m/s；不算8m/s性能对齐 |
| forest-long-39-takeoff4 | 接手后localization stale，仅3次策略调用；不能证明导航能力 |
| forest-long-40-takeoff4-repeat | 8m/s目标速度、4m起飞正常任务通过，当前保留基线；未证明重复可靠性 |
| forest-long-41-speed10 | 只将40轮目标速度提高至10，95.486m路径处植被碰撞；实际中位7.207/峰值9.003m/s |
| forest-long-20-speed7-camera-loss | 约7m/s时断流，未满足原10墙钟秒恢复门槛，超时速度.358m/s；保留失败 |

第40轮起飞接手高度3.944m，但前50–200m高度中位约2.23–2.37m。提高起飞高度没有
让后续航段保持4m，单轮成功也不能确定由起点提高造成。上游高速观测没有绝对高度
或高度误差，目标垂直速度为0；当前有限终点引导是部署适配，不额外叠加航段定高环。
lookahead方向和目标速度已解耦；短前视历史结果保持原判断。额外目标刹车调度未完成
真实任务验证，本次移除该实验参数与实现，仅保留问题记录。

## 相机与时延

早期long22–25“源图间隔”实际只测接收端，不能据此认定采集停顿。
long26源图15.03Hz但BEST_EFFORT接收丢91帧；改RELIABLE、关闭无消费者LiDAR与
重复采集后，long27覆盖窗口1608/1608接收。long27仍出现一次未定位的执行更新中断。
long41策略窗口15.01Hz、无内部接收漏帧，推理求解最大30ms、HTTP/服务往返最大28.39ms，
实测RTF约.997。没有证据把该轮碰撞归因为秒级推理长尾。

相机前移至X=.5m是避免当前PX4机体近场返回的平台适配。官方swarm相机X=.25m
在当前机体初始图像出现56个<1.2m返回，前移后为0；不能据此证明其他场景遮挡等价。
现有源相机HFOV91°约定已由Flightmare桥转换及反投影焦距核对。

## 深度源差异

官方v1.0归档中`src/agile_autonomy/data_generation/agile_autonomy/parameters/flightmare.yaml`
启用640×480、HFOV91°、baseline .1m、perform_sgm。`src/flightmare_bridge.cpp`
生成8位整数视差；0/255记无效，按Z=bf/d乘1000截断为uint16，以mono16发布sgm_depth。
`src/e2e_planner_v2/src/PlannerBase.py`转米，0转24m，clip[.3,24]，3/Z-.6，
82%裁剪、nearest36×48、pool3。当前DepthPlanar的光轴深度语义、米单位和预处理
已核对，不应再除1000；它仍不是双目SGM生成分布。

官方f≈314.463px，bf≈31.446m·px；整数视差1/2/3/4对应31.446/15.723/10.482/7.862m，
31.446m在网络前截到24m。连续理想深度没有这套离散档位，也不包含匹配孔洞与误差。
对long41六张真实帧仅模拟毫米传输截断，最终张量最大绝对差.005516；其余五张
非碰撞时刻帧最大.000569。这不是SGM复现，也未证明策略输出影响。
证据在`log/diffphys/depth-alignment/`，包括参考索引、复算脚本和real-frame-audit.json。

后续可先离线隔离整数视差量化对策略输出的影响；此实验尚未实施，不模拟孔洞或误匹配。
完整对齐需同步双目RGB和官方GPU SGM，比较有效率、分距离误差、最终张量与实时开销。
不把OpenCV SGBM或理想深度加噪声称为相同实现。当前不新增双目采集或SGM依赖。

## 控制器与物理模型

完整官方归档恢复的simulation.launch→base_quad_simulation.launch明确加载
e2e_planner_v2/parameters/rpg_rotors_interface_ours.yaml：低层500Hz、姿态gain13、
rate Pxy .2/Dxy .005。机体沿agile_autonomy/resources/urdf的hummingbird链加载，
body .68kg、rotor每个.009kg、arm .17m，motor升/降12.5/25ms、max838rad/s。
控制器mass参数.73kg与机身质量分别记录，完整质量应连同附加link核算。
simple.world为1ms物理步长、配置RTF .333、update_rate333；配置值不是实测RTF。
当前按用户要求保持1倍。

PX4固定版本61907962c9220437ec01a98fa26bfbf5cbb254b5含二阶姿态参考模型。
configuration-alignment/runtime-02未解锁快照确认SYS_AUTOSTART10016、roll/pitch P4，
MC_REF_FF1、WN50、FFmax100deg/s；rate P/I/D .15/.2/.003、max220deg/s、hover .5。
当前iris分配器几何与AirSim实际机体的一致性未证明。本地GenericQuad源码1kg/.2275m
不能替代预编译场景的构建来源或实测动力学。不同控制器增益不能逐值照抄。

long41策略→ROS侧PX4姿态设定值发布中位32.4ms；发布→估计姿态最优时移约100–125ms。
该时移混合物理、控制、传输及估计，ROS发布也不是固件接收确认。上游公开训练源码
自身包含约两拍动作延后和一阶响应，但checkpoint训练配方未确认。因此先前将约150ms
定为主要瓶颈的判断已撤回。AS2在ATTITUDE阶段确认bypass，无额外位置PID串联。

下一步控制调查应独立采集小幅姿态/推力响应、uORB设定值、角速度和真值，验证推力
动态映射、执行包络及状态估计误差后再决定调参。尚未实施，不宣称控制是主要瓶颈。
官方真实森林最高20m/s声明、发布测试4/7/10/13m/s配置均不代表本平台已验证能力。
