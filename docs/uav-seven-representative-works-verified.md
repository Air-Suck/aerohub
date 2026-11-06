# 无人机五条技术路线：七篇代表工作及开源、发表与 SOTA 核验

核查日期：2026-11-04；纯视觉条目更新于 **2026-11-05**。本文独立保存最终选定的七篇工作，数量为 **3＋1＋1＋1＋1**。核查依据包括官方仓库、许可证与分支、正式出版元数据、论文方法和实验，以及有限的后续文献检索。本次未编译算法、下载全部权重或进行飞行复现。

## 一、先看结论

**七篇都有实际算法源码，七篇均正式发表于本领域主流顶会／顶刊；但不能把七篇全部认定为“许可证清晰的开源项目”，也不能把它们全部认定为“截至 2026 年仍是 SOTA”。**

“顶会顶刊”按机器人与计算机视觉领域通常采用的口径：T-RO、Science Robotics 为顶级机器人期刊，ECCV 为顶级视觉会议，IROS 为机器人领域旗舰会议。这里不是用某个机构的 CCF、分区或绩效目录替代领域评价；所有会议条目均为正式会议论文，不是 workshop。

| 技术路线 | 代表工作 | 实际源码与许可核查 | 正式发表／顶会顶刊 | 当前 SOTA 判断 |
|---|---|---|---|---|
| 单机估计：纯激光雷达 | PIN-SLAM | 有；根目录 MIT 许可证 | T-RO 2024，是 | 发表时定位有竞争力、神经地图有优势；当前整体最优未证实 |
| 单机估计：纯视觉 | DPV-SLAM | 有；根目录 MIT 许可证 | ECCV 2024，是 | 原论文支持精度、帧率与显存的良好折中，TartanAir 对比领先；当前整体最优未证实 |
| 单机估计：惯性融合 | FAST-LIVO2 | 有；根目录 GPLv2 许可证 | T-RO 2025，是 | 原论文多模态定位对比领先；截至核查日仍领先未证实 |
| 多机协同定位与全局一致性 | D²SLAM | 有；README 明确 LGPL-3，未见独立许可证正文 | T-RO 2024，是 | 集群协同定位代表，原文部分指标领先；当前全面最优未证实 |
| 地图支持避障与轨迹优化 | ROG-Map | 有；根目录 GPLv3 许可证 | IROS 2024，是 | 局部占据更新／膨胀效率有领先证据；不能泛化为所有规划地图最优 |
| 地图支持探索与感知决策 | FALCON | 有；`main` 有 GPLv3，但实际代码分支缺少同一许可文件，适用范围待确认 | T-RO 2025，是 | 原论文探索基准领先；当前所有探索任务最优未证实 |
| 动态与未知环境安全飞行 | SUPER | 有；未发现覆盖主体的明确项目许可，不能确认严格意义的开源状态 | Science Robotics 2025，是 | 高速未知环境飞行强代表；当前综合最优未证实，且有正式勘误 |

按证据强度统计：**4 项有根目录许可证与主体代码同分支的直接证据；D²SLAM 有作者明确的开源许可声明；FALCON 有跨分支许可歧义；SUPER 的主体许可未明确。** 这不是对所有第三方依赖、模型权重或数据集许可证的完整审计。

关于 SOTA，本次没有足够证据给任何一篇盖上“截至核查日，该整条路线仍最优”的章。这是“尚未证实”，不等于断言它们已全部落后。纯视觉代表按用户选型采用 DPV-SLAM：其优势是精度与计算资源的折中，而非所有数据集上的最低轨迹误差。

## 二、路线一：单机状态估计与 SLAM

这里“纯雷达”专指纯激光雷达（LiDAR），不是毫米波雷达；“纯”指定位所需的传感器输入，和是否使用神经网络无关。惯性融合指 IMU 与外部传感器融合，不是只用 IMU 的纯惯性导航。

### 1. PIN-SLAM：纯激光雷达代表

**论文：** *PIN-SLAM: LiDAR SLAM Using a Point-Based Implicit Neural Representation for Achieving Global Map Consistency*。

- **正式发表：** IEEE Transactions on Robotics，2024，卷 40，4045–4064。[正式 DOI](https://doi.org/10.1109/TRO.2024.3422055) · [论文全文](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/pan2024tro.pdf)。本次按完整标题核对 Crossref 与作者引用，确认是 T-RO 正式论文。
- **开源：** [官方仓库](https://github.com/PRBonn/PIN_SLAM)，`main` 分支有 [MIT 许可证](https://github.com/PRBonn/PIN_SLAM/blob/main/LICENSE)、`pin_slam.py`、`model`、`utils`、数据配置和评估入口。属于有实际实现与明确许可的开源项目。
- **输入与输出：** 选用激光点云配置，输出位姿、经回环修正的轨迹及神经地图。论文首页示例明确不使用 GNSS、IMU 或轮式里程计。它也支持其他测距输入，不应反过来写成“只能接激光雷达”。
- **通俗实现：** 在环境中放一些保存局部形状信息的“神经点”。小网络读取这些信息，估计周围表面的位置。新点云到来时，先调整自身位置和朝向，让观测贴合地图，再更新地图。回到旧地点发现漂移后，修正轨迹，神经点也随之移动，使前后建的地图重新对齐。
- **性能与 SOTA 证据：** 原论文明确称定位精度“better or on par with”已有领先激光里程计／SLAM，并在其比较范围内优于近期神经隐式 SLAM。这个措辞本身就不等于所有定位指标第一。其地图重建结果也不能替代位姿精度排名。
- **当前判断：** 可以称“有竞争力的纯激光神经 SLAM 代表”；不能据 2024 年实验认定它是 2026 年纯激光定位精度或机载效率最优。通常需要 GPU，主要验证平台也不等于轻型无人机闭环飞控部署。

### 2. DPV-SLAM：纯视觉代表

**论文：** *Deep Patch Visual SLAM*，Lahav Lipson、Zachary Teed、Jia Deng。

- **正式发表：** ECCV 2024，正式会议论文。[ECVA 正式会议入口](https://www.ecva.net/papers/eccv_2024/papers_ECCV/html/272_ECCV_2024_paper.php) · [会议论文全文](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/00272.pdf)。ECCV 是计算机视觉领域顶级会议，不是 workshop。不要与前作 *Deep Patch Visual Odometry*（DPVO，NeurIPS 2023）混用标题与年份。
- **开源：** [官方仓库 princeton-vl/DPVO](https://github.com/princeton-vl/DPVO)同时承载 DPVO 与 DPV-SLAM。`main` 分支有 [MIT 许可证](https://github.com/princeton-vl/DPVO/blob/main/LICENSE)、`dpvo` 算法源码、`DPRetrieval`、训练与评测脚本及模型下载入口；README 明确提供 SLAM 后端启用方式。主体源码与许可证已确认，不能因仓库名叫 DPVO 就判断没有 SLAM 实现；第三方依赖和权重仍需按各自条款使用。
- **输入与输出：** 单目 RGB 图像与相机标定，不使用激光或 IMU；输出相机轨迹及稀疏三维结构。单目尺度自由度需要单独处理，不能把尺度对齐后的评测轨迹直接当作可供飞控使用的公制位姿；也不直接提供稠密避障地图。
- **通俗实现：** 每帧只跟踪少量小图像块，用学习到的匹配信息和几何优化估计运动与深度。发现当前位置接近历史轨迹时，在图中补上跨时间联系，通过全局束调整一起修正历史位姿和深度。增强版再利用图像检索寻找较大的回环，通过相似变换位姿图优化修正尺度漂移。
- **主要贡献与版本区别：** 在 DPVO 上增加低开销的邻近回环和 CUDA 块稀疏全局束调整，形成 **DPV-SLAM**；同时启用传统图像检索回环与位姿图优化的是 **DPV-SLAM++**。论文利用图像块表示减少历史特征存储成本，重点改善速度、显存与全局一致性的折中。两种 SLAM 配置属于同一篇论文，不重复计数。
- **性能与 SOTA 证据：** 原论文表 3 的 **EuRoC 单目**评测中，DPV-SLAM 平均 ATE 为 **0.023 m**、**50 FPS**、约 **5 GB** 显存；DROID-SLAM 为 **0.022 m、20 FPS、20 GB**。这支持“精度接近、速度约 2.5 倍、显存约四分之一”，不支持“定位精度第一”。表 4 的 TartanAir 单目比较中，DPV-SLAM 平均 ATE **0.16 m**，优于表中 DPVO 的 **0.17 m** 和 DROID-SLAM 的 **0.24 m**；DPV-SLAM++ 为 **0.21 m**，增强版并非每个基准都更好。计时使用 **RTX 3090**，不能直接移用为机载板端性能。
- **与 GO-SLAM 的取舍：** 原论文表 1 的 TUM 单目结果中，DPV-SLAM 为 **0.076 m、30 FPS、4 GB**，DPV-SLAM++ 为 **0.054 m、30 FPS、6 GB**；GO-SLAM 为 **0.035 m、6.4 FPS、7.2 GB**，其中 GO-SLAM 计时按 tracking/mono 模式测量。本清单选择 DPV-SLAM，体现对实时性和资源成本的重视，并不抹去 GO-SLAM 在该表中的轨迹精度优势。
- **实现入口：** 官方 `demo.py` 或 `evaluate_*.py` 添加 `--opts LOOP_CLOSURE True` 启用 DPV-SLAM。若使用 DPV-SLAM++，先按 README 安装传统后端依赖，再使用 `--opts LOOP_CLOSURE True CLASSIC_LOOP_CLOSURE True` 同时启用两种回环。只运行默认 DPVO 配置不能算复现了 DPV-SLAM。
- **无人机适用性与当前判断：** 有 EuRoC MAV 数据集验证，适合作为低显存学习式单目 SLAM 的研究与工程候选；数据集评测不等于机载闭环实飞。仍需要 GPU，论文指出大场景全局束调整的扩展瓶颈以及图像检索误回环风险。可称“原论文有特定基准领先证据、精度与效率折中突出的纯视觉代表”；截至 **2026-11-05**，未证实它仍是该路线的全面 SOTA。

### 3. FAST-LIVO2：惯性融合代表

**论文：** *FAST-LIVO2: Fast, Direct LiDAR–Inertial–Visual Odometry*。

- **正式发表：** IEEE Transactions on Robotics，2025，卷 41，326–346。[正式 DOI](https://doi.org/10.1109/TRO.2024.3502198) · [论文全文](https://arxiv.org/abs/2408.14035)。2024 是接收／DOI 年份，正式卷年为 2025。
- **开源：** [官方仓库](https://github.com/hku-mars/FAST-LIVO2)，`main` 分支有 [GPLv2 许可证](https://github.com/hku-mars/FAST-LIVO2/blob/main/LICENSE)、`src`、`include`、配置及 ROS 启动文件；README 记录 2025-01-23 发布代码。主体源码与许可已确认。
- **输入与输出：** 激光雷达＋相机＋IMU，输出位姿、运动状态与融合地图；核心是里程计，不应默认包含完整全局回环。
- **通俗实现：** 先用 IMU 预测运动，再用激光看到的形状和相机看到的图像修正预测。地图同时保存表面形状与局部图像，让两种观测互相补充；还处理曝光变化，减少画面变亮或变暗带来的误差。
- **性能与飞行证据：** 原论文表 II 在 Hilti’22、NTU VIRAL 等序列比较定位误差，并与 FAST-LIO2、R3LIVE、LVI-SAM、FAST-LIVO 等对照；论文还提供机载状态估计、规划与控制共同运行的自主飞行实验。但表中不是每条序列、每个变体都最优。
- **当前判断：** 是三模态惯性融合的强代表，原论文支持其精度、鲁棒性和效率优势；没有足够证据确认它截至 2026 年仍全面领先。检索已找到 [FAST-LIVO2 on Resource-Constrained Platforms（RA-L 2025）](https://doi.org/10.1109/LRA.2025.3581125)，说明机载资源折中仍在演进；本次只核实该后续工作的出版记录，不据标题判定它已超过原方法。

## 三、路线二：多机协同定位与全局一致性

### 4. D²SLAM：无人机集群协同定位代表

**论文：** *D²SLAM: Decentralized and Distributed Collaborative Visual-Inertial SLAM System for Aerial Swarm*。

- **正式发表：** IEEE Transactions on Robotics，2024，卷 40，3445–3464。[正式 DOI](https://doi.org/10.1109/TRO.2024.3422003) · [论文全文](https://arxiv.org/abs/2211.01538)。正式出版信息已核对。
- **开源：** [官方仓库](https://github.com/HKUST-Aerial-Robotics/D2SLAM)公开 `d2vins`、`d2pgo`、`d2frontend`、通信模块、Docker 和运行说明。[README 的 License 节](https://github.com/HKUST-Aerial-Robotics/D2SLAM#license)明确声明 **LGPL-3**。本次递归检查 `main` 的文件树，未见独立 LICENSE／COPYING 文件；不能把“有作者声明”写成“已读取完整独立许可证正文”。
- **输入与输出：** 每机使用相机与 IMU，并交换跨机观测／约束；输出各机自身状态、相对关系与全局一致轨迹。D²VINS 和 D²PGO 是系统组成部分，不重复计数。
- **通俗实现：** 各机先自己估计运动，再把看到的共同环境联系起来。近距离时重点提高“我在你哪里”的精度；大范围飞行时利用轨迹联系和回环修正漂移，使多架无人机的轨迹对齐。计算分布在各机上，不依赖一台中心计算机。
- **性能与飞行证据：** 原文比较相对定位、全局一致性和分布式计算，并提供集群实飞。其“2.5 cm”结果有明确条件：TUM ROOM 前 30 秒、共同视场较好的区间，不能推广到所有实飞。部分序列的对照方法仍更优。
- **当前判断：** 可以称“面向无人机集群的代表性分布式视觉惯性 SLAM”；未建立覆盖 2025–2026 后续方法、相同传感器与通信条件的完整排名，不能断言当前全面 SOTA。共享状态也不等于已解决共享避障地图、任务分工或多机避碰。

## 四、路线三：地图支持避障与轨迹优化

### 5. ROG-Map：局部规划地图代表

**论文：** *ROG-Map: An Efficient Robocentric Occupancy Grid Map for Large-scene and High-resolution LiDAR-based Motion Planning*。

- **正式发表：** IROS 2024，8119–8125。[正式 DOI](https://doi.org/10.1109/IROS58592.2024.10802303) · [论文全文](https://arxiv.org/abs/2302.14819)。预印本为 2023 年，不应据此误写会议年份。
- **开源：** [官方仓库](https://github.com/hku-mars/ROG-Map)，`main` 有 [GPLv3 许可证](https://github.com/hku-mars/ROG-Map/blob/main/LICENSE)、`rog_map` 源码、A*／RRT* 等示例。当前 README 还明确展示 **Sliding ESDF map generation**，并说明 ESDF 模块改自 Fast-Planner。
- **输入与接口：** 点云＋外部位姿，主要提供占据、空闲、未知与膨胀障碍信息，帮助检查路径和机身是否会碰撞。论文的核心贡献是局部占据与增量膨胀；当前软件仓库另有 ESDF 功能，不能笼统说“ROG-Map 没有 ESDF”。这也不意味着论文已证明其 ESDF 全面优于专门距离场方法。
- **通俗实现：** 为无人机维护一张范围固定、随它移动的三维方格地图。离开范围的格子被复用到前方，其他数据不必整块搬动。障碍物周围留出考虑机身体积的安全边，只更新发生变化的附近区域，减少计算。
- **性能与飞行证据：** 原论文表 I 对比 OctoMap、HashMap、UniformMap、FIIMap，显示膨胀与整体更新效率优势；但概率更新和随机查询并非每项都胜过其他均匀栅格。实飞配置为 **0.05 m** 分辨率、**30×30×6 m** 地图，50 Hz 输入下平均更新 **5.96 ms**。原摘要中膨胀百分比与毫秒数有算术不一致，本文不沿用该百分比。
- **当前判断：** 在“局部高分辨率占据地图与障碍膨胀效率”这个窄任务上有较强的原论文领先证据；不能称“当前所有无人机建图／轨迹优化方法中的 SOTA”。占据查询、ESDF 梯度、动态预测解决的问题不同，不能放在单一排名里。

## 五、路线四：地图支持探索与感知决策

### 6. FALCON：地图指导自主探索代表

**论文：** *FALCON: Fast Autonomous Aerial Exploration Using Coverage Path Guidance*。

- **正式发表：** IEEE Transactions on Robotics，2025，卷 41，1365–1385。[正式 DOI](https://doi.org/10.1109/TRO.2024.3522148) · [论文全文](https://arxiv.org/abs/2407.00577)。仓库引用写 2024，正式卷年按出版元数据为 2025。
- **源码确认：** [官方仓库](https://github.com/HKUST-Aerial-Robotics/FALCON)默认分支为 [`ros1-noetic`](https://github.com/HKUST-Aerial-Robotics/FALCON/tree/ros1-noetic)，有 `falcon_planner`、`uav_simulator`、编译及启动说明。因此它已经提供实际实现，不能因另一分支的旧占位说明而判定“未发布代码”。
- **许可证需要限定：** [`main/LICENSE`](https://github.com/HKUST-Aerial-Robotics/FALCON/blob/main/LICENSE)确实是 GPLv3；但 `main` 当前只有 LICENSE 和写着“Coming soon in December, 2024”的 README。实际代码所在 `ros1-noetic` 分支未发现独立许可文件，其[核心包 package.xml](https://github.com/HKUST-Aerial-Robotics/FALCON/blob/ros1-noetic/falcon_planner/exploration_manager/package.xml)的 license 仍为 TODO。不能仅凭 `main/LICENSE` 就宣称代码分支全部模块的许可范围已明确；本文记为“源码公开，跨分支许可范围待确认”。
- **地图如何帮助飞行：** 地图中的未知边界告诉无人机哪里值得继续观察，区域连通关系帮助安排顺路的访问次序；目标是减少重复飞行、缩短探索时间。
- **通俗实现：** 像检查陌生建筑一样，先把已知空间分成可互相通行的区域，安排整体巡视顺序，再在附近挑具体观察位置并生成平滑轨迹。发现新通道后持续更新路线，避免只追逐最近的未知区域而反复折返。
- **性能与飞行证据：** 原文与 NBVP、FUEL、RACER、ECHO、FAEP 等探索器比较，并采用同一四旋翼仿真器；文中报告相应场景约 **13.81%–29.67%** 的探索效率改善，另有机载自主探索实飞。结果适用于论文的场景、覆盖目标与配置，不能推广为所有环境的固定提升。
- **当前判断：** 原论文支持其作为探索规划领先代表；未完成截至 2026 年所有后续方法的同协议比较，当前 SOTA 未证实。它代表“去哪探索”，不同时代表“怎样飞才能保护视觉定位”的全部感知约束规划。

## 六、路线五：动态与未知环境中的安全飞行

### 7. SUPER：未知空间高速安全飞行代表

**论文：** *Safety-assured high-speed navigation for MAVs*。

- **正式发表：** Science Robotics，2025-01-29，卷 10，第 98 期，eado6187。[正式 DOI](https://doi.org/10.1126/scirobotics.ado6187) · [作者提供的论文 PDF](https://github.com/hku-mars/SUPER/blob/master/misc/scirobotics.ado6187.pdf)。顶刊正式研究论文已确认。
- **源码确认：** [官方仓库](https://github.com/hku-mars/SUPER)的 `master` 分支有 `super_planner`、`rog_map`、`mission_planner`、仿真和启动入口；另有[硬件项目](https://github.com/hku-mars/SUPER-Hardware)。不是只有视频或空仓库。
- **开源结论：** 本次递归检查文件树，仅找到部分第三方库许可证，没有找到覆盖主体规划器的统一项目许可；[super_planner/package.xml](https://github.com/hku-mars/SUPER/blob/master/super_planner/package.xml)仍写 `<license>TODO</license>`。因此可以确认“源码公开”，不能确认“主体按明确开源许可证发布”。第三方库的许可证不能自动覆盖作者主体代码。
- **地图与状态估计的作用：** 论文系统使用 FAST-LIO2 提供状态与已配准点云；点云和已知空闲空间用于构建飞行安全约束。SUPER 是完整飞行系统／规划框架，不是新的独立 SLAM 前端。
- **通俗实现：** 每次规划准备两条轨迹：一条争取快速前进，可以朝还未完全看清的区域延伸；另一条留在已经确认空闲的空间内，作为可切换的安全备用。随着新观测到来持续更新，尽量在快速前进时仍保留安全退路。
- **性能与飞行证据：** 所读取原始论文报告超过 **20 m/s** 的真实自主飞行，并比较未知杂乱环境中的任务成功率、速度和规划耗时。该速度来自感知、机体、估计、规划与控制的共同作用，不是只替换地图就能得到的结果。
- **勘误检查：** 查到 2025-03-26 的[正式勘误](https://doi.org/10.1126/scirobotics.adx3476)，Crossref 与 [PubMed](https://pubmed.ncbi.nlm.nih.gov/40138487/)均确认其对应本论文。出版社页面访问受限，索引没有勘误正文；本次未能核清更正具体内容，也未确认仓库 PDF 已纳入更正。因此上述数值标明来自所读原论文，不作为已核清勘误后的最终性能结论；本文也不据失败率倍率给出当前 SOTA 认证。勘误记录本身不等于否定论文。
- **当前判断：** 可称“未知环境高速安全飞行的强代表”，不能确认当前综合最优。它主要代表未知空间安全策略，不意味着已解决动态障碍未来预测、任意漏检或任意控制误差下的绝对安全。

## 七、当前 SOTA 检索的范围与限制

本次重新读取七篇论文的摘要、方法或性能相关段落，核对官方仓库与出版记录；另对 2025-01-01 至 2026-11-04 的后续文献进行了 Crossref 关键词检索，覆盖纯激光 SLAM、单目 SLAM、LIVO、集群协同定位、占据地图、探索与未知环境导航。

这属于有针对性的证据复核，不是穷尽所有会议论文、代码排行榜与最新预印本的系统性综述。出版索引检索有召回缺口，部分 2026 记录只有年份；未找到更优结果不能作为仍居第一的证明。

此前核查的后续对照包括 MASt3R-SLAM：正式 CVPR 2025 记录与论文表 1 均已核查。2026-11-05 更新纯视觉选型时，重新读取 DPV-SLAM 的 ECVA 正式会议论文，核对表 1–4、方法与局限，以及官方仓库的 MIT 许可证和后端启用说明；没有借本次选型变更宣称完成了截至当日的全面排名。FAST-LIVO2 的资源受限平台后续论文仅作为需要进一步比较的线索，不把其出版元数据当作性能胜出的证据。对其他路线，没有拿“论文更新”或“题目相近”替代公平实验。

若要进一步认定“当前 SOTA”，至少需要固定任务、传感器、数据集、硬件、成功／失败定义和指标，再与截至指定日期的有竞争力方法比较。特别是：

- PIN-SLAM 应比较定位误差、回环与计算代价，不能只比较网格重建。
- DPV-SLAM 应固定普通版／++ 版、回环开关、图像分辨率和硬件，比较单目轨迹误差、回环收益、尺度、帧率、延迟与显存；不能把单目尺度对齐结果直接当作公制飞控位姿。
- FAST-LIVO2 应固定传感器与标定条件；D²SLAM 还要固定机群规模、通信与共同视场。
- ROG-Map 应固定分辨率、点云密度、膨胀半径与查询任务。
- FALCON 应固定覆盖目标、环境、传感器视场与动力学；SUPER 应固定安全约束、感知能力、机体与控制条件，并核清勘误。

## 八、保存与使用建议

本文件保存七篇核心工作；2026-11-05 按用户选型将纯视觉代表调整为 **DPV-SLAM**，并同步更新本文件的总览、方法说明、性能依据与使用建议。此前独立核验的其他结论继续保留：FALCON 的许可需要区分分支；ROG-Map 当前仓库包含滑动 ESDF 模块；SUPER 有正式勘误记录。

推荐将这份清单称为 **“五条路线的七篇代表工作及证据”**，不要称为“七篇全部开源且当前最优的 SOTA”。若收录标准要求“主体代码许可明确且当前 SOTA 已证实”两项同时满足，则目前这七篇不能整体通过筛选；FALCON、SUPER 需要先解决许可证据问题，各篇的当前排名仍需更严格比较。

源码分支快照（本次 API 文件树记录）：D²SLAM `main`：`4c33871083e4ae97b97973eb5bd88dc6735359d7`；FALCON `ros1-noetic`：`312eb4d32c6c7af1a482f94a0a204aa2bb150cca`；SUPER `master`：`2ad3419c127a617c6d7df6925e81a14175a9c096`。其他仓库核对 `main` 的当日状态，链接后续可能变化；复现时应另固定提交版本。
