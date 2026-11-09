# SUPER 原生 ROS 2 源码

本目录是实际构建、运行及后续修改的主仓库源码，与 CAT-ORA 的 native/bridge 组织一致。
`../../upstream/` 子模块只作上游参考，构建不依赖其初始化或本地修改。

保留三个实际依赖包：`super_planner/`、`rog_map/`、`mars_uav_sim/mars_quadrotor_msgs/`。
原有独立仿真器、图片/场景资源和 mission_planner 不在本次运行链，未复制到本目录。
这些包保留原 ROS 包名、接口、文件头、嵌套许可证与配置，不重新授权。
上游说明全文见 UPSTREAM_README.md；来源、参考提交与导入时逐文件哈希见 SOURCE.json。

导入内容来自参考3bfa403及已验证的原生修复148116e：未占据起点保留实测位置，
占据/越界起点仍走原搜索。未引入第二套 Behavior 或控制桥。后续功能修复直接修改本目录；
SOURCE.json 的 import_sha256 是导入基线，后续修改由主仓库 Git 历史追溯。

构建入口为 deployment/airsim/demos/super_forest/Dockerfile，native阶段单独编译三个包；
AS2适配包位于相邻bridge目录，运行和验收方式见 ../../../../../deployment/airsim/demos/super_forest/README.md。
