# RoboPi 迁移就绪审计

审计时间：2026-11-08T11:26:36.133387+00:00
源码基线：50be892；审计方式：源码、OHOS 工作区比较、Docker 与 HDC 只读查询。

## 结论

迁移尚未就绪。没有完成迁移后的 CAT-ORA、Swarm 或 SUPER RoboPi 端到端验收。
PC 编译和单元测试不能替代交叉编译、板端加载或飞行闭环。

## 实测

- HDC 可连接到配置的 RoboPi。
- OHOS 工作区已有接口包、framework、CAT-ORA Adapter，但与当前源码不一致。
- OHOS 工作区与安装中均无 uav_follow_path_execution。
- 板端 /data/aerohub/as2、lib/catora、lib/swarm-formation 存在部署清单，
  三处均未发现 install/uav_follow_path_execution。
- PC 仍有运行两天的 SUPER inputs、clock、vehicle 和 trajectory adapter 容器。
  本审计没有停止、替换它们，也未将它们视为新迁移验证。

## 静态缺口

| 对象 | 当前情况 | 缺少的迁移验证 |
| --- | --- | --- |
| 公共 AS2 | 构建/打包入口已加入扩展，但没有实际目标产物 | 当前源码同步、OHOS 编译、ABI/依赖、部署哈希、板端 Action 加载 |
| CAT-ORA | 应用显式 legacy；板端入口只选 takeoff,land | Action 编排、停止反馈、四机 PC/板端/混合拓扑 |
| Swarm | 复用旧应用编排；板端入口只选 takeoff,land | 滚动 Action 更新、真实地图健康失效、三机混合拓扑 |
| SUPER | 保留旧执行桥与历史任务，缺少统一构建部署启动入口 | 可复现入口、主/备用轨迹语义、板端执行与长程复验 |
| 板端 smoke | verify_lib 未覆盖 FollowTimedPath | 类型支持导入、扩展启动、Goal/Feedback/Result/取消 |

## 验收顺序

1. 先完成独立 OHOS 构建与产物检查；不覆盖现有消费者正在使用的安装。
2. 在隔离 ROS 域完成新扩展的板端无飞行接口验证，不能用旧 smoke 代替。
3. 迁移 CAT-ORA 应用与板端启动器，验证正常、取消、断流以及停止后降落。
4. Swarm 验证滚动时序、地图失效、单机拒绝后的机群收口和原三机混合场景。
5. 补齐 SUPER 环境入口与主/备用轨迹执行映射，再复验 forest8。
6. 每一组合记录当前主仓库/子模块、镜像/安装哈希、命令和证据；旧摘要仅作历史比较。

本次未执行构建同步、板端写入、服务请求、解锁或飞行；上述检查不是 smoke 或闭环通过声明。
