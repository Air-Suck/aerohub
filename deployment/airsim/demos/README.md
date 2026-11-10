# Library 端到端 Demo

本目录集中维护各 Library 从真实输入、原生算法、Planner、AS2 到 PX4/AirSim
执行与结果核验的示例。开发者从这里选择可运行组合，再进入对应 Library 修改算法和适配。

| Demo | Library | 运行拓扑 | 入口与验收说明 |
| --- | --- | --- | --- |
| [CAT-ORA 空旷场地](catora_open_field/README.md) | `cat_ora` | PC 四机、RoboPi 四机、三 PC 加一 RoboPi | 正常任务、主动取消、实际输出失流；各拓扑范围见详情 |
| [Swarm 空旷场地](swarm_open_field/README.md) | `swarm_formation` | 两 PC 加一 RoboPi，三机真实 LiDAR | 地面输入检查及完整任务；不代表默认七机验收 |
| [SUPER forest8](super_forest/README.md) | SUPER | 四方：工作站 Mission → RoboPi SUPER/AS2 → PC PX4/AirSim；保留旧模式 | 四方地面/短程通过；原 forest8 更新拒绝，详见 Validation |

## 从仓库根目录运行

先完成对应 README 的镜像、AirSim 场景、设备与 `.env` 配置。下面的地面检查
不解锁；去掉 `--preflight-only` 会实际发送仿真飞行指令。

```bash
bash tools/robopi/build_lib.sh catora
python3 deployment/airsim/demos/catora_open_field/run_robopi.py --preflight-only --output log/airsim/catora-ground-new

bash tools/robopi/build_lib.sh swarm-formation
python3 deployment/airsim/demos/swarm_open_field/run_robopi.py --preflight-only --output log/airsim/swarm-ground-new
```

两组示例共享部分端口与板端资源，须串行运行。完整运行、PC 构建、混合部署、
停止与故障验证命令统一记录在各 Demo 的 README 中。

## 新增 Demo 的约定

- 每个示例放在 `deployment/airsim/demos/<library>_<scenario>/`，包含 README、运行入口、任务与场景配置。
- README 声明前置资源、运行拓扑、地面检查、完整运行、停止方式、通过条件和未覆盖范围。
- Demo 负责场景、任务编排、控制权交接、观测和验收；可复用算法、输入输出转换及
  算法专有 adapter 放在 `src/planner/<name>/ros2_adapter/`。公共执行能力归 `src/behavior/` 和 AS2。
- 通用构建、部署与设备工具继续放在 `tools/robopi/`、`deployment/`；完整示例入口放在本目录，
  不在这些工具目录中另起一套任务。
- 结果、日志、截图及生成配置写入 `log/airsim/`，不得覆盖已有成功或失败记录。
- 在根目录 `deployment/airsim/libraries.yaml` 的对应 binding 中登记 Demo 文档与验收索引，并更新本表。
- 不为运行 Demo 放宽 AS2 的参考新鲜度、失流保护或绕过控制权交接。

Swarm 当前复用 CAT-ORA Demo 的部分任务与测量辅助代码，尚未提取为公共模块。
